#include <cassert>
#include <cmath>
#include <limits>
#include <cstdio>

#include "chassis_power.hpp"
#include "super_cap.hpp"
using namespace wdr_chassis;
extern "C" void reference_allocate(const float[4], const float[4], const float[4],
                                    float, float, float, float[4]);
extern "C" void reference_rls(float, float, float, int, float[2]);
static void near(float a, float b, float tolerance = 0.002f)
{
  if (!(std::fabs(a - b) < tolerance)) std::fprintf(stderr, "near actual=%g expected=%g tolerance=%g\n", a,b,tolerance);
  assert(std::fabs(a - b) < tolerance);
}
int main()
{
  PowerParam p;
  p.enabled = true;
  p.rls_enabled = false;
  assert(PowerControl::CheckParam(p));
  auto bad = p;
  bad.k2 = 0;
  assert(!PowerControl::CheckParam(bad));
  PowerControl controller(p);
  PowerInput input{};
  std::array<float, 4> zero{}, requested{16000, 16000, 16000, 16000};
  const auto offline =
      controller.Calculate(input, 1000, zero, zero, zero, requested, 16000, true);
  assert(!offline.cap_online && !offline.referee_online && offline.limited);
  near(offline.budget_w, 85);
  assert(offline.limited_positive_w <= offline.budget_w + 0.01f);
  for (float output : offline.limited_current)
  {
    assert(output > 0 && output < 16000);
  }
  // 冻结原车分配尾段对照：包含正反转、制动及混合正负功。
  for (unsigned n = 0; n < 10000; ++n)
  {
    std::array<float, 4> omega{}, current{}, error{};
    for (unsigned i = 0; i < 4; ++i)
    {
      const float sign = (n + i) % 2 ? -1.0f : 1.0f;
      omega[i] = (n % 3 ? sign : -sign) * float((n * 7 + i * 13) % 500);
      current[i] = sign * float(1000 + (n * 3 + i * 23) % 15000);
      error[i] = float((n + i) % 40);
    }
    const auto state =
        controller.Calculate(input, n, omega, zero, error, current, 16000, true);
    assert(!state.invalid);
    float rpm[4], expected[4];
    for (unsigned i = 0; i < 4; ++i) rpm[i] = omega[i] * 30 / 3.14159265358979323846f;
    reference_allocate(rpm, error.data(), current.data(), state.budget_w, p.k1, p.k2, expected);
    for (unsigned i = 0; i < 4; ++i)
    {
      near(state.limited_current[i], expected[i], .25f);
      assert(std::isfinite(state.limited_current[i]));
      assert(std::fabs(state.limited_current[i]) <= 16000);
    }
  }
  input.referee_limit_seen = input.referee_buffer_seen = true;
  input.referee_limit_w = 100;
  input.referee_buffer_j = 60;
  input.referee_limit_time_us = input.referee_buffer_time_us = 1000;
  auto state =
      controller.Calculate(input, 1000, zero, zero, zero, requested, 16000, true);
  assert(state.referee_online && !state.cap_online);
  near(state.budget_w, 100);
  input.cap_seen = input.cap_ok = true;
  input.cap_percent = 100;
  input.cap_input_w = 90;
  input.cap_output_w = 120;
  input.cap_time_us = 1000;
  state = controller.Calculate(input, 1000, zero, zero, zero, requested, 16000, true);
  near(state.cap_target_w, 90);  // Pin=90，k=0.5，目标95，发给超电减5W。
  assert(state.cap_online);
  assert(state.budget_w > 100);
  state = controller.Calculate(input, 101000, zero, zero, zero, requested, 16000, true);
  assert(!state.cap_online && state.referee_online);
  near(state.budget_w, 100);
  // 单独续期缓冲报文不能替过期功率上限续期。
  input.referee_buffer_time_us = 600000;
  state = controller.Calculate(input, 600000, zero, zero, zero, requested, 16000, true);
  assert(!state.referee_online);
  near(state.budget_w, 85);
  input.cap_time_us = 700000;  // 未来帧不能当作在线。
  state = controller.Calculate(input, 600000, zero, zero, zero, requested, 16000, true);
  assert(!state.cap_online);
  // 非法机械反馈清零输出，停机不能偷偷保留电流。
  auto invalid = zero;
  invalid[2] = std::numeric_limits<float>::quiet_NaN();
  state =
      controller.Calculate(input, 600000, invalid, zero, zero, requested, 16000, true);
  assert(state.invalid && state.limited_current == zero);
  state = controller.Calculate(input, 600000, zero, zero, zero, requested, 16000, false);
  assert(state.limited_current == zero);
  // RLS沿用原车每拍更新，超电失联冻结。
  p.rls_enabled = true;
  PowerControl learner(p);
  input.cap_time_us = 1000;
  std::array<float, 4> omega{100, 110, 120, 130}, measured{2000, 3000, 4000, 5000};
  state = learner.Calculate(input, 1000, omega, measured, zero, requested, 16000, true);
  assert(state.rls_updated && state.rls_updates == 1);
  state = learner.Calculate(input, 2000, omega, measured, zero, requested, 16000, true);
  assert(state.rls_updated && state.rls_updates == 2);
  input.cap_ok = false;
  input.cap_time_us = 3000;
  state = learner.Calculate(input, 3000, omega, measured, zero, requested, 16000, true);
  assert(!state.rls_updated && state.rls_updates == 2);
  // 负功保持原指令；不能拿再生功率给其他轮额外预算。
  PowerControl regen(p);
  std::array<float, 4> braking{-1000, -1000, -1000, -1000};
  omega.fill(100);
  state = regen.Calculate({}, 0, omega, zero, zero, braking, 16000, true);
  assert(state.limited_current == braking);
  // 高转速时原车允许反向电流，不能把零电流损耗误当成全域最低功率。
  omega.fill(10000);
  state = regen.Calculate({}, 0, omega, zero, zero, requested, 16000, true);
  assert(state.limited);
  for (auto value : state.limited_current)
  {
    assert(value < 0 && value >= -16000);
  }
  // RLS标准二阶更新的首拍与独立标量计算一致，并持续受原车边界约束。
  PowerControl first_rls(p);
  PowerInput sample{};
  sample.cap_seen = sample.cap_ok = true;
  sample.cap_percent = 50;
  sample.cap_output_w = 20;
  sample.cap_input_w = 20;
  sample.cap_time_us = 1;
  omega.fill(1);
  const float phi = 4;
  const float expected_k1 = std::clamp(p.k1 + p.rls_covariance * phi /
      (p.rls_lambda + p.rls_covariance * phi * phi) *
      (sample.cap_output_w - p.k3 - p.k1 * phi), p.k1_min, p.k1_max);
  state = first_rls.Calculate(sample, 1, omega, zero, zero, zero, 16000, true);
  near(state.k1, expected_k1, 0.000001f);
  near(state.k2, p.k2);
  // 连续变化的真实模型样本：对照原车更新公式，低功率样本跳过。
  PowerControl adaptive(p);
  for (unsigned n = 0; n < 2000; ++n)
  {
    float mechanical_power = 0, speed_feature = 0, torque_feature = 0;
    for (unsigned i = 0; i < 4; ++i)
    {
      omega[i] = float(10 + (n * 13 + i * 17) % 80);
      measured[i] = float(1000 + (n * 7 + i * 103) % 7000);
      const float torque = measured[i] * p.torque_coefficient;
      mechanical_power += torque * omega[i];
      speed_feature += std::fabs(omega[i]);
      torque_feature += torque * torque;
    }
    sample.cap_time_us = n * 2000;
    sample.cap_output_w = mechanical_power + .02f * speed_feature +
                          480.0f * torque_feature + p.k3;
    state = adaptive.Calculate(sample, sample.cap_time_us, omega, measured,
                               zero, requested, 16000, true);
    if (state.rls_updated)
    {
    float expected[2];
    reference_rls(speed_feature, torque_feature, sample.cap_output_w - mechanical_power - p.k3,
                  state.rls_updates == 1, expected);
    near(state.k1, expected[0], .00001f);
    near(state.k2, expected[1], .01f);
    }
    assert(std::isfinite(state.k1) && std::isfinite(state.k2));
    assert(state.k1 >= p.k1_min && state.k1 <= p.k1_max);
    assert(state.k2 >= p.k2_min && state.k2 <= p.k2_max);
  }
  assert(state.rls_updates > 500);
  // 实际CAN回调只复制，Update在任务上下文解码；时间与帧一起保存。
  LibXR::CAN can;
  SuperCap cap(&can, {});
  assert(can.sent.empty());
  LibXR::CAN::ClassicPack frame{};
  frame.id = 0x329;
  frame.dlc = 8;
  frame.data[0] = 0x23;
  frame.data[1] = 0x28;  // 90W
  frame.data[2] = 0x2E;
  frame.data[3] = 0xE0;  // 120W
  frame.data[4] = 0x0F;
  frame.data[5] = 0xA0;  // 40%
  LibXR::Timebase::now = 123456;
  can.Emit(frame);
  PowerInput received{};
  assert(!received.cap_seen);
  cap.Update(received);
  assert(received.cap_seen && received.cap_ok && received.cap_time_us == 123456);
  near(received.cap_input_w, 90);
  near(received.cap_output_w, 120);
  near(received.cap_percent, 40);
  frame.dlc = 7;
  frame.data[0] = 0;
  can.Emit(frame);
  cap.Update(received);
  near(received.cap_input_w, 90);
  cap.SendPower(95);
  assert(can.sent.back().id == 0x330 && can.sent.back().dlc == 8);
  assert(can.sent.back().data[0] == 0x25 && can.sent.back().data[1] == 0x1C);
  for (unsigned i = 2; i < 8; ++i)
  {
    assert(can.sent.back().data[i] == 0);
  }
  puts(
      "PASS: power allocation, offline fallback, frozen 26_sentry allocation/RLS, STOP and supercap "
      "CAN");
}
