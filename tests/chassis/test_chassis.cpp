#include <cassert>
#include <cmath>
#include <cstdio>
#include <limits>

#include "Chassis.hpp"
extern "C" void reference_wheel(float, float, float, float[4]);
using namespace wdr_chassis;
static void near(float a, float b, float eps = 1e-5f)
{
  if (!(std::fabs(a - b) < eps))
  {
    std::fprintf(stderr, "near: actual=%g expected=%g\n", a, b);
  }
  assert(std::fabs(a - b) < eps);
}
static void control_tests()
{
  Param p;
  assert(Control::CheckParam(p));
  Control ctrl(p);
  Feedback f{};
  f.imu_yaw = .7f;
  f.yaw_relative = .2f;
  ctrl.FeedbackUpdate(f);
  near(ctrl.GetState().chassis_yaw, .5f);
  ctrl.SetMode(Mode::NO_FOLLOW_YAW);
  ctrl.ModeTransition();
  near(ctrl.GetState().chassis_yaw_set, .5f);
  f.imu_yaw = 1;
  ctrl.FeedbackUpdate(f);
  ctrl.ModeTransition();
  near(ctrl.GetState().chassis_yaw_set, .5f);
  // 原车运动学独立冻结片段作对照；转动衰减在此关闭，验证1000组混合目标。
  f.moving_posture = true;
  for (unsigned i = 0; i < 1000; ++i)
  {
    ctrl.FeedbackUpdate(f);
    ctrl.SetMode(Mode::RAW);
    ctrl.ModeTransition();
    Command cmd{float(int(i % 17) - 8) * 100, float(int(i % 23) - 11) * 100,
                float(int(i % 29) - 14) * 50, Frame::BODY};
    ctrl.SetControl(cmd);
    ctrl.ControlLoop();
    float expected[4];
    reference_wheel(cmd.x * p.translation_comp, cmd.y * p.translation_comp, cmd.z,
                    expected);
    for (unsigned j = 0; j < 4; ++j)
    {
      assert(ctrl.GetState().current[j] == int16_t(Limit(expected[j], -16000, 16000)));
    }
  }
  ctrl.Reset();
  ctrl.FeedbackUpdate(f);
  ctrl.SetMode(Mode::NO_FOLLOW_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({1, 0, 0, Frame::BODY});
  near(ctrl.GetState().vx_set, .002f / (p.filter_tau + .002f));
  float before = ctrl.GetState().vx_set;
  ctrl.SetControl({1, 0, 0, Frame::BODY});
  near(ctrl.GetState().vx_set,
       p.filter_tau / (p.filter_tau + .002f) * before + .002f / (p.filter_tau + .002f));
  // 零目标立即清历史；随后低于旧PC死区的目标仍能产生运动给定。
  ctrl.SetControl({0, 0, 0, Frame::BODY});
  near(ctrl.GetState().vx_set, 0);
  ctrl.SetControl({.01f, 0, 0, Frame::BODY});
  near(ctrl.GetState().vx_set, .01f * .002f / (p.filter_tau + .002f));
  // 普通零目标对仍在转动的轮子保留闭环制动，RELAX才清零输出。
  Control braking(p);
  Feedback rolling{};
  rolling.rpm.fill(100);
  braking.FeedbackUpdate(rolling);
  braking.SetMode(Mode::NO_FOLLOW_YAW);
  braking.ModeTransition();
  braking.SetControl({0, 0, 0, Frame::BODY});
  braking.ControlLoop();
  for (auto current : braking.GetState().current)
  {
    assert(current < 0);
  }
  braking.Reset();
  for (auto current : braking.GetState().current)
  {
    assert(current == 0);
  }
  // 故障清历史，恢复首拍与新对象一致；NO_MOVE保持闭环制动但目标立即清零。
  ctrl.Reset();
  ctrl.FeedbackUpdate(f);
  ctrl.SetMode(Mode::NO_FOLLOW_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({1, 0, 0, Frame::BODY});
  near(ctrl.GetState().vx_set, .002f / (p.filter_tau + .002f));
  ctrl.SetMode(Mode::NO_MOVE);
  ctrl.ModeTransition();
  ctrl.SetControl({});
  ctrl.ControlLoop();
  near(ctrl.GetState().vx_set, 0);
  ctrl.SetMode(Mode::RAW);
  ctrl.ModeTransition();
  ctrl.SetControl({std::numeric_limits<float>::quiet_NaN(), 0, 0});
  ctrl.ControlLoop();
  assert(ctrl.GetState().mode == Mode::RELAX);
  for (auto v : ctrl.GetState().current)
  {
    assert(v == 0);
  }
  ctrl.Reset();
  f.yaw_relative = PI / 2;
  ctrl.FeedbackUpdate(f);
  ctrl.SetMode(Mode::NO_FOLLOW_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({1, 0, 0, Frame::GIMBAL});
  near(ctrl.GetState().vx_set, 0);
  near(ctrl.GetState().vy_set, .002f / (p.filter_tau + .002f));
  ctrl.Reset();
  ctrl.FeedbackUpdate(f);
  ctrl.SetMode(Mode::NO_FOLLOW_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({0, 0, 100, Frame::BODY});
  ctrl.ControlLoop();
  for (auto speed : ctrl.GetState().speed_set)
  {
    near(speed, 10);
  }
  for (auto current : ctrl.GetState().current)
  {
    assert(current == 16000);
  }
  ctrl.Reset();
  ctrl.FeedbackUpdate(f);
  ctrl.SetMode(Mode::FOLLOW_GIMBAL_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({0, 0, 0});
  near(ctrl.GetState().wz_set, 12);
  ctrl.SetMode(Mode::RELAX);
  ctrl.ModeTransition();
  ctrl.SetMode(Mode::FOLLOW_GIMBAL_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({0, 0, 0});
  near(ctrl.GetState().wz_set, 12);
  f.imu_yaw = std::numeric_limits<float>::infinity();
  ctrl.FeedbackUpdate(f);
  assert(ctrl.GetState().mode == Mode::RELAX);
  f.imu_yaw = 0;
  f.rpm[1] = std::numeric_limits<float>::quiet_NaN();
  ctrl.SetMode(Mode::NO_FOLLOW_YAW);
  ctrl.ModeTransition();
  ctrl.FeedbackUpdate(f);
  assert(ctrl.GetState().mode == Mode::RELAX);
  // 公共PID使用微分先行：首拍D为零，后续保留低通；故障恢复不带入旧导数。
  f.rpm = {};
  f.imu_yaw = .1f;
  f.yaw_relative = .01f;
  ctrl.Reset();
  ctrl.FeedbackUpdate(f);
  ctrl.SetMode(Mode::FOLLOW_GIMBAL_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({0, 0, 0});
  near(ctrl.GetState().wz_set, .12f);
  f.yaw_relative = .03f;
  ctrl.FeedbackUpdate(f);
  ctrl.SetControl({0, 0, 0});
  // PID原始目标0.40；保留斜坡每拍最多增加120×0.002=0.24，最终0.36。
  near(ctrl.GetState().wz_set, .36f);
  ctrl.Reset();
  ctrl.FeedbackUpdate(f);
  ctrl.SetMode(Mode::FOLLOW_GIMBAL_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({0, 0, 0});
  near(ctrl.GetState().wz_set, .36f);
  // 实际底盘跟随控制跨越正负π时走短角差，不因表示跳变输出反向饱和。
  Control cyclic_ctrl(p);
  Feedback cyclic_feedback{};
  cyclic_feedback.imu_yaw = 3.13f;
  cyclic_ctrl.FeedbackUpdate(cyclic_feedback);
  cyclic_ctrl.SetMode(Mode::FOLLOW_CHASSIS_YAW);
  cyclic_ctrl.ModeTransition();
  cyclic_ctrl.SetControl({});
  cyclic_feedback.imu_yaw = -3.13f;
  cyclic_ctrl.FeedbackUpdate(cyclic_feedback);
  cyclic_ctrl.SetControl({});
  assert(cyclic_ctrl.GetState().wz_set > 0 && cyclic_ctrl.GetState().wz_set < 1);
  near(RadFormat(PI), PI);
  near(RadFormat(-PI), -PI);
  puts(
      "PASS: 1000 frozen wheel mixes, original filter/source history, mode edge capture, "
      "zero targets and fault reset");
}
static void feed_can(LibXR::CAN& can, uint32_t id, int16_t rpm = 0, uint8_t dlc = 8)
{
  LibXR::CAN::ClassicPack p{};
  p.id = id;
  p.dlc = dlc;
  p.data[2] = uint16_t(rpm) >> 8;
  p.data[3] = rpm;
  can.Emit(p);
}
static void assert_zero(const LibXR::CAN::ClassicPack& p)
{
  assert(p.id == 0x200 && p.dlc == 8);
  for (auto v : p.data)
  {
    assert(v == 0);
  }
}
int main()
{
  control_tests();
  using namespace LibXR;
  CAN can;
  RMMotor rf(can, {RMMotor::Model::MOTOR_M3508, false, 0x201});
  RMMotor lf(can, {RMMotor::Model::MOTOR_M3508, false, 0x202});
  RMMotor lr(can, {RMMotor::Model::MOTOR_M3508, false, 0x203});
  RMMotor rr(can, {RMMotor::Model::MOTOR_M3508, false, 0x204});
  // 反馈从未收到、短帧及真实时间过期，均不能被Update次数掩盖。
  assert(!rf.FeedbackFresh(1, 10));
  feed_can(can, 0x201, 123, 7);
  assert(!rf.FeedbackFresh(1, 10));
  feed_can(can, 0x201, 123);
  rf.Update();
  assert(rf.FeedbackFresh(1, 10));
  near(rf.GetFeedback().velocity, 123);
  assert(!rf.FeedbackFresh(11, 10));
  CMD cmd;
  Chassis chassis(lr, rf, rr, lf, cmd);
  assert(can.sent.empty());  // 构造不发送。
  auto* task = Thread::threads.back();
  chassis.GetEvent().Active(uint32_t(Mode::NO_FOLLOW_YAW));
  task->Run(1);
  assert(can.sent.size() == 1);
  assert_zero(can.sent.back());
  auto status = std::any_cast<State>(Topic::values.at("chassis_status"));
  assert(status.mode == Mode::RELAX && (status.faults & YAW_LOST));
  auto refresh = [&](bool yaw = true, unsigned missing = 4)
  {
    for (unsigned i = 0; i < 4; ++i)
    {
      if (i != missing)
      {
        feed_can(can, 0x201 + i);
      }
    }
    wdr_imu::BoardState board{};
    board.yaw = .4;
    board.time_us = Timebase::now;
    Topic::CreateTopic<wdr_imu::BoardState>("board_attitude").Publish(board);
    GimbalFeedback g{};
    g.yaw_relative_rad = .1;
    g.time_us = Timebase::now;
    g.yaw_valid = yaw;
    Topic::CreateTopic<GimbalFeedback>("gimbal_chassis_feedback").Publish(g);
    CMD::Data data{};
    data.input_online = true;
    data.chassis.x = .7;
    data.chassis.z = .1;
    data.chassis.frame = CMD::ChassisFrame::BODY;
    cmd.FeedRC(CMD::RCInputSource::RC_INPUT_I6X, data);
  };
  refresh();
  task->Run(1);
  assert(can.sent.size() == 2);
  status = std::any_cast<State>(Topic::values.at("chassis_status"));
  assert(status.mode == Mode::NO_FOLLOW_YAW && status.faults == 0);
  assert(status.current[0] != 0);
  const auto first_current = status.current;
  // YAML逻辑轮序为ID203、201、204、202，发送槽必须仍按物理ID排列。
  const unsigned logical_slot[]{2, 0, 3, 1};
  for (unsigned i = 0; i < 4; ++i)
  {
    const unsigned slot = logical_slot[i];
    const int16_t sent = int16_t((uint16_t(can.sent.back().data[2 * slot]) << 8) |
                                 can.sent.back().data[2 * slot + 1]);
    assert(std::abs(int(sent) - int(status.current[i])) <= 1);
  }

  near(status.chassis_yaw, .3);
  // 等待槽已有正常反馈时发布STOP，STOP不能因ASync忽略第二帧而漏掉。
  refresh();
  GimbalFeedback urgent{};
  urgent.time_us = Timebase::now;
  urgent.yaw_valid = true;
  urgent.stop_chassis = true;
  Topic::CreateTopic<GimbalFeedback>("gimbal_chassis_feedback").Publish(urgent);
  task->Run(1);
  assert_zero(can.sent.back());
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).faults & GIMBAL_STOP);
  refresh(false);
  task->Run(1);
  assert_zero(can.sent.back());
  refresh();
  task->Run(1);
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).current[0] != 0);
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).current ==
         first_current);
  Timebase::now += 10000;
  refresh(true, 2);
  task->Run(1);
  assert_zero(can.sent.back());
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).faults & MOTOR_LOST);
  refresh();
  task->Run(1);
  GimbalFeedback stop{};
  stop.yaw_valid = true;
  stop.time_us = Timebase::now;
  stop.stop_chassis = true;
  Topic::CreateTopic<GimbalFeedback>("gimbal_chassis_feedback").Publish(stop);
  task->Run(1);
  assert_zero(can.sent.back());
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).faults & GIMBAL_STOP);
  // 遥控失联即整组无力；RAW和BODY也不能绕过yaw运行条件。
  CMD::Data lost{};
  cmd.FeedRC(CMD::RCInputSource::RC_INPUT_I6X, lost);
  task->Run(1);
  assert_zero(can.sent.back());
  refresh();
  Timebase::now += 21000;
  chassis.GetEvent().Active(uint32_t(Mode::RAW));
  task->Run(1);
  assert_zero(can.sent.back());
  // 显式RELAX停止并保持，不发送上次残留槽位；每周期恰好一帧完整四槽。
  refresh();
  chassis.GetEvent().Active(uint32_t(Mode::RELAX));
  auto count = can.sent.size();
  task->Run(1);
  assert(can.sent.size() == count + 1);
  assert_zero(can.sent.back());
  puts(
      "PASS: real Chassis/CMD/RMMotor integration, no constructor TX, missing/stale yaw, "
      "motor expiry, RC loss, RAW guard, four-slot zero output");
}
