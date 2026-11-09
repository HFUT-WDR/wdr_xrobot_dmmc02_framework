#include <cassert>
#include <cmath>
#include <iostream>
#include <string>
#include <vector>

#include "Launcher.hpp"
#include "reference_api.h"

using namespace LibXR;
static void near(float actual, float expected, float tolerance = 1e-4f)
{
  if (!(std::fabs(actual - expected) < tolerance))
  {
    std::cerr << "near failed: " << actual << " vs " << expected << '\n';
    assert(false);
  }
}
static CAN::ClassicPack feedback(uint32_t id, int raw_rpm, int raw_current = 0)
{
  CAN::ClassicPack p{};
  p.id = id;
  p.dlc = 8;
  p.data[2] = uint8_t(uint16_t(int16_t(raw_rpm)) >> 8);
  p.data[3] = uint8_t(uint16_t(int16_t(raw_rpm)) & 255);
  p.data[4] = uint8_t(uint16_t(int16_t(raw_current)) >> 8);
  p.data[5] = uint8_t(uint16_t(int16_t(raw_current)) & 255);
  return p;
}
static int16_t slot(const CAN::ClassicPack& p, unsigned index)
{
  return int16_t((uint16_t(p.data[index * 2]) << 8) | p.data[index * 2 + 1]);
}

int main(int argc, char** argv)
{
  assert(argc == 2);
  const std::string scenario = argv[1];
  Timebase::now = 1000000;
  CAN fric_bus, trigger_bus;
  // 原车0x205为右轮、出弹方向为负rpm，故右轮reverse；0x206左轮与拨弹0x207为正向。
  RMMotor fric_right(fric_bus, {RMMotor::Model::MOTOR_M3508, true, 0x205});
  RMMotor fric_left(fric_bus, {RMMotor::Model::MOTOR_M3508, false, 0x206});
  RMMotor trigger(trigger_bus, {RMMotor::Model::MOTOR_M2006, false, 0x207});
  Launcher::Param param{};
  Launcher launcher(fric_right, fric_left, trigger, param);
  assert(fric_bus.sent.empty() && trigger_bus.sent.empty());
  auto fire_topic = Topic::FindOrCreate<CMD::LauncherCMD>("launcher_cmd");
  auto referee_topic = Topic::FindOrCreate<RefereeTypes::RobotGameRefereePack>("robot_game_ref");
  const auto& state = launcher.GetState();
  using Mode = Launcher::ShootMode;
  // 逻辑速度→原始rpm：右轮reverse，原始值取反。
  float fric_speed = 0, trigger_speed = 0;
  int fric_current = 0;
  bool trigger_online = true;
  auto tick = [&](Launcher::Request request)
  {
    Timebase::now += 1000;
    const int rpm = int(std::lround(fric_speed / param.fric_rpm_to_speed));
    fric_bus.Emit(feedback(0x205, -rpm, -fric_current));
    fric_bus.Emit(feedback(0x206, rpm, fric_current));
    if (trigger_online)
      trigger_bus.Emit(
          feedback(0x207, int(std::lround(trigger_speed / param.trigger_rpm_to_speed))));
    launcher.Update(request, .001f, uint32_t(Timebase::now / 1000));
    launcher.Send();
  };
  const Launcher::Request op{.permit = true};

  if (scenario == "behavior")
  {
    tick(op);
    // 未请求摩擦轮：两轮零电流，拨弹零电流；每拍各发一帧完整组帧。
    assert(!state.fric_request && !state.running);
    assert(fric_bus.sent.back().id == 0x1FF && slot(fric_bus.sent.back(), 0) == 0 &&
           slot(fric_bus.sent.back(), 1) == 0);
    assert(trigger_bus.sent.back().id == 0x1FF && slot(trigger_bus.sent.back(), 2) == 0);
    // OP切换开启：起转期间拨弹不放行。
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    fire_topic.Publish(CMD::LauncherCMD{true});
    tick(op);
    assert(state.fric_request && state.running && !state.spun_up);
    assert(state.fric_current[0] > 0 && state.fric_current[1] > 0);
    // 右轮reverse：逻辑正电流对应负协议值。
    assert(slot(fric_bus.sent.back(), 0) < 0 && slot(fric_bus.sent.back(), 1) > 0);
    assert(!state.firing && state.trigger_current == 0);
    fric_speed = param.initial_speed;
    tick(op);
    assert(state.shoot_mode == Mode::SHOOT_READY && !state.firing);
    tick(op);
    assert(state.shoot_mode == Mode::SHOOT_BULLET && state.firing);
    near(state.trigger_target, param.fire_speed);
    assert(state.trigger_current > 0 && slot(trigger_bus.sent.back(), 2) > 0);
    // 射击时转速下跌不打断连发。
    fric_speed = param.initial_speed - 2;
    trigger_speed = param.fire_speed;
    tick(op);
    assert(state.spun_up && state.firing);
    // 停射：拨盘以零转速保持，摩擦轮继续。
    fire_topic.Publish(CMD::LauncherCMD{false});
    tick(op);
    assert(state.running && !state.firing);
    near(state.trigger_target, 0);
    // 云台停发射联锁：拨弹立停，摩擦轮速度环刹车，低于刹停阈值后撤电流。
    fire_topic.Publish(CMD::LauncherCMD{true});
    fric_speed = param.initial_speed;
    tick({.permit = false});
    assert(state.shoot_mode == Mode::SHOOT_STOP && !state.fric_request &&
           !state.running && state.trigger_current == 0);
    assert(state.fric_current[0] < 0 && state.fric_current[1] < 0);
    fric_speed = .05f;
    tick({.permit = false});
    assert(state.fric_current[0] == 0 && state.fric_current[1] == 0);
    // 停止期间拨动开关不留请求；许可恢复必须重新产生开启沿。
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    tick({.permit = false});
    tick(op);
    assert(state.shoot_mode == Mode::SHOOT_STOP && !state.running);
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    tick(op);
    assert(state.shoot_mode == Mode::SHOOT_READY_FRIC && !state.firing);
    fric_speed = param.initial_speed;
    tick(op);
    tick(op);
    tick(op);
    assert(state.firing);
    // AUTO：摩擦轮只跟随AIM，OP切换状态不起作用。
    tick({.permit = true, .automatic = true, .aim = false});
    assert(!state.fric_request && !state.running && state.trigger_current == 0);
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    tick({.permit = true, .automatic = true, .aim = true});
    assert(state.shoot_mode == Mode::SHOOT_READY_FRIC && state.running);
    // 拨弹掉线：拨弹无输出，摩擦轮继续。
    trigger_online = false;
    for (unsigned i = 0; i < 25; ++i) tick(op);
    assert(!state.trigger_online && !state.firing && state.trigger_current == 0);
    assert(state.running);
    trigger_online = true;
    tick(op);
    assert(state.firing);
    // 热量：裁判热量接近上限时停拨，摩擦轮保持。
    RefereeTypes::RobotGameRefereePack pack{};
    pack.robot_status.shooter_heat_limit = 200;
    pack.robot_status.shooter_cooling_value = 10;
    pack.launcher_17_heat = 150;
    for (unsigned i = 0; i < 2000; ++i)
    {
      referee_topic.Publish(pack);
      tick(op);
    }
    assert(state.referee_online && state.heat >= 120 && state.heat_blocked);
    assert(state.running && state.firing);
    near(state.trigger_target, 0);
    pack.launcher_17_heat = 0;
    for (unsigned i = 0; i < 2000; ++i)
    {
      referee_topic.Publish(pack);
      tick(op);
    }
    assert(!state.heat_blocked);
    near(state.trigger_target, param.fire_speed);
    // 再切换一次关闭摩擦轮。
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    tick(op);
    assert(!state.fric_request && !state.running && state.trigger_current == 0);
  }
  else if (scenario == "ready")
  {
    const Launcher::Request aim{.permit = true, .automatic = true, .aim = true};
    fric_speed = param.initial_speed;
    tick(aim);
    assert(state.shoot_mode == Mode::SHOOT_READY_FRIC);
    tick(aim);
    assert(state.shoot_mode == Mode::SHOOT_READY && state.spun_up && !state.firing);
    fire_topic.Publish(CMD::LauncherCMD{true});
    tick(aim);
    assert(state.shoot_mode == Mode::SHOOT_BULLET && state.firing);
    for (unsigned i = 0; i < 10; ++i) tick(aim);
    assert(state.shoot_mode == Mode::SHOOT_CONTINUE_BULLET && state.trigger_current > 0);
    // 连发转待发须清掉拨弹积分；零反馈、零目标时不能残留送弹电流。
    fire_topic.Publish(CMD::LauncherCMD{false});
    tick(aim);
    assert(state.shoot_mode == Mode::SHOOT_READY && !state.firing);
    assert(state.trigger_current == 0);
    // 新开火命令直接响应，不再经过无动作的准备拍。
    fire_topic.Publish(CMD::LauncherCMD{true});
    tick(aim);
    assert(state.shoot_mode == Mode::SHOOT_BULLET && state.firing);
  }
  else if (scenario == "events")
  {
    // 消费前同拍到达多次事件，保持操作次序；消费后不留第二套开关状态。
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    tick(op);
    assert(state.shoot_mode == Mode::SHOOT_STOP);
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_ON));
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    tick(op);
    assert(state.shoot_mode == Mode::SHOOT_STOP);
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_OFF));
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    tick(op);
    assert(state.shoot_mode == Mode::SHOOT_READY_FRIC);
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_OFF));
    tick(op);
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_ON));
    tick({.permit = false});
    tick(op);
    assert(state.shoot_mode == Mode::SHOOT_STOP && !state.running);
  }
  else if (scenario == "modes")
  {
    // 对照冻结原车模式函数；适配层合并无动作的READY_BULLET阶段。
    int previous_switch = 1;
    for (unsigned i = 0; i < 6000; ++i)
    {
      const bool automatic = i >= 3000;
      const unsigned phase = i % 300;
      const bool permit = phase < 230;
      const bool aim = phase < 200;
      const int position = (phase / 25) % 2 == 0 ? 1 : -1;
      if (position == -1 && previous_switch != -1)
        launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
      previous_switch = position;
      const bool fire = (phase / 17) % 2 == 0;
      fire_topic.Publish(CMD::LauncherCMD{fire});
      fric_speed = phase % 25 < 7 ? 0 : phase % 25 < 20 ? param.initial_speed
                                                                     : param.initial_speed + 1;
      tick({.permit = permit, .automatic = automatic, .aim = aim});
      const int expected = ref_mode_update(automatic, permit, aim, position, fire,
                                           state.fric_speed[0], state.fric_speed[1],
                                           state.speed_setpoint);
      assert(static_cast<int>(state.shoot_mode) == expected);
    }
  }
  else if (scenario == "heat")
  {
    // 与原车Shoot_Virtual_Heat_Update逐拍对照：电流脉冲判定、冷却离散化、裁判贴齐及默认值。
    // 参考与本模块从同一拍开始计时，冷却结算节拍才能对齐。
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_ON));
    fric_speed = param.initial_speed;
    const std::vector<unsigned> periods{40, 73, 130, 55, 97};
    const std::vector<unsigned> widths{5, 12, 25, 9, 18};
    unsigned phase = 0, cycle = 0;
    unsigned shots = 0;
    for (unsigned t = 0; t < 20000; ++t)
    {
      const unsigned period = periods[cycle % periods.size()];
      const bool pulse = phase < widths[cycle % widths.size()];
      fric_current = pulse ? 3686 : 410;  // 约4.5 A与0.5 A。
      if (++phase >= period)
      {
        phase = 0;
        ++cycle;
      }
      RefereeTypes::RobotGameRefereePack pack{};
      pack.robot_status.shooter_heat_limit = t < 8000 ? 200 : t < 12000 ? 0 : 320;
      pack.robot_status.shooter_cooling_value = t < 5000 ? 10 : t < 9000 ? 0 : 40;
      pack.launcher_17_heat = uint16_t((t / 50) % 180);
      referee_topic.Publish(pack);
      tick(op);
      const float amps = float(fric_current) * 20.0f / 16384.0f;
      ref_heat_set(pack.robot_status.shooter_cooling_value,
                   pack.robot_status.shooter_heat_limit, pack.launcher_17_heat,
                   uint32_t(Timebase::now / 1000));
      const float expected =
          ref_heat_update(amps, amps, state.fric_speed[0], state.fric_target);
      near(state.heat, expected, 1e-3f);
      shots = state.shots;
    }
    assert(shots > 50);
  }
  else if (scenario == "speed")
  {
    // 与原车shoot_speed_control逐拍对照：EMA、紧急下调、时间锁与死区。
    RefereeTypes::RobotGameRefereePack pack{};
    const std::vector<unsigned> gaps{37, 113, 260, 541, 1200, 2900};
    unsigned next = 50, shot = 0;
    for (unsigned t = 0; t < 60000; ++t)
    {
      if (t == next)
      {
        ++shot;
        pack.shot_seq = uint16_t(pack.shot_seq + 1);
        pack.launcher_data.bullet_speed =
            shot % 9 == 0 ? 4.0f
                          : 22.0f + 3.6f * std::sin(float(shot) * .7f) + float(shot) * 1e-4f;
        next += gaps[shot % gaps.size()];
      }
      referee_topic.Publish(pack);
      tick(op);
      const float expected =
          ref_speed_update(pack.launcher_data.bullet_speed, uint32_t(Timebase::now / 1000));
      near(state.speed_setpoint, expected, 1e-4f);
      near(state.speed_ema, ref_speed_ema(), 1e-4f);
    }
    assert(shot > 40);
  }
  else if (scenario == "jam")
  {
    // 与原车trigger_motor_turn_back逐拍对照：低速计时、反转与恢复。
    launcher.GetEvent().Active(uint32_t(Launcher::Event::FRIC_ON));
    fire_topic.Publish(CMD::LauncherCMD{true});
    fric_speed = param.initial_speed;
    const std::vector<std::pair<unsigned, float>> segments{
        {300, 20}, {1500, 0}, {200, 20}, {800, 0}, {2000, 20}, {1200, .5f}, {600, 3}, {1400, 0}};
    unsigned reversed = 0;
    for (auto [length, speed] : segments)
    {
      for (unsigned i = 0; i < length; ++i)
      {
        // 反转期间按反向转速反馈，模拟拨盘真正退弹。
        trigger_speed = state.reversing ? -speed : speed;
        tick(op);
        const float expected = -ref_jam(state.firing ? -param.fire_speed : 0, state.trigger_speed);
        near(state.trigger_target, expected);
        reversed += state.reversing;
      }
    }
    assert(reversed > 500);
  }
  else if (scenario == "filter")
  {
    // 与原车LowPassFilter2p逐拍对照最新输出；原车读取的Output[0]滞后两拍。
    std::vector<float> latest;
    for (unsigned t = 0; t < 3000; ++t)
    {
      trigger_speed = 15.0f * std::sin(float(t) * .013f) + float(int(t % 17) - 8);
      tick(op);
      const float input = float(std::lround(trigger_speed / param.trigger_rpm_to_speed)) *
                          param.trigger_rpm_to_speed;
      float expected, delayed;
      ref_filter(input, &expected, &delayed);
      near(state.trigger_speed, expected, 1e-3f);
      latest.push_back(expected);
      if (t >= 2) near(delayed, latest[t - 2], 1e-5f);
    }
  }
  else
    assert(false);
  std::cout << "PASS: launcher " << scenario << '\n';
}
