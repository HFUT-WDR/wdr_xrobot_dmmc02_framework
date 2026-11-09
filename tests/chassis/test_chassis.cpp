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
  // 轮速正逆解保持一致，覆盖单轴和组合运动，反馈不反除平移补偿。
  for (const auto& motion : {std::array<float, 3>{1, 0, 0}, {0, 1, 0},
                             {0, 0, 1}, {.4f, -.3f, .8f}})
  {
    float wheel[4];
    reference_wheel(motion[0], motion[1], motion[2], wheel);
    Feedback feedback{};
    for (unsigned i = 0; i < 4; ++i)
    {
      feedback.rpm[i] = wheel[i] / p.rpm_to_speed;
    }
    Control inverse(p);
    inverse.UpdateControllerState(feedback);
    near(inverse.GetState().vx, motion[0]);
    near(inverse.GetState().vy, motion[1]);
    near(inverse.GetState().wz, motion[2]);
  }
  // 实际dt同时影响滤波、轮缘加速度和PID，标称参数换算保持不变。
  Control timed(p);
  Feedback timed_feedback{};
  timed_feedback.rpm.fill(100);
  timed.SetDt(.004f);
  timed.UpdateControllerState(timed_feedback);
  near(timed.GetState().accel[0], 100 * p.rpm_to_speed / .004f);
  timed.SetMode(Mode::NO_FOLLOW_YAW);
  timed.ModeTransition();
  timed.SetControl({1, 0, 0, Frame::BODY});
  near(timed.GetState().vx_set, .004f / (p.filter_tau + .004f));
  Control ctrl(p);
  Feedback f{};
  f.imu_yaw = .7f;
  f.yaw_relative = .2f;
  ctrl.UpdateControllerState(f);
  near(ctrl.GetState().chassis_yaw, .5f);
  ctrl.SetMode(Mode::NO_FOLLOW_YAW);
  ctrl.ModeTransition();
  near(ctrl.GetState().chassis_yaw_set, .5f);
  f.imu_yaw = 1;
  ctrl.UpdateControllerState(f);
  ctrl.ModeTransition();
  near(ctrl.GetState().chassis_yaw_set, .5f);
  // 原车运动学冻结片段对照；使用目标阶段已衰减的旋转给定。
  for (unsigned i = 0; i < 1000; ++i)
  {
    ctrl.UpdateControllerState(f);
    ctrl.SetMode(Mode::RAW);
    ctrl.ModeTransition();
    Command cmd{float(int(i % 17) - 8) * 100, float(int(i % 23) - 11) * 100,
                float(int(i % 29) - 14) * 50, Frame::BODY};
    ctrl.SetControl(cmd);
    ctrl.ControlLoop();
    float expected[4];
    reference_wheel(cmd.x * p.translation_comp, cmd.y * p.translation_comp, ctrl.GetState().wz_set,
                    expected);
    for (unsigned j = 0; j < 4; ++j)
    {
      assert(ctrl.GetState().current[j] == int16_t(Limit(expected[j], -16000, 16000)));
    }
  }
  ctrl.Reset();
  ctrl.UpdateControllerState(f);
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
  braking.UpdateControllerState(rolling);
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
  ctrl.UpdateControllerState(f);
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
  ctrl.UpdateControllerState(f);
  ctrl.SetMode(Mode::NO_FOLLOW_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({1, 0, 0, Frame::GIMBAL});
  near(ctrl.GetState().vx_set, 0);
  near(ctrl.GetState().vy_set, .002f / (p.filter_tau + .002f));
  ctrl.Reset();
  ctrl.UpdateControllerState(f);
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
  ctrl.UpdateControllerState(f);
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
  ctrl.UpdateControllerState(f);
  assert(ctrl.GetState().mode == Mode::RELAX);
  f.imu_yaw = 0;
  f.rpm[1] = std::numeric_limits<float>::quiet_NaN();
  ctrl.SetMode(Mode::NO_FOLLOW_YAW);
  ctrl.ModeTransition();
  ctrl.UpdateControllerState(f);
  assert(ctrl.GetState().mode == Mode::RELAX);
  // 公共PID使用微分先行：首拍D为零，后续保留低通；故障恢复不带入旧导数。
  f.rpm = {};
  f.imu_yaw = .1f;
  f.yaw_relative = .01f;
  ctrl.Reset();
  ctrl.UpdateControllerState(f);
  ctrl.SetMode(Mode::FOLLOW_GIMBAL_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({0, 0, 0});
  near(ctrl.GetState().wz_set, .12f);
  f.yaw_relative = .03f;
  ctrl.UpdateControllerState(f);
  ctrl.SetControl({0, 0, 0});
  // PID原始目标0.40；保留斜坡每拍最多增加120×0.002=0.24，最终0.36。
  near(ctrl.GetState().wz_set, .36f);
  ctrl.Reset();
  ctrl.UpdateControllerState(f);
  ctrl.SetMode(Mode::FOLLOW_GIMBAL_YAW);
  ctrl.ModeTransition();
  ctrl.SetControl({0, 0, 0});
  near(ctrl.GetState().wz_set, .36f);
  // 实际底盘跟随控制跨越正负π时走短角差，不因表示跳变输出反向饱和。
  Control cyclic_ctrl(p);
  Feedback cyclic_feedback{};
  cyclic_feedback.imu_yaw = 3.13f;
  cyclic_ctrl.UpdateControllerState(cyclic_feedback);
  cyclic_ctrl.SetMode(Mode::FOLLOW_CHASSIS_YAW);
  cyclic_ctrl.ModeTransition();
  cyclic_ctrl.SetControl({});
  cyclic_feedback.imu_yaw = -3.13f;
  cyclic_ctrl.UpdateControllerState(cyclic_feedback);
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
static void auto_spin_tests()
{
  AutoSpinParam param;
  AutoSpin spin(param);
  auto calculate = [&](float vx, bool aim, float distance, uint8_t progress,
                        float buffer, uint32_t time)
  { return spin.Calculate(vx, 0, aim, distance, progress, buffer, time); };
  near(calculate(0, false, 0, 4, 60, 0), -2);
  near(calculate(0, true, 5, 4, 60, 0), 0);
  near(calculate(0, true, 4, 4, 60, 0), -3.5f);
  near(calculate(.3f, true, 5, 4, 60, 0), -2);
  spin.UpdateReferee(400, 0, 4, 10);
  spin.UpdateReferee(390, 0, 4, 100);
  near(calculate(0, true, 5, 4, 60, 100), -7);
  near(calculate(0, false, 0, 4, 60, 6100), -7);
  near(calculate(0, false, 0, 4, 60, 6101), -2);
  near(calculate(0, false, 0, 1, 60, 100), -1);
  near(calculate(0, false, 0, 3, 60, 100), -1);
  near(calculate(0, false, 0, 2, 60, 100), 0);
  near(calculate(0, false, 0, 4, 29, 100), 0);
  near(calculate(0, false, 0, 4, 30, 100), -7);
  AutoSpin other(param);
  other.UpdateReferee(400, 0, 4, 0);
  other.UpdateReferee(390, 2, 4, 100);
  near(other.Calculate(0, 0, false, 0, 4, 60, 100), -2);
  other.UpdateReferee(380, 2, 0, 200);
  near(other.Calculate(0, 0, false, 0, 0, 60, 200), -7);
  // 缩减在滤波、坐标转换之后执行一次；普通移动姿态跳过，强化移动不跳过。
  Param control_param;
  Control control(control_param);
  control.SetMode(Mode::NO_FOLLOW_YAW);
  control.ModeTransition();
  Command command{2, 0, -7, Frame::BODY};
  for (unsigned i = 0; i < 2000; ++i) control.SetControl(command);
  near(control.GetState().wz_set, -2.1f, 1e-4f);
  command.moving_posture = true;
  control.SetControl(command);
  near(control.GetState().wz_set, -7);
  // 冻结原车0x020D前6字节的真实布局，覆盖强化移动姿态标志。
  const uint8_t wire[]{0, 0, 0, 0, 0, 0xB0};
  RefereeTypes::SentryInfo info{};
  static_assert(sizeof(info) == 6);
  std::memcpy(&info, wire, sizeof(info));
  assert(info.current_state == 3 && info.enhanced);
  std::puts("PASS: automatic spin branches, priority, attack hold, posture scaling and referee bit layout");
}

int main()
{
  auto_spin_tests();
  // 每项故障可单独触发总判据；发送前合并不能清掉本拍已有故障。
  bool Faults::* fields[]{&Faults::input_lost, &Faults::motor_lost,
                          &Faults::invalid_target, &Faults::invalid_feedback};
  Faults combined{};
  assert(!combined.Any());
  for (auto field : fields)
  {
    Faults single{};
    single.*field = true;
    assert(single.Any());
    combined.Merge(single);
    assert(combined.*field);
  }
  combined.Merge(Faults{});
  for (auto field : fields)
  {
    assert(combined.*field);
  }
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
  ::PowerControl power;
  Chassis<Omni> chassis(lr, rf, rr, lf, cmd, power);
  assert(can.sent.empty());  // 构造不发送。
  auto* task = Thread::threads.back();
  chassis.GetEvent().Active(uint32_t(Mode::NO_FOLLOW_YAW));
  task->Run(1);
  assert(can.sent.size() == 1);
  assert_zero(can.sent.back());
  auto status = std::any_cast<State>(Topic::values.at("chassis_status"));
  assert(status.mode == Mode::RELAX && status.faults.input_lost);
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
    (void)yaw;
    Topic::CreateTopic<GimbalFeedback>("gimbal_chassis_feedback").Publish(g);
    CMD::Data data{};
    data.input_online = true;
    data.chassis.x = .7;
    data.chassis.z = .1;
    data.chassis.frame = CMD::ChassisFrame::BODY;
    cmd.FeedRC(CMD::RCInputSource::RC_INPUT_I6X, data);
  };
  refresh();
  chassis.GetEvent().Active(uint32_t(Mode::NO_FOLLOW_YAW));
  task->Run(1);
  assert(can.sent.size() == 2);
  status = std::any_cast<State>(Topic::values.at("chassis_status"));
  assert(status.mode == Mode::NO_FOLLOW_YAW && !status.faults.Any());
  assert(status.current[0] != 0);
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
  // 角度消息停更沿用旧值；停止由模式事件表达。
  chassis.GetEvent().Active(uint32_t(Mode::RELAX));
  task->Run(1);
  assert_zero(can.sent.back());
  refresh();
  chassis.GetEvent().Active(uint32_t(Mode::NO_FOLLOW_YAW));
  task->Run(1);
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).current[0] != 0);
  // 算法拒绝非法角度后，模式选择不能覆盖本拍故障重新使能输出。
  wdr_imu::BoardState invalid_board{};
  invalid_board.yaw = std::numeric_limits<float>::quiet_NaN();
  Topic::CreateTopic<wdr_imu::BoardState>("board_attitude").Publish(invalid_board);
  task->Run(1);
  assert_zero(can.sent.back());
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).faults.invalid_feedback);
  refresh();
  task->Run(1);
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).mode == Mode::NO_FOLLOW_YAW);
  Timebase::now += 10000;
  refresh(true, 2);
  task->Run(1);
  assert_zero(can.sent.back());
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).faults.motor_lost);
  refresh();
  task->Run(1);
  // 遥控失联即整组无力；RAW和BODY也不能绕过yaw运行条件。
  CMD::Data lost{};
  cmd.FeedRC(CMD::RCInputSource::RC_INPUT_I6X, lost);
  task->Run(1);
  assert_zero(can.sent.back());
  refresh();
  task->Run(1);
  // 同26_sentry：遥控恢复后按原模式请求自动继续，不需要重新拨杆。
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).mode == Mode::NO_FOLLOW_YAW);
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).current[0] != 0);
  // 云台停底盘为电平许可：置位期间无力，撤销后自动继续。
  {
    GimbalFeedback stop{};
    stop.yaw_relative_rad = .1;
    stop.stop_chassis = true;
    for (unsigned i = 0; i < 4; ++i) feed_can(can, 0x201 + i);
    Topic::CreateTopic<GimbalFeedback>("gimbal_chassis_feedback").Publish(stop);
    task->Run(1);
    assert_zero(can.sent.back());
    assert(std::any_cast<State>(Topic::values.at("chassis_status")).faults.gimbal_stop);
    refresh();
    task->Run(1);
    assert(std::any_cast<State>(Topic::values.at("chassis_status")).mode == Mode::NO_FOLLOW_YAW);
  }
  for (unsigned i = 0; i < 4; ++i) feed_can(can, 0x201 + i);
  chassis.GetEvent().Active(uint32_t(Mode::RAW));
  task->Run(1);
  assert(std::any_cast<State>(Topic::values.at("chassis_status")).mode == Mode::RAW);
  // 显式RELAX停止并保持，不发送上次残留槽位；每周期恰好一帧完整四槽。
  refresh();
  chassis.GetEvent().Active(uint32_t(Mode::RELAX));
  auto count = can.sent.size();
  task->Run(1);
  assert(can.sent.size() == count + 1);
  assert_zero(can.sent.back());
  // 实际任务接入功率控制：RAW也不能绕过预算，电容帧与四轮帧分别发送。
  CAN powered_bus;
  RMMotor prf(powered_bus, {RMMotor::Model::MOTOR_M3508, false, 0x201});
  RMMotor plf(powered_bus, {RMMotor::Model::MOTOR_M3508, false, 0x202});
  RMMotor plr(powered_bus, {RMMotor::Model::MOTOR_M3508, false, 0x203});
  RMMotor prr(powered_bus, {RMMotor::Model::MOTOR_M3508, false, 0x204});
  CMD power_cmd(CMD::Mode::CMD_OP_CTRL, "power_cmd", "power_gimbal", "power_fire");
  Param power_param;
  PowerParam power_settings;
  power_settings.enabled = true;
  ::PowerControl power_module(power_settings, &powered_bus);
  Chassis<Omni> powered(prf, plf, plr, prr, power_cmd, power_module, power_param, "power_cmd",
                  "power_attitude", "power_yaw", "power_status", 4096,
                  Thread::Priority::MEDIUM);
  auto* power_task = Thread::threads.back();
  powered.GetEvent().Active(uint32_t(Mode::RAW));
  for (unsigned i = 0; i < 4; ++i)
  {
    feed_can(powered_bus, 0x201 + i);
  }
  wdr_imu::BoardState power_attitude{};
  power_attitude.time_us = Timebase::now;
  Topic::CreateTopic<wdr_imu::BoardState>("power_attitude").Publish(power_attitude);
  GimbalFeedback power_yaw{};
  power_yaw.time_us = Timebase::now;
  Topic::CreateTopic<GimbalFeedback>("power_yaw").Publish(power_yaw);
  CMD::Data power_data{};
  power_data.input_online = true;
  power_data.chassis.x = 30000;
  power_data.chassis.frame = CMD::ChassisFrame::BODY;
  power_cmd.FeedRC(CMD::RCInputSource::RC_INPUT_I6X, power_data);
  powered.GetEvent().Active(uint32_t(Mode::RAW));
  power_task->Run(1);
  auto powered_state = std::any_cast<State>(Topic::values.at("power_status"));
  assert(powered_state.power.enabled && powered_state.power.limited);
  assert(powered_state.power.limited_positive_w <= 85.02f);
  assert(powered_bus.sent.size() == 2 && powered_bus.sent.front().id == 0x330);
  powered.GetEvent().Active(uint32_t(Mode::RELAX));
  power_task->Run(1);
  assert_zero(powered_bus.sent.back());
  // 反向电机的原始电流与逻辑轮速同向，INT16_MIN取反不能溢出。
  CAN reversed_bus;
  RMMotor reversed(reversed_bus, {RMMotor::Model::MOTOR_M3508, true, 0x201});
  CAN::ClassicPack reversed_frame{};
  reversed_frame.id = 0x201;
  reversed_frame.dlc = 8;
  reversed_frame.data[4] = 0x80;
  reversed_bus.Emit(reversed_frame);
  reversed.Update();
  assert(reversed.GetFeedbackCurrent() == 32768);
  // 实际模块订阅链路：本地自动旋转消费目标距离、裁判血量和功率缓冲。
  chassis.GetEvent().Active(uint32_t(Mode::NO_FOLLOW_YAW));
  RefereeTypes::RobotGameRefereePack game{};
  game.game_status.game_progress = 4;
  game.robot_status.remain_hp = 400;
  game.sentry_info.current_state = 3;
  Topic::FindOrCreate<RefereeTypes::RobotGameRefereePack>("robot_game_ref").Publish(game);
  RefereeTypes::ChassisPower energy{};
  energy.buffer_j = 60;
  energy.buffer_seen = true;
  Topic::FindOrCreate<RefereeTypes::ChassisPower>("chassis_power_ref").Publish(energy);
  CMD::ChassisCMD local{};
  local.local_spin = local.aiming = true;
  local.target_distance = 6;
  auto spin_step = [&]()
  {
    for (unsigned i = 0; i < 4; ++i) feed_can(can, 0x201 + i);
    Topic::FindOrCreate<CMD::ChassisCMD>("chassis_cmd").Publish(local);
    task->Run(1);
    return std::any_cast<State>(Topic::values.at("chassis_status")).wz_set;
  };
  near(spin_step(), 0);
  local.target_distance = 2;
  near(spin_step(), -3.5f);
  game.robot_status.remain_hp = 390;
  Topic::FindOrCreate<RefereeTypes::RobotGameRefereePack>("robot_game_ref").Publish(game);
  near(spin_step(), -7);
  energy.buffer_j = 29;
  Topic::FindOrCreate<RefereeTypes::ChassisPower>("chassis_power_ref").Publish(energy);
  near(spin_step(), 0);
  std::puts("PASS: module auto-spin distance, referee damage and buffer wiring");
  puts(
      "PASS: real Chassis/CMD/RMMotor integration, no constructor TX, held angle snapshots, "
      "motor expiry, RC loss/auto-resume, gimbal stop gate, RAW guard, four-slot zero output");
}
