#include <cassert>
#include <iostream>

#include "Gimbal.hpp"
#include "HostData.hpp"
#include "thread_step.hpp"
#include "reference_api.h"
using namespace LibXR;
static void near(float a, float b, float tolerance = 1e-4f)
{
  assert(std::fabs(a - b) < tolerance);
}
static CAN::ClassicPack yaw_pack(uint32_t id, uint16_t ecd)
{
  CAN::ClassicPack p{};
  p.id = id;
  p.dlc = 8;
  p.data[0] = ecd >> 8;
  p.data[1] = ecd & 255;
  return p;
}
static CAN::ClassicPack pitch_pack(uint16_t pos = 32768)
{
  CAN::ClassicPack p{};
  p.id = 0x11;
  p.dlc = 8;
  p.data[0] = 0x11;
  p.data[1] = pos >> 8;
  p.data[2] = pos & 255;
  p.data[3] = 0x80;
  p.data[4] = 0x08;
  p.data[5] = 0x00;
  return p;
}
static int voltage(const CAN::ClassicPack& p, unsigned slot)
{
  return int16_t((uint16_t(p.data[slot * 2]) << 8) | p.data[slot * 2 + 1]);
}
int main()
{
  static_assert(sizeof(HostData::HostGimbalTarget) == 9 * sizeof(float));
  uint16_t offset;
  float maximum, minimum;
  DualYaw::CalibrateAxis(500, 7500, offset, maximum, minimum);
  assert(offset == 8096);
  assert(maximum > 0 && minimum < 0);
  near(maximum, 596 * 2 * wdr_control::GIMBAL_PI / 8192);

  // 同一原车冻结函数逐步对照，验证历史积分/差分/抗饱和而非仅比较公式。
  wdr_control::GimbalPidParam pid_param{3, .12f, .2f, .3f, 2, 10, .1f};
  wdr_control::GimbalPID migrated_angle(pid_param), migrated_velocity(pid_param);
  pid_type_def old_angle{}, old_velocity{};
  old_angle.Param = old_velocity.Param = {3, .12f, .2f, .3f, 2, 10, .1f};
  for (unsigned i = 0; i < 3000; ++i)
  {
    const float target = float(int(i % 97) - 48) * .01f;
    const float measured = float(int(i % 73) - 36) * .02f;
    near(migrated_angle.Angle(target - measured, .1f, .001f),
         PID_CalculateByError(&old_angle, target - measured, .1f));
    near(migrated_velocity.Velocity(target, measured, .001f),
         PID_CalculateVelocity(&old_velocity, measured, target));
  }
  for (unsigned i = 0; i < 8192; i += 17)
  {
    unsigned short original_offset;
    float original_max, original_min;
    const unsigned short max = (i + 1234) % 8192, min = i;
    DualYaw::CalibrateAxis(max, min, offset, maximum, minimum);
    calc_gimbal_axis_cali(max, min, &original_offset, &original_max, &original_min);
    assert(offset == original_offset);
    near(maximum, original_max);
    near(minimum, original_min);
  }
  Timebase::now = 1000000;
  CAN can;
  RMMotor inner(
      can, {RMMotor::Model::MOTOR_GM6020, false, 0x205, RMMotor::Gm6020Mode::VOLTAGE});
  RMMotor outer(
      can, {RMMotor::Model::MOTOR_GM6020, false, 0x206, RMMotor::Gm6020Mode::VOLTAGE});
  DMMotor dm(can, {DMMotor::Model::MOTOR_DM4310, false, 1});
  Database database;
  DualYaw::Param yp{};
  DualYaw yaw(inner, outer, yp, &database);
  Pitch::Param pp{};
  Pitch pitch(dm, pp);
  CMD cmd;
  Gimbal::Param gp{};
  gp.calibration_hold_ms = 2;
  gp.init_hold_ms = 3;
  Gimbal gimbal(yaw, pitch, cmd, gp);
  auto& task = *Thread::threads.back();
  assert(can.sent.empty());
  assert(Thread::threads.size() == 1);
  auto refresh = [&](uint16_t a = 0, uint16_t b = 0)
  {
    can.Emit(yaw_pack(0x205, a));
    can.Emit(yaw_pack(0x206, b));
    can.Emit(pitch_pack());
  };
  auto board = Topic::FindOrCreate<wdr_imu::BoardState>("board_attitude");
  auto attitude = Topic::FindOrCreate<wdr_imu::Attitude>("dm_attitude");
  board.Publish(wdr_imu::BoardState{});
  attitude.Publish(wdr_imu::Attitude{});
  CMD::Data rc{};
  rc.input_online = true;
  cmd.FeedRC(rc);
  refresh();
  run_thread_once(task);
  assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
  assert(can.sent.back().data[7] == 0xFD);
  gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::ABSOLUTE_ANGLE));
  Timebase::now += 1000;
  refresh();
  run_thread_once(task);
  assert(gimbal.GetState().mode == Gimbal::Mode::ABSOLUTE_ANGLE);
  near(dm.GetAngle(), 6.283185f / 65535, 2e-6f);  // 无符号位置解码覆盖半圈分界。
  CMD::Data ai{};
  ai.gimbal.yaw = 0.2f;
  ai.gimbal.pit = 0;
  cmd.FeedAI(CMD::Channel::GIMBAL, ai);  // 故意先捕获切源前的OP帧。
  cmd.GetEvent().Active(uint32_t(CMD::Mode::CMD_AUTO_CTRL));
  gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::AUTO));
  Timebase::now += 1000;
  refresh();
  run_thread_once(task);
  assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
  assert(gimbal.GetState().waiting_for_source);
  cmd.FeedAI(CMD::Channel::GIMBAL, ai);
  Timebase::now += 1000;
  refresh();
  run_thread_once(task);
  assert(gimbal.GetState().mode == Gimbal::Mode::AUTO);
  for (unsigned i = 0; i < 100; ++i)
  {
    Timebase::now += 1000;
    refresh();
    run_thread_once(task);
  }
  assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
  cmd.FeedAI(CMD::Channel::GIMBAL, ai);
  Timebase::now += 1000;
  refresh();
  run_thread_once(task);
  // 同26_sentry：AI失效只令本拍无力，有效指令恢复后按原请求自动继续。
  assert(gimbal.GetState().mode == Gimbal::Mode::AUTO);
  assert(!gimbal.GetState().ai_lost);
  inner.SetVoltage(30000);
  outer.SetVoltage(-30000);
  assert(can.sent.back().id == 0x1FF);
  assert(voltage(can.sent.back(), 0) == 30000);
  assert(voltage(can.sent.back(), 1) == -30000);
  rc.input_online = false;
  cmd.FeedRC(rc);
  Timebase::now += 1000;
  refresh();
  run_thread_once(task);
  assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
  rc.input_online = true;
  cmd.FeedRC(rc);
  cmd.FeedAI(CMD::Channel::GIMBAL, ai);
  Timebase::now += 1000;
  refresh();
  run_thread_once(task);
  // 遥控恢复后不需要重新拨杆，回到断连前请求的模式。
  assert(gimbal.GetState().mode == Gimbal::Mode::AUTO);

  // 完整双yaw端点扫描；结果更新零位/限位，不发送电机设零帧。
  gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::CALI));
  for (unsigned step = 1; step <= 4; ++step)
  {
    const uint16_t a = step == 3 ? 1000 : step == 4 ? 7000 : 0;
    const uint16_t b = step == 1 ? 600 : step == 2 ? 7600 : 0;
    for (unsigned n = 0; n < 3; ++n)
    {
      Timebase::now += 1000;
      refresh(a, b);
      run_thread_once(task);
    }
  }
  assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
  assert(gimbal.GetState().calibration_complete && gimbal.GetState().calibration_saved);
  assert(yaw.GetCalibration().inner_offset == 8096);
  assert(yaw.GetCalibration().outer_offset == 4);
  assert(database.values.contains("gimbal_yaw_calibration"));
  // 存储内容损坏（零位越界、NaN限位）时上电不断言，沿用YAML标定。
  {
    Database corrupt;
    corrupt.values["gimbal_yaw_calibration"] = DualYaw::Calibration{
        9000, 4, std::nanf(""), 1, -1, 1};
    DualYaw fallback(inner, outer, yp, &corrupt);
    assert(!fallback.CalibrationLoaded());
    assert(fallback.GetCalibration().inner_offset == yp.inner.offset_ecd);
    near(fallback.GetCalibration().inner_max, yp.inner.maximum);
    DualYaw loaded(inner, outer, yp, &database);
    assert(loaded.CalibrationLoaded());
    assert(loaded.GetCalibration().inner_offset == 8096);
  }
  // CALI请求保持期间不重复扫描端点。
  for (unsigned i = 0; i < 5; ++i)
  {
    Timebase::now += 1000;
    refresh(8096, 4);
    run_thread_once(task);
    assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
  }
  cmd.GetEvent().Active(uint32_t(CMD::Mode::CMD_OP_CTRL));
  cmd.FeedRC(rc);
  Timebase::now += 1000;
  refresh(8096, 4);
  run_thread_once(task);
  for (auto mode :
       {Gimbal::Mode::RELATIVE_ANGLE, Gimbal::Mode::MOTIONLESS, Gimbal::Mode::INIT})
  {
    gimbal.GetEvent().Active(uint32_t(mode));
    Timebase::now += 1000;
    refresh(8096, 4);
    run_thread_once(task);
    assert(gimbal.GetState().mode == mode);
  }
  for (unsigned i = 0; i < 4; ++i)
  {
    Timebase::now += 1000;
    refresh(8096, 4);
    run_thread_once(task);
  }
  assert(gimbal.GetState().mode == Gimbal::Mode::ABSOLUTE_ANGLE);
  gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::RELAX));
  Timebase::now += 1000;
  refresh(8096, 4);
  run_thread_once(task);
  cmd.GetEvent().Active(uint32_t(CMD::Mode::CMD_AUTO_CTRL));
  ai.gimbal.auto_mode = CMD::GimbalAutoMode::SCAN;
  cmd.FeedAI(CMD::Channel::GIMBAL, ai);
  gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::AUTO));
  Timebase::now += 1000;
  refresh(8096, 4);
  run_thread_once(task);
  assert(yaw.GetState().path == DualYaw::Path::SCAN);
  assert(pitch.GetState().path == Pitch::Path::SCAN);
  ai.gimbal.auto_mode = CMD::GimbalAutoMode::STOP;
  cmd.FeedAI(CMD::Channel::GIMBAL, ai);
  Timebase::now += 1000;
  refresh(8096, 4);
  run_thread_once(task);
  assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);

  // 自瞄STOP只令本拍无力；视觉切回AIM后无需重新发模式事件。
  ai.gimbal.auto_mode = CMD::GimbalAutoMode::AIM;
  ai.gimbal.yaw = ai.gimbal.pit = 0;
  cmd.FeedAI(CMD::Channel::GIMBAL, ai);
  Timebase::now += 1000;
  refresh(8096, 4);
  run_thread_once(task);
  assert(gimbal.GetState().mode == Gimbal::Mode::AUTO);
  assert(yaw.GetState().path == DualYaw::Path::SMALL_FIRST);
  assert(pitch.GetState().path == Pitch::Path::AIM);
  // 任一yaw真实反馈超时使整套云台停机，不依赖Update调用次数。
  Timebase::now += 11000;
  can.Emit(yaw_pack(0x205, 8096));
  can.Emit(pitch_pack());
  run_thread_once(task);
  assert(gimbal.GetState().motor_lost);
  assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
  cmd.FeedAI(CMD::Channel::GIMBAL, ai);
  Timebase::now += 1000;
  refresh(8096, 4);
  run_thread_once(task);
  assert(!gimbal.GetState().motor_lost);
  assert(gimbal.GetState().mode == Gimbal::Mode::AUTO);
  // SMALL_FIRST增益调度后切到相对角，小yaw速度环须恢复配置增益而非沿用调度值。
  {
    DualYaw::Param gain_param{};
    gain_param.align_alpha = 0;
    DualYaw gain_yaw(inner, outer, gain_param);
    wdr_imu::BoardState gain_board{};
    wdr_imu::Attitude gain_dm{};
    auto relative_voltage = [&](bool scheduled)
    {
      gain_yaw.SetPath(DualYaw::Path::RELAX, false);
      refresh(0, 0);
      gain_yaw.UpdateFeedback(gain_board, gain_dm, 0, uint32_t(Timebase::now / 1000));
      if (scheduled)
      {
        gain_yaw.SetPath(DualYaw::Path::SMALL_FIRST, true);
        gain_yaw.SetAbsolute(0, 1.0f);  // 小yaw大误差，增益调度t=1。
        gain_yaw.ControlLoop(.001f);
      }
      gain_yaw.SetPath(DualYaw::Path::RELATIVE, false);
      gain_yaw.SetRelative(0, .05f);
      gain_yaw.ControlLoop(.001f);
      return gain_yaw.GetState().axes[DualYaw::OUTER_YAW].voltage;
    };
    const auto clean = relative_voltage(false);
    assert(relative_voltage(true) == clean);
  }
  DualYaw::Param planning_param{};
  planning_param.align_alpha = 0;
  planning_param.inner.angle = planning_param.inner.auto_angle = {1, 0, 0, 0, 0, 100, 0};
  planning_param.outer.angle = planning_param.outer.auto_angle = {1, 0, 0, 0, 0, 100, 0};
  planning_param.outer_min_kp = 1;
  DualYaw planning(inner, outer, planning_param);
  wdr_imu::BoardState planning_board{};
  wdr_imu::Attitude planning_dm{};
  refresh();
  planning.UpdateFeedback(planning_board, planning_dm, 0, uint32_t(Timebase::now / 1000));
  planning.SetPath(DualYaw::Path::SMALL_FIRST, true);
  float drag = planning.GetState().drag_target;
  for (unsigned i = 0; i < 1000; ++i)
  {
    planning_board.yaw = float(int(i % 61) - 30) * .1f;
    planning_dm.yaw = float(int(i % 59) - 29) * .1f;
    refresh(0, (i * 97) % 8192);
    planning.UpdateFeedback(planning_board, planning_dm, 0,
                            uint32_t(Timebase::now / 1000));
    const float target = float(int(i % 57) - 28) * .1f;
    planning.SetAbsolute(target, target);
    float big_error, small_error;
    reference_plan(planning.GetState().axes[1].relative, planning_dm.yaw,
                   planning_board.yaw, target, planning_param.outer.minimum,
                   planning_param.outer.maximum, &drag, &big_error, &small_error);
    planning.ControlLoop(.001f);
    near(planning.GetState().drag_target, drag);
    near(planning.GetState().axes[0].velocity_target, wdr_control::wrap_angle(big_error));
    near(planning.GetState().axes[1].velocity_target,
         wdr_control::wrap_angle(small_error));
  }

  // 此处仅对照未触及机械限位时的原车力矩；边界场景另验证限位后的行为。
  auto reference_param = pp;
  reference_param.minimum = -2;
  reference_param.maximum = 2;
  Pitch tracking(dm, reference_param);
  tracking.SetPath(Pitch::Path::AIM);
  float old_pitch_integral = 0;
  for (unsigned i = 0; i < 1000; ++i)
  {
    wdr_imu::Attitude feedback{};
    feedback.pitch = float(int(i % 61) - 30) * .01f;
    feedback.gyro[1] = float(int(i % 43) - 21) * .01f;
    const float target = float(int(i % 101) - 50) * .01f;
    const float velocity = float(int(i % 31) - 15) * .02f;
    const float acceleration = float(int(i % 29) - 14) * .1f;
    tracking.UpdateFeedback(feedback, uint32_t(Timebase::now / 1000));
    tracking.SetAbsolute(target, velocity, acceleration);
    tracking.ControlLoop(.001f);
    near(tracking.GetState().torque,
         reference_pitch(feedback.pitch, feedback.gyro[1], target, velocity, acceleration,
                         &old_pitch_integral));
  }
  CAN current_bus;
  RMMotor current_motor(current_bus, {RMMotor::Model::MOTOR_GM6020, false, 0x205});
  current_motor.Control({.mode = Motor::MODE_CURRENT, .velocity = 1});
  assert(current_bus.sent.back().id == 0x1FE);
  assert(voltage(current_bus.sent.back(), 0) == 16384);
  CAN range_bus;
  DMMotor ranged_dm(range_bus, {DMMotor::Model::MOTOR_DM4310, false, 3, 12.5f});
  auto range_pack = pitch_pack(65535);
  range_pack.id = 0x13;
  range_pack.data[0] = 0x13;
  range_bus.Emit(range_pack);
  ranged_dm.Update();
  near(ranged_dm.GetAngle(), 12.5f);
  range_pack.dlc = 7;
  Timebase::now += 21000;
  range_bus.Emit(range_pack);
  ranged_dm.Update();
  assert(!ranged_dm.FeedbackFresh(uint32_t(Timebase::now / 1000), 20));
  // 两路yaw零输出与DM失能分别验证，不能将MIT零力矩等同RELAX。
  bool zero_yaw = false, disabled = false;
  for (const auto& p : can.sent)
  {
    if (p.id == 0x1FF && voltage(p, 0) == 0 && voltage(p, 1) == 0) zero_yaw = true;
    if (p.id == 1 && p.data[7] == 0xFD) disabled = true;
  }
  assert(zero_yaw && disabled);
  std::cout << "PASS: no constructor TX, single thread, voltage mode, DM unsigned "
               "decode, AI expiry/recovery, RC loss/auto-resume, STOP->AIM resume, motor-loss resume, "
               "SMALL_FIRST gain restore, complete "
               "calibration/persistence, modes; frozen PID/zero calibration 1000 "
               "dual-yaw plans and 1000 frozen pitch torque samples\n";
}
