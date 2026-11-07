#include <cassert>
#include <iostream>
#include <optional>
#include <string>

#include "Gimbal.hpp"
#include "HostData.hpp"
#include "thread_step.hpp"

using namespace LibXR;
static void near(float actual, float expected, float tolerance = 1e-4f)
{
  assert(std::fabs(actual - expected) < tolerance);
}
static CAN::ClassicPack yaw_pack(uint32_t id)
{
  CAN::ClassicPack pack{};
  pack.id = id;
  pack.dlc = 8;
  return pack;
}
static CAN::ClassicPack pitch_pack(float angle, bool enabled = true)
{
  CAN::ClassicPack pack{};
  pack.id = 0x11;
  pack.dlc = 8;
  pack.data[0] = enabled ? 0x11 : 0x01;
  const auto raw = uint16_t((angle + 6.283185f) / (2 * 6.283185f) * 65535);
  pack.data[1] = raw >> 8;
  pack.data[2] = raw & 255;
  pack.data[3] = 0x80;
  pack.data[4] = 8;
  return pack;
}
int main(int argc, char** argv)
{
  assert(argc == 2);
  const std::string scenario = argv[1];
  Timebase::now = 1000000;
  CAN can;
  RMMotor inner(
      can, {RMMotor::Model::MOTOR_GM6020, false, 0x205, RMMotor::Gm6020Mode::VOLTAGE});
  RMMotor outer(
      can, {RMMotor::Model::MOTOR_GM6020, false, 0x206, RMMotor::Gm6020Mode::VOLTAGE});
  DMMotor motor(can, {DMMotor::Model::MOTOR_DM4310, false, 1});
  DualYaw::Param yaw_param;
  DualYaw yaw(inner, outer, yaw_param);
  Pitch::Param pitch_param;
  pitch_param.zero = 1;
  Pitch pitch(motor, pitch_param);
  CMD cmd;
  Gimbal::Param param;
  if (scenario == "late_stop") param.init_hold_ms = 0;
  // 拨弹M2006 0x207与两台yaw同属0x1FF组；只在发射场景注册，避免其他场景组帧缺槽。
  CAN shoot_bus;
  std::optional<RMMotor> fric_right, fric_left, trigger;
  std::optional<Launcher> launcher;
  if (scenario == "launcher")
  {
    fric_right.emplace(shoot_bus, RMMotor::Param{RMMotor::Model::MOTOR_M3508, true, 0x205});
    fric_left.emplace(shoot_bus, RMMotor::Param{RMMotor::Model::MOTOR_M3508, false, 0x206});
    trigger.emplace(can, RMMotor::Param{RMMotor::Model::MOTOR_M2006, false, 0x207});
    launcher.emplace(*fric_right, *fric_left, *trigger, Launcher::Param{});
  }
  Gimbal gimbal(yaw, pitch, cmd, param, "gimbal_cmd", "board_attitude", "dm_attitude",
                "gimbal_context", "robot_game_ref", "gimbal_chassis_feedback",
                "gimbal_status", 4096, Thread::Priority::MEDIUM,
                launcher ? &*launcher : nullptr);
  auto& task = *Thread::threads.back();
  HostData host(cmd);
  CMD::Data rc{};
  rc.input_online = true;
  wdr_imu::BoardState board{};
  wdr_imu::Attitude attitude{};
  auto board_topic = Topic::FindOrCreate<wdr_imu::BoardState>("board_attitude");
  auto dm_topic = Topic::FindOrCreate<wdr_imu::Attitude>("dm_attitude");
  bool pitch_enabled = true;
  auto fresh = [&](bool with_pitch = true)
  {
    cmd.FeedRC(rc);
    can.Emit(yaw_pack(0x205));
    can.Emit(yaw_pack(0x206));
    if (with_pitch) can.Emit(pitch_pack(pitch_param.zero, pitch_enabled));
    board_topic.Publish(board);
    dm_topic.Publish(attitude);
  };
  auto tick = [&](bool with_pitch = true)
  {
    Timebase::now += 1000;
    fresh(with_pitch);
    run_thread_once(task);
  };
  fresh();
  run_thread_once(task);
  if (scenario == "late_stop")
  {
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::INIT));
    bool fired = false;
    Timebase::now += 1000;
    fresh();
    Timebase::time_hook = [&]
    {
      fired = true;
      gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::RELAX));
    };
    run_thread_once(task);
    assert(fired);
    // 拍内到达的RELAX在下一拍生效；INIT完成不得改写外部请求而覆盖RELAX。
    tick();
    assert(gimbal.GetState().requested == Gimbal::Mode::RELAX);
    assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
    assert(can.sent.back().data[7] == 0xFD);
    tick();
    assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
  }
  else if (scenario == "mode_loss")
  {
    // 同26_sentry：pitch掉线只重发使能，yaw照常闭环，模式请求保持。
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::ABSOLUTE_ANGLE));
    tick();
    for (unsigned i = 0; i < 19; ++i) tick(false);
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::RELATIVE_ANGLE));
    const auto first = can.sent.size();
    tick(false);
    assert(!gimbal.GetState().motor_lost);
    assert(!pitch.GetState().enabled);
    assert(gimbal.GetState().mode == Gimbal::Mode::RELATIVE_ANGLE);
    bool enable_sent = false;
    for (auto i = first; i < can.sent.size(); ++i)
      if (can.sent[i].id == 1)
      {
        // 失能期间只允许使能帧，不下发MIT控制。
        assert(can.sent[i].data[7] == 0xFC);
        enable_sent = true;
      }
    assert(enable_sent);
    tick();
    assert(pitch.GetState().enabled);
    assert(gimbal.GetState().mode == Gimbal::Mode::RELATIVE_ANGLE);
  }
  else if (scenario == "revive")
  {
    // 阵亡断电：全部电机反馈消失后整云台无力；复活上电后无需拨杆即恢复原模式。
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::ABSOLUTE_ANGLE));
    tick();
    assert(gimbal.GetState().mode == Gimbal::Mode::ABSOLUTE_ANGLE);
    for (unsigned i = 0; i < 2000; ++i)
    {
      Timebase::now += 1000;
      cmd.FeedRC(rc);
      board_topic.Publish(board);
      dm_topic.Publish(attitude);
      run_thread_once(task);
    }
    assert(gimbal.GetState().motor_lost);
    assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
    pitch_enabled = false;  // 复活上电后DM电机处于失能态，需云台重新使能。
    tick();
    assert(gimbal.GetState().mode == Gimbal::Mode::ABSOLUTE_ANGLE);
    pitch_enabled = true;
    tick();
    assert(pitch.GetState().enabled);
    assert(pitch.GetState().path == Pitch::Path::REMOTE_CONTROL);
  }
  else if (scenario == "init")
  {
    // 底座倾斜且编码器零点非零，归中仍保持机械零点。
    attitude.pitch = .2f;
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::INIT));
    for (unsigned i = 0; i < 500; ++i) tick();
    assert(gimbal.GetState().mode == Gimbal::Mode::INIT);
    near(pitch.GetState().output.position, pitch_param.zero);
    near(pitch.GetState().output.velocity, .5f);
    near(pitch.GetState().output.kp, 15);
    near(pitch.GetState().output.kd, 1.4f);
    near(pitch.GetState().output.torque, 0);
  }
  else if (scenario == "startup")
  {
    pitch_enabled = false;
    tick();
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::ABSOLUTE_ANGLE));
    const auto first = can.sent.size();
    tick();
    assert(gimbal.GetState().mode == Gimbal::Mode::ABSOLUTE_ANGLE);
    assert(!pitch.GetState().enabled);
    bool enable_sent = false;
    for (auto i = first; i < can.sent.size(); ++i)
      enable_sent |= can.sent[i].id == 1 && can.sent[i].data[7] == 0xFC;
    assert(enable_sent);
    pitch_enabled = true;
    tick();
    assert(gimbal.GetState().mode == Gimbal::Mode::ABSOLUTE_ANGLE);
    assert(pitch.GetState().path == Pitch::Path::REMOTE_CONTROL);
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::RELATIVE_ANGLE));
    tick();
    assert(gimbal.GetState().mode == Gimbal::Mode::RELATIVE_ANGLE);
  }
  else if (scenario == "scan_zero")
  {
    attitude.pitch = .2f;
    fresh();
    run_thread_once(task);
    pitch.SetPath(Pitch::Path::REMOTE_CONTROL);
    pitch.SetRelative(0);
    pitch.ControlLoop(.001f);
    const float before = pitch.GetState().output.position;
    pitch.SetPath(Pitch::Path::SCAN);
    pitch.ControlLoop(.001f);
    assert(std::fabs(pitch.GetState().output.position - before) < .003f);
    near(pitch.GetState().output.position - pitch_param.zero,
         pitch.GetState().relative + pitch.GetState().target - attitude.pitch);
  }
  else if (scenario == "scan_limits" || scenario == "aim_limits")
  {
    for (float sign : {-1.0f, 1.0f})
    {
      // 机械角居中，倾斜底座使惯性目标对应到机械范围之外。
      attitude.pitch = -sign;
      fresh();
      run_thread_once(task);
      pitch.SetPath(Pitch::Path::RELAX);
      pitch.SetPath(scenario == "scan_limits" ? Pitch::Path::SCAN : Pitch::Path::AIM);
      pitch.SetAbsolute(sign * .5f, sign * 2, sign * 3);
      pitch.ControlLoop(.001f);
      const auto& state = pitch.GetState();
      near(state.relative_target, sign < 0 ? pitch_param.minimum : pitch_param.maximum);
      near(state.target, attitude.pitch + state.relative_target - state.relative);
      near(state.output.velocity, 0);
      if (scenario == "scan_limits")
      {
        near(state.output.position, pitch_param.zero + state.relative_target);
        near(state.output.torque, -pitch_param.gravity * std::cos(state.target));
      }
      assert(std::isfinite(state.output.torque));
      assert(std::fabs(state.output.torque) <= pitch_param.torque_limit);
    }
  }
  else if (scenario == "boundary_feedforward")
  {
    for (float sign : {-1.0f, 1.0f})
    {
      attitude.pitch = -sign * .2f;
      fresh();
      run_thread_once(task);
      pitch.SetPath(Pitch::Path::AIM);
      const float mechanical = sign < 0 ? pitch_param.minimum : pitch_param.maximum;
      const float target = attitude.pitch + mechanical - pitch.GetState().relative;
      // 目标恰好在机械边界时也应撤销向外速度，而非只有目标越界才撤销。
      pitch.SetAbsolute(target, sign * 2, sign * 3);
      pitch.ControlLoop(.001f);
      near(pitch.GetState().output.velocity, 0);
      pitch.SetAbsolute(target, -sign * 2, -sign * 3);
      pitch.ControlLoop(.001f);
      near(pitch.GetState().output.velocity, -sign * 2);
    }
  }
  else if (scenario == "navigation")
  {
    cmd.GetEvent().Active(uint32_t(CMD::Mode::CMD_AUTO_CTRL));
    CMD::Data ai{};
    ai.gimbal.yaw = .2f;
    cmd.FeedAI(CMD::Channel::GIMBAL, ai);
    tick();
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::AUTO));
    tick();
    auto chassis_topic =
        Topic::FindOrCreate<HostData::HostChassisTarget>("host_chassis_data");
    for (auto mode : {CMD::GimbalAutoMode::AIM, CMD::GimbalAutoMode::SCAN})
    {
      ai.gimbal.auto_mode = mode;
      cmd.FeedAI(CMD::Channel::GIMBAL, ai);
      tick();
      const float yaw_target = yaw.GetState().axes[0].target;
      const float drag_target = yaw.GetState().drag_target;
      for (float speed : {-2.0f, 2.0f})
      {
        chassis_topic.Publish(HostData::HostChassisTarget{speed, .5f, .3f});
        tick();
        const auto chassis =
            std::any_cast<CMD::ChassisCMD>(Topic::values.at("chassis_cmd"));
        near(chassis.x, speed);
        assert(chassis.frame == CMD::ChassisFrame::GIMBAL);
        near(yaw.GetState().axes[0].target, yaw_target);
        near(yaw.GetState().drag_target, drag_target);
        assert(yaw.GetState().path == (mode == CMD::GimbalAutoMode::SCAN
                                           ? DualYaw::Path::SCAN
                                           : DualYaw::Path::SMALL_FIRST));
        assert(gimbal.GetState().mode == Gimbal::Mode::AUTO);
      }
    }
    // 旧枚举值不得被误解为新的云台行为。
    for (unsigned value : {2, 3, 5})
    {
      ai.gimbal.auto_mode = static_cast<CMD::GimbalAutoMode>(value);
      cmd.FeedAI(CMD::Channel::GIMBAL, ai);
      tick();
      assert(gimbal.GetState().invalid_input);
      assert(gimbal.GetState().mode == Gimbal::Mode::RELAX);
    }
  }
  else if (scenario == "launcher")
  {
    // 拨弹由云台线程同拍写入：每拍恰好一帧0x1FF，同时含两路yaw和拨弹槽位。
    const auto feed_shooter = [&](float speed)
    {
      const int rpm = int(std::lround(speed / Launcher::Param{}.fric_rpm_to_speed));
      CAN::ClassicPack p{};
      p.dlc = 8;
      for (auto [id, raw] : {std::pair{0x205u, -rpm}, std::pair{0x206u, rpm}})
      {
        p.id = id;
        p.data[2] = uint8_t(uint16_t(int16_t(raw)) >> 8);
        p.data[3] = uint8_t(uint16_t(int16_t(raw)) & 255);
        shoot_bus.Emit(p);
      }
      CAN::ClassicPack t{};
      t.id = 0x207;
      t.dlc = 8;
      can.Emit(t);
    };
    auto frames = [&](size_t from)
    {
      unsigned n = 0;
      for (auto i = from; i < can.sent.size(); ++i) n += can.sent[i].id == 0x1FF;
      return n;
    };
    launcher->GetEvent().Active(uint32_t(Launcher::Event::FRIC_TOGGLE));
    rc.launcher.isfire = true;  // OP开火命令经CMD每帧发布到launcher_cmd。
    // 云台RELAX：停发射联锁，摩擦轮请求保留但不运行。
    auto first = can.sent.size();
    feed_shooter(0);
    tick();
    assert(frames(first) == 1);
    assert(gimbal.GetState().stop_launcher && launcher->GetState().fric_request);
    assert(!launcher->GetState().running);
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::ABSOLUTE_ANGLE));
    feed_shooter(0);
    tick();
    assert(launcher->GetState().running && !launcher->GetState().spun_up);
    for (unsigned i = 0; i < 3; ++i)
    {
      feed_shooter(Launcher::Param{}.initial_speed);
      first = can.sent.size();
      tick();
      assert(frames(first) == 1);
    }
    assert(launcher->GetState().firing && launcher->GetState().trigger_current > 0);
    const auto& frame = can.sent.back();
    assert(frame.id == 0x1FF);
    assert(int16_t((uint16_t(frame.data[4]) << 8) | frame.data[5]) ==
           launcher->GetState().trigger_current);
    // MOTIONLESS仍允许发射（同原车），RELAX撤销许可后拨弹立停。
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::MOTIONLESS));
    feed_shooter(Launcher::Param{}.initial_speed);
    tick();
    assert(!gimbal.GetState().stop_launcher && launcher->GetState().firing);
    gimbal.GetEvent().Active(uint32_t(Gimbal::Mode::RELAX));
    feed_shooter(Launcher::Param{}.initial_speed);
    tick();
    assert(!launcher->GetState().running && launcher->GetState().trigger_current == 0);
  }
  else
    assert(false);
  std::cout << "PASS: gimbal boundary " << scenario << '\n';
}
