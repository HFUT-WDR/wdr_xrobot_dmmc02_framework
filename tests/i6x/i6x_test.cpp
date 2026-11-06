#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include "I6x.hpp"
#include "EventBinder.hpp"
#include "HostData.hpp"

extern "C"
{
#include "i6x.h"
}

using Frame = std::array<uint8_t, 25>;
Frame pack(const std::array<int, 10>& channels, uint8_t flags = 0)
{
  Frame frame{};
  frame[0] = 0x0f;
  frame[23] = flags;
  for (size_t channel = 0; channel < channels.size(); ++channel)
  {
    for (unsigned bit = 0; bit < 11; ++bit)
    {
      const size_t offset = 11 * channel + bit;
      frame[1 + offset / 8] |= ((channels[channel] >> bit) & 1) << (offset % 8);
    }
  }
  return frame;
}
void feed(I6xInput& input, const Frame& frame, uint32_t now)
{
  for (auto byte : frame)
  {
    input.Receive(byte, now);
  }
}
CMD::ChassisCMD chassis()
{
  return std::any_cast<CMD::ChassisCMD>(LibXR::Topic::values.at("chassis_cmd"));
}
CMD::GimbalCMD gimbal()
{
  return std::any_cast<CMD::GimbalCMD>(LibXR::Topic::values.at("gimbal_cmd"));
}
bool fire()
{
  return std::any_cast<CMD::LauncherCMD>(LibXR::Topic::values.at("launcher_cmd")).isfire;
}
int main()
{
  // 与未修改的 26_sentry C 源码逐项对照通道值和拨杆符号。
  for (int value = 240; value <= 1807; ++value)
  {
    std::array<int, 10> channels{};
    for (int i = 0; i < 10; ++i)
    {
      channels[i] = 240 + (value - 240 + i * 137) % 1568;
    }
    const auto frame = pack(channels);
    i6x_ctrl_t frozen{};
    sbus_to_i6x(&frozen, frame.data());
    I6xInput input;
    feed(input, frame, 10);
    const auto& state = input.Update(10);
    assert(state.online && state.sequence == 1);
    for (size_t i = 0; i < 6; ++i) { assert(state.channels[i] == frozen.ch[i]); }
    for (size_t i = 0; i < 4; ++i) { assert(state.switches[i] == frozen.s[i]); }
  }
  const std::array<int, 10> remote = {1807, 240, 1024, 1807, 1807, 1024, 240, 1807, 1024, 240};
  const auto valid = pack(remote);
  I6xInput input;
  input.Receive(0xaa, 0);
  feed(input, valid, 20);
  assert(input.Update(20).channels[0] == 660);
  auto broken = valid;
  broken[24] = 0xff;
  feed(input, broken, 30);
  feed(input, valid, 40);
  assert(input.Update(40).sequence == 2);
  const auto switches = input.Update(40).switches;
  feed(input, pack(remote, 4), 50);
  assert(input.Update(50).online && input.Update(50).channels[0] == 0);
  assert(input.Update(50).switches == switches && input.Update(50).received_ms == 40);
  feed(input, pack(remote, 8), 60);
  assert(input.Update(60).online);
  feed(input, pack(remote, 12), 70);
  assert(!input.Update(70).online);
  feed(input, valid, 80);
  assert(input.Update(110).online && !input.Update(111).online);
  feed(input, valid, UINT32_MAX - 20);
  assert(input.Update(9).online && !input.Update(10).online);
  I6xInput half;
  for (size_t i = 0; i < 24; ++i) { half.Receive(valid[i], 0); }
  assert(!half.Update(0).online);

  // 输入端死区与量程，PC的小目标不经过此映射。
  I6xInput::State mapping{};
  mapping.channels[1] = 10;
  assert(I6x::MakeCommand(mapping).chassis.x == 0);
  mapping.channels[1] = 11;
  assert(I6x::MakeCommand(mapping).chassis.x > 0);
  mapping.channels[1] = 660;
  mapping.channels[4] = 660;
  const auto mapped = I6x::MakeCommand(mapping, {3, 4, 5, .02f});
  assert(mapped.chassis.x == 3 && mapped.chassis.z == 5);
  CMD command;
  LibXR::UART uart;
  I6x receiver(uart, command);
  struct ModeProbe
  {
    LibXR::Event event;
    int mode = 0;
    LibXR::Event& GetEvent() { return event; }
  } chassis_probe;
  chassis_probe.event.Register(0, [&](bool, uint32_t) { chassis_probe.mode = 0; });
  chassis_probe.event.Register(1, [&](bool, uint32_t) { chassis_probe.mode = 1; });
  // 测试替身仅观察事件，生产代码中不创建虚假机构任务。
  command.GetEvent().Register(CMD::CMD_EVENT_LOST_CTRL,
      [&](bool, uint32_t) { chassis_probe.mode = 0; });
  command.GetEvent().Register(CMD::CMD_EVENT_START_CTRL,
      [&](bool, uint32_t) { chassis_probe.mode = 0; });
  using Pos = I6x::SwitchPos;
  using Chord = I6x::Chord;
  EventBinder binder({{"i6x", receiver}, {"cmd", command}, {"chassis", chassis_probe}},
    {{{{"i6x", Pos::SW2_MID, "cmd", CMD::Mode::CMD_OP_CTRL},
       {"i6x", Pos::SW2_UP, "cmd", CMD::Mode::CMD_AUTO_CTRL},
       {"i6x", Chord::SW2_MID_SW3_UP_ENTER, "chassis", 1},
       {"i6x", Chord::SW2_UP_SW3_UP_ENTER, "chassis", 1},
       {"i6x", Chord::SW2_UP_SW3_UP_EXIT, "chassis", 0},
       {"i6x", Chord::SW2_MID_SW3_UP_EXIT, "chassis", 0}}}});
  unsigned friction_edges = 0;
  receiver.GetEvent().Register(static_cast<uint32_t>(Pos::SW1_UP),
                              [&](bool, uint32_t) { ++friction_edges; });
  I6xEvents events;
  I6xInput::State state{};
  state.online = true;
  state.channels[1] = 330;
  state.channels[2] = 330;
  state.switches = {-1, -1, -1, -1};
  auto submit = [&](bool fresh = true)
  {
    events.Update(state, fresh, [&](uint32_t id) { receiver.GetEvent().Active(id); });
    command.FeedRC(CMD::RCInputSource::RC_INPUT_I6X, I6x::MakeCommand(state));
  };
  submit();
  assert(command.Online() && chassis_probe.mode == 0 && friction_edges == 0);
  // 源选择由真正的 EventBinder 转发，不依赖任何 I6x 专用 CMD 策略。
  state.switches[2] = 0;
  state.switches[3] = 1;
  submit();
  assert(chassis_probe.mode == 1 && chassis().x == .5f * 2.64f && gimbal().yaw == -.5f);
  assert(command.GetCtrlMode() == CMD::Mode::CMD_OP_CTRL);
  CMD::Data ai{};
  ai.chassis.x = .7f;
  ai.gimbal.yaw = .8f;
  ai.launcher.isfire = true;
  LibXR::Timebase::now = 1000;
  command.FeedAI(CMD::Channel::CHASSIS, ai);
  command.FeedAI(CMD::Channel::GIMBAL, ai);
  command.FeedAI(CMD::Channel::LAUNCHER, ai);
  state.switches[2] = 1;
  submit();
  assert(chassis().x == .7f && gimbal().yaw == .8f);
  // AUTO开火不受遥控开火拨杆影响。
  assert(command.GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL);
  assert(fire());
  state.switches[1] = 0;
  submit();
  assert(fire());
  state.switches[1] = -1;
  submit();
  assert(fire());
  // PC 组合切到 RC 组合时，退出事件不会覆盖新选择。
  state.switches[2] = 0;
  submit();
  assert(chassis().x == .5f * 2.64f && chassis_probe.mode == 1);
  state.switches[2] = 1;
  submit();
  assert(chassis().x == .7f);
  assert(command.GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL);
  // 单帧坏帧不新增 CMD 有效位判据，沿用链路在线状态。
  state.frame_lost = true;
  submit();
  assert(chassis_probe.mode == 1 && chassis().x == .7f && gimbal().yaw == .8f && fire());
  state.frame_lost = false;
  submit();
  LibXR::Timebase::now = 1100;
  submit(false);
  assert(chassis().x == 0 && gimbal().yaw == 0);
  // 摩擦轮只提供拨杆进入事件：保持位置、其他拨杆变化及坏帧均不重复开关。
  state.switches[1] = 1;
  submit();
  assert(friction_edges == 1);
  submit();
  state.switches[0] = 1;
  submit();
  assert(friction_edges == 1);
  state.switches[1] = -1;
  state.failsafe = true;
  submit();
  state.switches[1] = 1;
  state.failsafe = false;
  submit();
  assert(friction_edges == 1);
  // 离线后恢复保持来源，首帧不凭当前档位重新使能机构。
  state.online = false;
  submit(false);
  assert(!command.Online() && chassis_probe.mode == 0 && !fire());
  state.online = true;
  submit();
  assert(chassis_probe.mode == 0 && friction_edges == 1);
  assert(command.GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL);
  // 从两个独立事件分别设置来源；上位机数据过期跨计时回绕后仍归零。
  command.GetEvent().Active(static_cast<uint32_t>(CMD::Mode::CMD_AUTO_CTRL));
  LibXR::Timebase::now = UINT32_MAX - 20;
  command.FeedAI(CMD::Channel::CHASSIS, ai);
  command.FeedAI(CMD::Channel::GIMBAL, ai);
  command.FeedAI(CMD::Channel::LAUNCHER, ai);
  submit(false);
  assert(chassis().x == .7f && gimbal().yaw == .8f);
  LibXR::Timebase::now = 80;
  submit(false);
  assert(gimbal().yaw == 0);
  // 全局AUTO独立开火，不要求遥控扳机许可。
  command.SetCtrlMode(CMD::Mode::CMD_AUTO_CTRL);
  LibXR::Timebase::now = 2000;
  command.FeedAI(CMD::Channel::CHASSIS, ai);
  command.FeedAI(CMD::Channel::GIMBAL, ai);
  command.FeedAI(CMD::Channel::LAUNCHER, ai);
  state.switches[0] = -1;
  submit(false);
  assert(fire());
  state.switches[0] = 1;
  submit(false);
  assert(fire());
  state.online = false;
  submit(false);
  CMD::Data dr16{};
  dr16.input_online = true;
  dr16.chassis.x = -.25f;
  command.FeedRC(CMD::RCInputSource::RC_INPUT_DR16, dr16);
  assert(command.GetCtrlMode() == CMD::Mode::CMD_AUTO_CTRL);
  assert(chassis().x == .7f);
  command.SetCtrlMode(CMD::Mode::CMD_OP_CTRL);
  command.FeedRC(CMD::RCInputSource::RC_INPUT_DR16, dr16);
  assert(chassis().x == -.25f);
  // 真实 HostData 回调：零角度保留导数，各通道接收独立刷新时效。
  CMD pc_command;
  HostData host(pc_command);
  CMD::Data rc{};
  rc.input_online = true;
  rc.launcher.isfire = true;
  pc_command.FeedRC(rc);
  pc_command.SetCtrlMode(CMD::Mode::CMD_AUTO_CTRL);
  auto poll = [&]() { pc_command.FeedRC(rc); };
  LibXR::Timebase::now = 3000;
  HostData::HostChassisTarget target{.6f, .2f, .1f};
  auto chassis_topic = LibXR::Topic::CreateTopic<HostData::HostChassisTarget>("host_chassis_data");
  auto gimbal_topic = LibXR::Topic::CreateTopic<HostData::HostGimbalTarget>("target_euler");
  auto fire_topic = LibXR::Topic::CreateTopic<HostData::LauncherCMD>("host_fire_notify");
  chassis_topic.Publish(target);
  assert(chassis().x == .6f && gimbal().yaw_dot == 0 && !fire());
  target.vx = .01f;
  chassis_topic.Publish(target);
  assert(chassis().x == .01f);  // PC低速透传，不套遥控死区。
  LibXR::Timebase::now = 3050;
  HostData::HostGimbalTarget aim{};
  aim.yaw_dot = 2.0f; aim.rol = .3f; aim.rol_ddot = .4f;
  gimbal_topic.Publish(aim);
  HostData::LauncherCMD shot{true};
  fire_topic.Publish(shot);
  assert(gimbal().yaw == 0 && gimbal().pit == 0 && gimbal().yaw_dot == 2.0f);
  assert(gimbal().rol == .3f && gimbal().rol_ddot == .4f && fire());
  LibXR::Timebase::now = 3100;
  poll();
  assert(chassis().x == 0 && gimbal().yaw_dot == 2.0f && fire());
  // 持续云台收包不能延长旧开火命令，也不能延长旧底盘命令。
  LibXR::Timebase::now = 3150;
  gimbal_topic.Publish(aim);
  assert(chassis().x == 0 && !fire() && gimbal().yaw_dot == 2.0f);
  target = {}; chassis_topic.Publish(target);
  assert(chassis().x == 0 && gimbal().yaw_dot == 2.0f);
  rc.input_online = false; poll();
  assert(!pc_command.Online() && gimbal().yaw_dot == 0 && !fire());

}
