#include <cassert>
#include <cmath>
#include <cstring>
#include <limits>
#include "I6x.hpp"

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

  CMD command;
  CMD::Data ai{};
  ai.chassis.x = .7f;
  ai.gimbal.yaw = .8f;
  ai.chassis_online = ai.gimbal_online = true;
  ai.launcher.isfire = true;
  LibXR::Timebase::now = 1000;
  command.FeedAI(ai);
  I6xInput::State state{};
  state.online = true;
  state.channels[1] = 330;
  state.channels[2] = 330;
  state.switches = {1, -1, 0, 1};
  auto knob = I6x::MakeIntent(state);
  knob.receiver.channels[4] = -660;
  assert(I6x::MakeCommand(knob).chassis.z == 0);
  knob.receiver.channels[4] = 0;
  assert(I6x::MakeCommand(knob).chassis.z == .5f);
  knob.receiver.channels[4] = 660;
  assert(I6x::MakeCommand(knob).chassis.z == 1);
  auto submit = [&]()
  {
    command.FeedRC(CMD::RCInputSource::RC_INPUT_I6X,
                   I6x::MakeCommand(I6x::MakeIntent(state)));
  };
  submit();
  assert(chassis().x == .5f && gimbal().yaw == -.5f && fire());
  // 验证独立拨杆、停机，以及上位机数据过期后不回退到手动输入。
  state.switches[2] = 1;
  submit();
  assert(chassis().x == .7f && gimbal().yaw == .8f);
  LibXR::Timebase::now = 1100;
  submit();
  assert(chassis().x == 0 && gimbal().yaw == 0);
  state.switches = {-1, 0, -1, -1};
  command.FeedAI(ai);
  submit();
  assert(chassis().x == 0 && gimbal().yaw == 0 && !fire());
  state.online = false;
  submit();
  assert(!command.Online() && !fire());
  // 摩擦轮切换按输入边沿计数，保持拨杆时不重复触发。
  LibXR::UART uart;
  I6x receiver(uart, command);
  state.online = true;
  state.switches[1] = 1;
  assert(receiver.UpdateIntent(state, true).friction_toggle_sequence == 0);
  state.switches[1] = -1;
  receiver.UpdateIntent(state, true);
  state.switches[1] = 1;
  assert(receiver.UpdateIntent(state, true).friction_toggle_sequence == 1);
  assert(receiver.UpdateIntent(state, true).friction_toggle_sequence == 1);
  state.switches[1] = -1;
  receiver.UpdateIntent(state, true);
  state.frame_lost = true;
  state.switches[1] = 1;
  assert(receiver.UpdateIntent(state, true).friction_toggle_sequence == 1);
  state.online = false;
  receiver.UpdateIntent(state, false);
  state.online = true;
  state.frame_lost = false;
  assert(receiver.UpdateIntent(state, true).friction_toggle_sequence == 1);
  // 验证摇杆归零时在线、各轴独立停机，以及计时回绕后的上位机数据过期。
  LibXR::Timebase::now = UINT32_MAX - 20;
  command.FeedAI(ai);
  state.channels.fill(0);
  state.switches = {-1, -1, 1, -1};
  submit();
  assert(chassis().x == 0 && gimbal().yaw == .8f);
  LibXR::Timebase::now = 50;
  submit();
  assert(gimbal().yaw == .8f);
  LibXR::Timebase::now = 80;
  submit();
  assert(gimbal().yaw == 0 && !fire());
  state.online = false;
  submit();
  // 验证原 DR16 路径及活动输入源离线后的切换仍可使用。
  CMD::Data dr16{};
  dr16.chassis_online = true;
  dr16.chassis.x = -.25f;
  command.FeedRC(CMD::RCInputSource::RC_INPUT_DR16, dr16);
  assert(chassis().x == -.25f);
}
