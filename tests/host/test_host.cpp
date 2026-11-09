#include <cassert>
#include <chrono>
#include <fstream>
#include <iterator>
#include <thread>
#include <vector>

#include "HostData.hpp"

// 离线测试不启动机构遥测。
void HostData::StartTelemetry() { std::abort(); }
struct Capture
{
  CMD::GimbalCMD gimbal{};
  CMD::ChassisCMD chassis{};
  CMD::LauncherCMD launcher{};
  unsigned count{};
};
int main(int argc, char** argv)
{
  assert(argc == 3);
  CMD cmd(CMD::Mode::CMD_AUTO_CTRL, "test_chassis", "test_gimbal", "test_launcher",
          CMD::ChassisWzSource::RC);
  HostData host(cmd);
  Capture capture;
  auto gimbal_cb = LibXR::Topic::Callback::Create(
      [](bool, Capture* c, const CMD::GimbalCMD& data)
      { c->gimbal = data; ++c->count; }, &capture);
  auto chassis_cb = LibXR::Topic::Callback::Create(
      [](bool, Capture* c, const CMD::ChassisCMD& data) { c->chassis = data; }, &capture);
  auto fire_cb = LibXR::Topic::Callback::Create(
      [](bool, Capture* c, const CMD::LauncherCMD& data) { c->launcher = data; }, &capture);
  LibXR::Topic(LibXR::Topic::Find("test_gimbal")).RegisterCallback(gimbal_cb);
  LibXR::Topic(LibXR::Topic::Find("test_chassis")).RegisterCallback(chassis_cb);
  LibXR::Topic(LibXR::Topic::Find("test_launcher")).RegisterCallback(fire_cb);
  CMD::Data rc{};
  rc.input_online = true;
  rc.chassis.z = 6;
  cmd.FeedRC(rc);
  wdr_host::AimCommand aim{};
  aim.mode = 2;
  aim.fire_control = 1;
  aim.yaw = 1.25f;
  aim.pitch = -0.25f;
  aim.distance = 6;
  assert(host.ReceiveAim(aim));
  assert(capture.gimbal.valid && capture.launcher.isfire);
  assert(capture.gimbal.distance == 6);
  assert(capture.chassis.x == 0);
  wdr_host::NavCommand nav{2, -1, 0.5f, 1, 2, 3};
  host.ReceiveNav(nav);
  assert(capture.chassis.x == 2 && capture.chassis.z == 6);
  aim.mode = 1;
  host.ReceiveAim(aim);
  assert(capture.gimbal.auto_mode == CMD::GimbalAutoMode::SCAN && !capture.launcher.isfire);
  for (uint8_t mode : {3, 4, 5, 255})
  {
    aim.mode = mode;
    const auto count = capture.count;
    assert(!host.ReceiveAim(aim) && capture.count == count);
  }
  // 导航停更期间持续更新aim，导航必须独立超时。
  aim.mode = 2;
  for (unsigned i = 0; i < 12; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    host.ReceiveAim(aim);
    cmd.FeedRC(rc);
  }
  assert(capture.gimbal.valid && capture.launcher.isfire && capture.chassis.x == 0);
  // 反向验证：nav更新不能刷新aim/开火。
  for (unsigned i = 0; i < 12; ++i)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
    host.ReceiveNav(nav);
    cmd.FeedRC(rc);
  }
  assert(!capture.gimbal.valid && !capture.launcher.isfire);
  LibXR::Topic::Server parser(256);
  parser.Register(LibXR::Topic::Find("host_aim"));
  std::ifstream input(argv[1], std::ios::binary);
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
  const auto before = capture.count;
  for (auto byte : bytes) parser.ParseData({&byte, 1});
  assert(capture.count == before + 2);
  assert(capture.gimbal.yaw == 1.25f);
  std::thread remote([&] { for (unsigned i = 0; i < 1000; ++i) cmd.FeedRC(rc); });
  std::thread ai([&] { for (unsigned i = 0; i < 1000; ++i) host.ReceiveAim(aim); });
  remote.join();
  ai.join();
  aim = {1234567, 1.25f, 2, 3, -0.25f, 4, 5, 6, 7, 2, 1, 1, 3, 4};
  uint8_t packed[sizeof(aim) + LibXR::Topic::PACK_BASE_SIZE]{};
  LibXR::Topic topic(LibXR::Topic::Find("host_aim"));
  assert(topic.PackRaw({&aim, sizeof(aim)}, {packed, sizeof(packed)},
                      LibXR::MicrosecondTimestamp(7654321)) == LibXR::ErrorCode::OK);
  CMD local(CMD::Mode::CMD_AUTO_CTRL, "local_chassis", "local_gimbal", "local_launcher",
            CMD::ChassisWzSource::LOCAL);
  CMD::ChassisCMD local_output{};
  auto local_cb = LibXR::Topic::Callback::Create(
      [](bool, CMD::ChassisCMD* result, const CMD::ChassisCMD& input)
      { *result = input; }, &local_output);
  LibXR::Topic(LibXR::Topic::Find("local_chassis")).RegisterCallback(local_cb);
  local.FeedRC(rc);
  CMD::Data local_aim{};
  local_aim.gimbal.auto_mode = CMD::GimbalAutoMode::AIM;
  local_aim.gimbal.distance = 5;
  local.FeedAIAim(local_aim);
  assert(local_output.local_spin && local_output.aiming);
  assert(local_output.target_distance == 5 && local_output.z == 0);
  local.GetEvent().Active(uint32_t(CMD::Mode::CMD_OP_CTRL));
  local.FeedRC(rc);
  assert(!local_output.local_spin && local_output.z == 6);
  std::ofstream output(argv[2], std::ios::binary);
  output.write(reinterpret_cast<char*>(packed), sizeof(packed));
}
