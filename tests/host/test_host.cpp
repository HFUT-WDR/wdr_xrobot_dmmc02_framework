#include <cassert>
#include <chrono>
#include <fstream>
#include <iterator>
#include <thread>
#include <vector>

#include "HostData.hpp"
#include "crc.hpp"

// 该测试仅运行通信适配，不启动机构遥测线程。
void HostData::StartTelemetry() { std::abort(); }

struct Capture
{
  CMD::GimbalCMD gimbal{};
  CMD::ChassisCMD chassis{};
  CMD::LauncherCMD launcher{};
  unsigned count{};
  bool coherent{};
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
      {
        c->gimbal = data;
        if (c->coherent && data.valid) assert(c->chassis.x == data.yaw);
        ++c->count;
      },
      &capture);
  auto chassis_cb = LibXR::Topic::Callback::Create(
      [](bool, Capture* c, const CMD::ChassisCMD& data) { c->chassis = data; }, &capture);
  auto fire_cb =
      LibXR::Topic::Callback::Create([](bool, Capture* c, const CMD::LauncherCMD& data)
                                     { c->launcher = data; }, &capture);
  LibXR::Topic(LibXR::Topic::Find("test_gimbal")).RegisterCallback(gimbal_cb);
  LibXR::Topic(LibXR::Topic::Find("test_chassis")).RegisterCallback(chassis_cb);
  LibXR::Topic(LibXR::Topic::Find("test_launcher")).RegisterCallback(fire_cb);
  CMD::Data rc{};
  rc.input_online = true;
  rc.chassis.z = 6;
  cmd.FeedRC(rc);
  wdr_host::Command command{};
  command.aim.mode = 2;
  command.aim.fire_control = 1;
  command.aim.yaw = 1.25f;
  command.aim.pitch = -0.25f;
  command.nav.vx = 2;
  command.nav.vy = -1;
  command.valid_mask = 7;
  assert(host.Receive(command));
  assert(capture.gimbal.valid && capture.gimbal.auto_mode == CMD::GimbalAutoMode::AIM);
  assert(capture.gimbal.yaw == 1.25f && capture.launcher.isfire);
  assert(capture.chassis.x == 2 && capture.chassis.z == 6);
  command.aim.mode = 1;
  host.Receive(command);
  assert(capture.gimbal.auto_mode == CMD::GimbalAutoMode::SCAN &&
         !capture.launcher.isfire);
  for (uint8_t mode : {3, 4, 5, 255})
  {
    command.aim.mode = mode;
    const auto count = capture.count;
    assert(!host.Receive(command) && capture.count == count);
  }
  command.aim.mode = 2;
  command.valid_mask = 1;
  host.Receive(command);
  assert(capture.chassis.x == 0 && capture.chassis.z == 6 && !capture.launcher.isfire);
  command.valid_mask = 0;
  host.Receive(command);
  assert(!capture.gimbal.valid);

  // 真实LibXR解析：正确帧、错长度、CRC错误、半包/粘包由Python准备。
  LibXR::Topic::Server parser(256);
  parser.Register(LibXR::Topic::Find("host_command"));
  std::ifstream input(argv[1], std::ios::binary);
  std::vector<uint8_t> bytes((std::istreambuf_iterator<char>(input)), {});
  const auto before = capture.count;
  for (auto byte : bytes) parser.ParseData({&byte, 1});
  assert(capture.count == before + 2);
  assert(capture.gimbal.yaw == 1.25f);
  std::this_thread::sleep_for(std::chrono::milliseconds(110));
  cmd.FeedRC(rc);
  assert(!capture.gimbal.valid && !capture.launcher.isfire && capture.chassis.x == 0);

  command.aim.mode = 2;
  command.aim.yaw = 2;
  command.nav.vx = 2;
  command.valid_mask = 7;
  host.Receive(command);
  capture.coherent = true;
  std::thread remote(
      [&]
      {
        for (unsigned i = 0; i < 1000; ++i) cmd.FeedRC(rc);
      });
  std::thread ai(
      [&]
      {
        for (unsigned i = 0; i < 1000; ++i)
        {
          auto input = command;
          input.aim.yaw = input.nav.vx = static_cast<float>(i);
          host.Receive(input);
        }
      });
  remote.join();
  ai.join();
  capture.coherent = false;

  command = {};
  command.target_time_mcu_us = 1234567;
  command.aim = {1.25f, 2, 3, -0.25f, 4, 5, 6, 7, 2, 1, 1, 3, 4};
  command.nav = {2, -1, 0.5f, 1, 2, 3};
  command.valid_mask = 7;
  uint8_t packed[sizeof(command) + LibXR::Topic::PACK_BASE_SIZE]{};
  LibXR::Topic topic(LibXR::Topic::Find("host_command"));
  assert(topic.PackRaw({&command, sizeof(command)}, {packed, sizeof(packed)},
                       LibXR::MicrosecondTimestamp(7654321)) == LibXR::ErrorCode::OK);

  std::ofstream output(argv[2], std::ios::binary);
  output.write(reinterpret_cast<char*>(packed), sizeof(packed));
  return 0;
}
