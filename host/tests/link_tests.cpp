#include <fcntl.h>
#include <pty.h>
#include <unistd.h>

#include <cassert>
#include <cstdio>
#include <cstdlib>

#include "HostLink.hpp"

struct Board
{
  int master;
  uint64_t origin{wdr_host::steady_us()};
  uint32_t boot{7};
  wdr_host::Command command{};
  unsigned command_count{};
  bool forbid_fire{};
  LibXR::Topic::Server parser{512};
  uint64_t Mcu() const { return wdr_host::steady_us() - origin + 1000000; }
  template <typename T>
  void Send(const T& data, const char* name)
  {
    uint8_t packet[256];
    LibXR::Topic topic(LibXR::Topic::Find(name));
    topic.PackRaw({&data, sizeof(data)}, {packet, sizeof(packet)},
                  LibXR::MicrosecondTimestamp(Mcu()));
    const auto size = sizeof(data) + LibXR::Topic::PACK_BASE_SIZE;
    assert(write(master, packet, size) == static_cast<ssize_t>(size));
  }
  explicit Board(int fd) : master(fd)
  {
    auto request_cb = LibXR::Topic::Callback::Create(
        [](bool, Board* self, const wdr_host::TimeRequest& request)
        {
          wdr_host::TimeResponse response{request.seq, self->boot, request.t1_host_us,
                                          self->Mcu(), self->Mcu()};
          self->Send(response, "robot_time_sync");
        },
        this);
    auto command_cb = LibXR::Topic::Callback::Create(
        [](bool, Board* self, const wdr_host::Command& command)
        {
          assert(!self->forbid_fire || command.aim.fire_control == 0);
          self->command = command;
          ++self->command_count;
        },
        this);
    LibXR::Topic(LibXR::Topic::Find("host_time_sync")).RegisterCallback(request_cb);
    LibXR::Topic(LibXR::Topic::Find("host_command")).RegisterCallback(command_cb);
    parser.Register(LibXR::Topic::Find("host_time_sync"));
    parser.Register(LibXR::Topic::Find("host_command"));
  }
  void Pump(unsigned milliseconds, bool aim_on, bool nav_on)
  {
    LibXR::Topic aim(LibXR::Topic::Find("aim_intent"));
    LibXR::Topic nav(LibXR::Topic::Find("nav_intent"));
    const auto end = wdr_host::steady_us() + milliseconds * 1000;
    uint64_t next = 0;
    while (wdr_host::steady_us() < end)
    {
      const auto now = wdr_host::steady_us();
      if (now >= next)
      {
        wdr_host::AimState state{};
        state.boot_id = boot;
        state.imu_time_us = Mcu();
        state.sample_seq = state.imu_time_us;
        state.valid_mask = 1;
        Send(state, "robot_aim_state");
        if (aim_on)
        {
          wdr_host::AimIntent input{};
          input.aim.mode = 2;
          input.aim.fire_control = 1;
          input.aim.yaw = 1.25f;
          input.steady_time_us = now;
          aim.Publish(input);
        }
        if (nav_on)
        {
          wdr_host::NavIntent input{};
          input.nav.vx = 2;
          input.steady_time_us = now;
          nav.Publish(input);
        }
        next = now + 10000;
      }
      uint8_t buffer[256];
      while (true)
      {
        const auto size = read(master, buffer, sizeof(buffer));
        if (size <= 0) break;
        parser.ParseData({buffer, static_cast<size_t>(size)});
      }
      usleep(2000);
    }
  }
};
int main()
{
  int master, slave;
  char path[128];
  assert(openpty(&master, &slave, path, nullptr, nullptr) == 0);
  fcntl(master, F_SETFL, O_NONBLOCK);
  const auto stable = std::string("/tmp/wdr-host-uart-") + std::to_string(getpid());
  assert(symlink(path, stable.c_str()) == 0);
  auto uart = new LibXR::LinuxUART(stable.c_str(), 115200, LibXR::UART::Parity::NO_PARITY,
                                   8, 1, 1, 512, 65536);
  uart->SetRealtimeTxLifetime(100);
  new HostLink(*uart, true);
  Board board(master);
  board.Pump(1400, true, true);
  assert(board.command_count > 100 && board.command.valid_mask == 7);
  assert(board.command.aim.mode == 2 && board.command.aim.fire_control == 1);
  assert(board.command.nav.vx == 2 && board.command.target_time_mcu_us > 0);
  board.Pump(160, true, false);
  assert(board.command.valid_mask == 5 && board.command.nav.vx == 0);
  board.Pump(160, false, true);
  assert(board.command.valid_mask == 2 && board.command.aim.mode == 0);
  assert(board.command.aim.fire_control == 0);
  board.boot = 8;
  board.origin = wdr_host::steady_us();
  board.Pump(30, false, false);
  assert(board.command.valid_mask == 0 && board.command.aim.fire_control == 0);
  board.Pump(1100, true, true);
  assert(board.command.aim.fire_control == 1);
  const auto generation = uart->ConnectionGeneration();
  close(master);
  close(slave);
  assert(openpty(&master, &slave, path, nullptr, nullptr) == 0);
  fcntl(master, F_SETFL, O_NONBLOCK);
  unlink(stable.c_str());
  assert(symlink(path, stable.c_str()) == 0);
  const auto deadline = wdr_host::steady_us() + 3000000;
  while (uart->ConnectionGeneration() == generation && wdr_host::steady_us() < deadline)
    usleep(10000);
  assert(uart->Connected() && uart->ConnectionGeneration() != generation);
  board.master = master;
  board.parser.Reset();
  board.forbid_fire = true;
  board.command_count = 0;
  board.Pump(350, false, false);
  assert(board.command_count > 5 && board.command.valid_mask == 0);
  unlink(stable.c_str());
  std::puts("PASS: LinuxUART PTY, LibXR frames, time sync, independent expiry, reboot");
  std::fflush(stdout);
  std::_Exit(0);
}
