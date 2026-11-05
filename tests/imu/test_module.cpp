#include <cstdio>

#include "DmImu.hpp"
#include "packets.hpp"
#include "session_tests.hpp"
extern "C" void* tracked_malloc(size_t n) { return std::malloc(n); }
static LibXR::CAN::ClassicPack pack(const wdr_imu::can_frame_t& f)
{
  LibXR::CAN::ClassicPack p{};
  p.id = f.id;
  p.dlc = f.size;
  std::memcpy(p.data, f.data, 8);
  return p;
}
int main()
{
  using namespace LibXR;
  using namespace wdr_imu;
  // 实际模块的CAN回调→槽池→任务→四组请求→姿态Topic，不在回调中发布。
  CAN can;
  DmConfig p{};
  p.output = DmOutput::DEVICE;
  DmImu module(p, &can, nullptr);
  auto* thread = Thread::threads.back();
  Semaphore::on_wait = [&]
  {
    if (!can.sent.empty())
    {
      auto sent = can.sent.front();
      can.sent.pop_front();
      can.Emit(pack(can_reply(sent.data[1] - 1)));
    }
  };
  thread->Run(8);
  assert(std::any_cast<Attitude>(Topic::values.at("dm_attitude")).sequence >= 1);
  assert(std::any_cast<DmDiagnostics>(Topic::values.at("dm_diagnostics")).outputs >= 1);
  // 队列满时丢新并计数；复用真实CAN回调，回调本身不解析/发布。
  unsigned before = std::any_cast<Attitude>(Topic::values.at("dm_attitude")).sequence;
  for (unsigned i = 0; i < 30; ++i) can.Emit(pack(can_reply(0)));
  assert(std::any_cast<Attitude>(Topic::values.at("dm_attitude")).sequence == before);
  thread->Run(1);
  assert(std::any_cast<DmDiagnostics>(Topic::values.at("dm_diagnostics")).rx_drops >= 14);
  // 485异步持久缓冲与24B任意分包。真实模块在TX完成前不能提交第二次发送。
  p.output = DmOutput::FUSED;
  UART uart;
  p.transport = DmTransport::RS485_REQUEST;
  Timebase::now = 1000;
  DmImu serial(p, nullptr, &uart, "serial_attitude", "serial_measurement", "serial_diag");
  thread = Thread::threads.back();
  Semaphore::on_wait = [&]
  {
    if (uart.tx_op)
    {
      auto rid = uart.CompleteTx();
      uint8_t reply[24];
      serial_reply(reply, rid);
      uart.Feed(reply, 7);
      uart.Feed(reply + 7, 17);
    }
  };
  thread->Run(30);
  assert(uart.baud == 2000000);
  assert(std::any_cast<Attitude>(Topic::values.at("serial_attitude")).sequence >= 1);
  assert(std::any_cast<DmDiagnostics>(Topic::values.at("serial_diag")).outputs >= 1);
  UART stalled;
  DmImu stuck(p, nullptr, &stalled, "stuck_attitude", "stuck_measurement", "stuck_diag");
  thread = Thread::threads.back();
  Semaphore::on_wait = [] {};
  thread->Run(100);
  assert(stalled.writes == 1 && stalled.tx_op);
  assert(std::any_cast<DmDiagnostics>(Topic::values.at("stuck_diag")).tx_stalls == 1);
  assert(std::memcmp(stalled.tx.data, stalled.held.data(), 24) == 0);
  assert(Topic::values.find("stuck_attitude") == Topic::values.end());
  // 上一次发送解除后，新发送独立计时，不能沿用旧TX的卡住标志。
  stalled.CompleteTx();
  thread->Run(10);
  assert(stalled.writes == 2);
  assert(std::any_cast<DmDiagnostics>(Topic::values.at("stuck_diag")).tx_stalls == 1);
  thread->Run(50);
  assert(std::any_cast<DmDiagnostics>(Topic::values.at("stuck_diag")).tx_stalls == 2);
  assert(std::memcmp(stalled.tx.data, stalled.held.data(), 24) == 0);
  // 旧固件模式不发任何请求，只接收显式选定的旧流。
  UART legacy;
  p.transport = DmTransport::RS485_STREAM_LEGACY;
  p.period_ms = 1;
  p.baudrate = 4000000;
  DmImu old(p, nullptr, &legacy, "legacy_attitude", "legacy_measurement", "legacy_diag");
  thread = Thread::threads.back();
  bool fed = false;
  Semaphore::on_wait = [&]
  {
    if (!fed)
    {
      uint8_t bytes[80];
      frame(bytes, .1f);
      legacy.Feed(bytes, 80);
      fed = true;
    }
  };
  thread->Run(5);
  assert(legacy.writes == 0);
  assert(std::any_cast<Attitude>(Topic::values.at("legacy_attitude")).sequence == 1);
  puts(
      "PASS: real DmImu CAN/RS485 adapters, ISR copy-only, queue drops, asynchronous "
      "buffer lifetime and explicit legacy mode");
}
