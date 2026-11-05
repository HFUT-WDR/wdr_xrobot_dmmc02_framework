#pragma once
#include <cassert>
#include <cmath>
#include <cstring>

#include "dm_session.hpp"
using namespace wdr_imu;
static can_frame_t can_reply(unsigned group, uint16_t id = 0x11)
{
  can_frame_t f{};
  f.id = id;
  f.size = 8;
  f.data[0] = group + 1;
  f.data[1] = 30;
  const uint16_t v[3] = {
      32768, 32768,
      group == 0 ? uint16_t((9.80665f + 235.2f) / 470.4f * 65535) : uint16_t(32768)};
  for (unsigned i = 0; i < 3; ++i)
  {
    f.data[2 + i * 2] = uint8_t(v[i]);
    f.data[3 + i * 2] = uint8_t(v[i] >> 8);
  }
  if (group == 3)
  {
    const unsigned q[4] = {16383, 8192, 8192, 8192};
    f.data[1] = q[0] >> 6;
    f.data[2] = uint8_t((q[0] & 63) << 2 | (q[1] >> 12));
    f.data[3] = q[1] >> 4;
    f.data[4] = uint8_t((q[1] & 15) << 4 | (q[2] >> 10));
    f.data[5] = q[2] >> 2;
    f.data[6] = uint8_t((q[2] & 3) << 6 | (q[3] >> 8));
    f.data[7] = q[3];
  }
  return f;
}
static void serial_reply(uint8_t* p, unsigned group)
{
  std::memset(p, 0, 24);
  p[0] = 0xa5;
  p[1] = 0x0d;
  p[2] = 1;
  p[3] = group;
  p[23] = 0x5a;
  float a[4] = {0, 0, 0, 0};
  if (group == 0) a[2] = 9.80665f;
  if (group == 3) a[0] = 1;
  for (unsigned i = 0; i < (group == 3 ? 4U : 3U); ++i) putfloat(p + 5 + 4 * i, a[i]);
}
static void test_sessions()
{
  DmConfig p{};
  assert(CheckDmConfig(p));
  auto invalid = p;
  invalid.data_mask = 0;
  assert(!CheckDmConfig(invalid));
  invalid = p;
  invalid.transport = DmTransport(99);
  assert(!CheckDmConfig(invalid));
  invalid = p;
  invalid.transport = DmTransport::RS485_REQUEST;
  invalid.device_id = 256;
  assert(!CheckDmConfig(invalid));
  DmSession can(p);
  DmRequest tx;
  uint64_t now = 1000;
  assert(!can.AcceptCan(can_reply(0), now));  // 未请求的数据不能当作当前周期。
  for (unsigned g = 0; g < 4; ++g)
  {
    assert(can.NextRequest(now, tx));
    assert(tx.can.id == 1 && tx.can.size == 8 && tx.can.data[0] == 0xcc &&
           tx.can.data[1] == g + 1 && tx.can.data[2] == 0 && tx.can.data[3] == 0xdd);
    assert(!can.NextRequest(now + 1, tx));
    assert(!can.AcceptCan(can_reply(g, 0x12), now + 2));
    assert(can.AcceptCan(can_reply(g), now + 3) == (g == 3));
    now += 1000;
  }
  auto raw = can.Measurement();
  assert(raw.data_mask == 15 && raw.sequence == 1 && raw.temperature_present &&
         raw.temperature == 30);
  Attitude state{};
  assert(DeviceAttitude(raw, IDENTITY, state));
  assert(std::fabs(state.yaw) < .001f);
  assert(!can.NextRequest(10999, tx));
  assert(can.NextRequest(11000, tx));
  // CAN四元数的14位边界保持完整，非合法ID/长度/RID不刷新状态。
  auto bad = can_reply(0);
  bad.size = 7;
  assert(!can.AcceptCan(bad, 11001));
  assert(can.Measurement().sequence == 1);
  p.retries = 1;
  p.timeout_ms = 2;
  DmSession timed(p);
  assert(timed.NextRequest(0, tx));
  assert(!timed.NextRequest(1999, tx));
  assert(timed.NextRequest(2000, tx));
  assert(!timed.AcceptCan(can_reply(0), 4000));
  assert(!timed.NextRequest(4000, tx));
  assert(timed.Diagnostics().timeouts == 2 && timed.Diagnostics().retries == 1 &&
         timed.Diagnostics().failed_cycles == 1);
  assert(!timed.NextRequest(13999, tx));
  assert(timed.NextRequest(14000, tx));
  timed.TransmitFailed(14001);
  assert(timed.Diagnostics().tx_errors == 1);
  // RS485每个分块位置、合法零值、错误应答与重新同步。
  p.transport = DmTransport::RS485_REQUEST;
  p.retries = 0;
  p.timeout_ms = 50;
  for (unsigned split = 0; split <= 24; ++split)
  {
    DmSession serial(p);
    DmReplyStream stream;
    now = UINT32_MAX - 100;
    for (unsigned g = 0; g < 4; ++g)
    {
      assert(serial.NextRequest(now, tx));
      assert(tx.serial[0] == 0xa5 && tx.serial[1] == 0x0d && tx.serial[2] == 1 &&
             tx.serial[3] == g && tx.serial[23] == 0x5a);
      uint8_t reply[24];
      serial_reply(reply, g);
      stream.Push(reply, split, now + 1, [&](const uint8_t* bytes, uint64_t t)
                  { serial.AcceptSerial(bytes, 24, t); });
      stream.Push(reply + split, 24 - split, now + 2,
                  [&](const uint8_t* bytes, uint64_t t)
                  { serial.AcceptSerial(bytes, 24, t); });
      now += 1000;
    }
    assert(serial.Measurement().sequence == 1 &&
           !serial.Measurement().temperature_present);
    assert(serial.Measurement().group_time_us[3] > UINT32_MAX);
    assert(DeviceAttitude(serial.Measurement(), IDENTITY, state));
    assert(std::fabs(state.yaw) < 1e-6);
  }
  DmSession bad_serial(p);
  assert(bad_serial.NextRequest(1, tx));
  uint8_t response[24];
  serial_reply(response, 0);
  response[21] = 1;
  assert(!bad_serial.AcceptSerial(response, 24, 2));
  response[21] = 0;
  response[2] = 2;
  assert(!bad_serial.AcceptSerial(response, 24, 3));
  response[2] = 1;
  putfloat(response + 5, NAN);
  assert(!bad_serial.AcceptSerial(response, 24, 4));
  assert(bad_serial.Diagnostics().responses == 0);
  serial_reply(response, 0);
  assert(!bad_serial.AcceptSerial(response, 24, 5));
  assert(bad_serial.Diagnostics().responses == 1);
  // 旋转的不只是向量，设备姿态也转换到同一机构坐标。覆盖trace<=0分支。
  raw = {};
  raw.data_mask = 15;
  raw.quaternion[0] = std::sqrt(.5f);
  raw.quaternion[3] = std::sqrt(.5f);
  raw.accel[0] = 1;
  const Installation rz90 = {0, -1, 0, 1, 0, 0, 0, 0, 1};
  assert(DeviceAttitude(raw, rz90, state));
  assert(std::fabs(state.yaw) < 1e-5 && state.accel[1] == 1);
  raw.quaternion[0] = 0;
  raw.quaternion[3] = 1;
  const Installation rz180 = {-1, 0, 0, 0, -1, 0, 0, 0, 1};
  assert(DeviceAttitude(raw, rz180, state));
  assert(std::fabs(state.yaw) < 1e-5);
  raw.quaternion[3] = 0;
  assert(!DeviceAttitude(raw, IDENTITY, state));
  // 57B兼容帧不能把先前80B的四元数标成新数据。
  p.transport = DmTransport::RS485_STREAM_LEGACY;
  p.period_ms = 1;
  DmSession legacy(p);
  dm_imu_t::state_t previous{};
  previous.valid_mask = 15;
  previous.sequence[3] = 1;
  legacy.AcceptLegacy(previous, 1);
  assert(legacy.Measurement().data_mask == 15);
  legacy.AcceptLegacy(previous, 2);
  assert(legacy.Measurement().data_mask == 7);
  assert(!legacy.NextRequest(3, tx));
  puts(
      "PASS: CAN/RS485 requests, four groups, raw attitude/matrix, deadlines/retries, "
      "malformed replies, no unsolicited CAN mode");
}
