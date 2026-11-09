#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstdlib>
static void* allocations[256];
static unsigned allocation_count;
extern "C" void* tracked_malloc(size_t size)
{
  assert(allocation_count < 256);
  void* p = std::malloc(size);
  assert(p);
  allocations[allocation_count++] = p;
  return p;
}

#include "ImuFusion.hpp"
#include "dm_imu_rs485_stream_assembler.hpp"
#include "packets.hpp"
#include "session_tests.hpp"
using namespace wdr_imu;
struct Capture
{
  unsigned count{};
  uint32_t time{};
  DmFilter filter;
  Attitude state;
};
static void received(void* ptr, const dm_imu_t::state_t& raw, uint32_t time)
{
  auto& c = *static_cast<Capture*>(ptr);
  Sample s{};
  std::memcpy(s.accel, raw.accel, sizeof(s.accel));
  std::memcpy(s.gyro, raw.gyro, sizeof(s.gyro));
  s.time_us = time;
  assert(c.filter.Update(s, c.state));
  c.count++;
  c.time = time;
}
int main()
{
  test_sessions();
  uint8_t bytes[160];
  frame(bytes, .2f);
  frame(bytes + 80, .3f);
  for (unsigned split = 0; split <= 80; ++split)
  {
    dm_imu_t imu;
    assert(imu.init({dm_imu_t::transport_e::RS485, 1, 0x11, 235.2f}));
    dm_imu_rs485_stream_assembler_t stream(imu);
    Capture c;
    stream.push(bytes, split, 100, received, &c);
    stream.push(bytes + split, 80 - split, 101, received, &c);
    assert(c.count == 1 && c.state.sequence == 1);
    uint8_t bad[80];
    std::memcpy(bad, bytes, 80);
    bad[78] ^= 1;
    stream.push(bad, 80, 200, received, &c);
    assert(c.count == 1);
    stream.push(bytes, 80, 300, received, &c);
    assert(c.count == 2);
  }
  dm_imu_t imu;
  assert(imu.init({dm_imu_t::transport_e::RS485, 1, 0x11, 235.2f}));
  dm_imu_rs485_stream_assembler_t stream(imu);
  Capture c;
  stream.push(bytes, 57, UINT32_MAX - 2, received, &c);
  assert(c.count == 0);
  stream.push(bytes, 4, 3, received, &c);
  assert(c.count == 1 && c.time == UINT32_MAX - 2);
  // 现场旧固件整帧：CRC 为设备表驱动算法，冻结厂商帧格式和量纲。
  static const uint8_t vendor_frame[80] = {
      0x55, 0xaa, 0x01, 0x01, 0x0d, 0x47, 0xb3, 0xc0, 0x5f, 0xe3, 0x38, 0x3e,
      0x8e, 0x67, 0x00, 0x41, 0xe4, 0x12, 0x0a, 0x55, 0xaa, 0x01, 0x02, 0x38,
      0xe7, 0x22, 0xbc, 0x6a, 0x6e, 0xe8, 0xbc, 0x68, 0xfb, 0x5d, 0xbb, 0x74,
      0xc6, 0x0a, 0x55, 0xaa, 0x01, 0x03, 0xd8, 0xb4, 0xdd, 0x3f, 0x00, 0xd5,
      0x0a, 0x42, 0xd3, 0x40, 0x94, 0xc1, 0x8f, 0x41, 0x0a, 0x55, 0xaa, 0x01,
      0x04, 0x0d, 0xf2, 0x70, 0x3f, 0xdd, 0x03, 0x7f, 0x3d, 0x6d, 0x84, 0x95,
      0x3e, 0x93, 0xe9, 0x21, 0xbe, 0xf7, 0x3e, 0x0a,
  };
  dm_imu_t vendor_imu;
  assert(vendor_imu.init({dm_imu_t::transport_e::RS485, 1, 0x11, 235.2f}));
  dm_imu_rs485_stream_assembler_t vendor_stream(vendor_imu);
  Capture vendor_capture;
  vendor_stream.push(vendor_frame, sizeof(vendor_frame), 400, received,
                     &vendor_capture);
  assert(vendor_capture.count == 1);
  const auto& decoded = vendor_imu.get_state();
  assert(decoded.valid_mask == 0x0f);
  assert(std::fabs(decoded.accel[0] + 5.602423f) < 1e-4f);
  assert(std::fabs(decoded.accel[2] - 8.025282f) < 1e-4f);
  assert(std::fabs(decoded.gyro[1] + 0.028373f) < 1e-5f);
  assert(std::fabs(decoded.euler[1] - 34.708008f) < 1e-4f);
  assert(std::fabs(decoded.quaternion[0] - 0.9411934f) < 1e-5f);
  const Installation r = {0, -1, 0, 1, 0, 0, 0, 0, 1};
  assert(CheckInstallation(r));
  assert(!CheckInstallation({-1, 0, 0, 0, 1, 0, 0, 0, 1}));
  auto invalid = r;
  invalid[0] = 1;
  assert(!CheckInstallation(invalid));
  Sample sample{{0, 0, 9.80665f}, {.2f, 0, 0}, 100};
  DmFilter rotated(r), identity;
  Attitude a, b;
  Sample expected = sample;
  Rotate(r, sample.accel, expected.accel);
  Rotate(r, sample.gyro, expected.gyro);
  for (unsigned i = 0; i < 1000; ++i)
  {
    assert(rotated.Update(sample, a));
    assert(identity.Update(expected, b));
    assert(std::memcmp(a.quaternion, b.quaternion, sizeof(a.quaternion)) == 0);
  }
  const auto seq = a.sequence;
  sample.accel[0] = NAN;
  assert(!rotated.Update(sample, a));
  assert(a.sequence == seq);
  BoardFilter board_rotated(r), board_identity;
  BoardState ba, bb;
  sample = {{0, 0, 9.80665f}, {.01f, .02f, .03f}, 200};
  expected = sample;
  Rotate(r, sample.accel, expected.accel);
  Rotate(r, sample.gyro, expected.gyro);
  for (unsigned i = 0; i < 1000; ++i)
  {
    assert(board_rotated.Update(sample, ba));
    assert(board_identity.Update(expected, bb));
    assert(std::memcmp(ba.quaternion, bb.quaternion, sizeof(ba.quaternion)) == 0);
    assert(ba.time_us == 200 && ba.sequence == i + 1);
    for (float x : ba.motion_accel_b)
    {
      assert(std::isfinite(x));
    }
  }
  for (unsigned i = 0; i < allocation_count; ++i)
  {
    std::free(allocations[i]);
  }
  puts(
      "PASS: stream splits/CRC/recovery/57B time, per-sample fusion, installation "
      "rotation, rejected NaN");
}
