#pragma once
#include <cstdint>
namespace LibXR
{
using MicrosecondTimestamp = uint64_t;
struct Timebase
{
  static inline uint32_t now = 0;
  static uint64_t GetMicroseconds() { return static_cast<uint64_t>(now) * 1000; }
  static uint32_t GetMilliseconds() { return now; }
};
}
