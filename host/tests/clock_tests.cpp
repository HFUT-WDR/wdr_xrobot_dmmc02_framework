#include <cassert>
#include <cmath>

#include "clock_sync.hpp"

int main()
{
  wdr_host::ClockSync clock;
  assert(!clock.Response({99, 7, 1, 2, 3}, 4));
  uint64_t host = 1000000000;
  for (uint32_t seq = 0; seq < 70; ++seq)
  {
    host = 1000000000ULL + seq * 1000000ULL;
    const auto before = clock.ToMcu(host + 550);
    const bool ready = clock.Valid(host);
    clock.Request(seq, host);
    clock.Response(
        {seq, 7, host, static_cast<uint64_t>((host + 500) * 1.00002 - 500000000),
         static_cast<uint64_t>((host + 600) * 1.00002 - 500000000)},
        host + 1100);
    if (ready) assert(std::fabs(clock.ToMcu(host + 550) - before) < 1);
  }
  assert(clock.Valid(host + 1100));
  assert(!clock.Valid(host + 4000000));
  assert(std::fabs(clock.Slope() - 1.00002) < 0.00002);
  assert(std::fabs(clock.ToHost(clock.ToMcu(host)) - host) < 1);
  host += 1000000;
  clock.Request(100, host);
  clock.Response({100, 8, host, 1000, 1100}, host + 1100);
  assert(!clock.Valid(host + 1100) && clock.BootId() == 8);
}
