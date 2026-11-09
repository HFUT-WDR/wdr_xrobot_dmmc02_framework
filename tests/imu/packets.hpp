#pragma once
#include <array>
#include <cstdint>
#include <cstring>
static constexpr std::array<uint16_t, 256> CRC16_TABLE = [] {
  std::array<uint16_t, 256> table{};
  for (size_t i = 0; i < table.size(); ++i)
  {
    uint16_t value = uint16_t(i << 8);
    for (unsigned bit = 0; bit < 8; ++bit)
      value = (value & 0x8000) ? uint16_t((value << 1) ^ 0x1021)
                               : uint16_t(value << 1);
    table[i] = value;
  }
  return table;
}();
static void putfloat(uint8_t* p, float f)
{
  uint32_t u;
  std::memcpy(&u, &f, 4);
  for (unsigned i = 0; i < 4; ++i) p[i] = uint8_t(u >> (i * 8));
}
static void packet(uint8_t* p, unsigned reg, const float* v, unsigned n)
{
  p[0] = 0x55;
  p[1] = 0xaa;
  p[2] = 1;
  p[3] = uint8_t(reg);
  for (unsigned i = 0; i < n; ++i) putfloat(p + 4 + 4 * i, v[i]);
  unsigned len = 4 + 4 * n;
  uint16_t crc = 0xffff;
  for (unsigned i = 0; i < len; ++i)
  {
    const uint8_t index = uint8_t((crc >> 8) ^ p[i]);
    crc = uint16_t((crc << 1) ^ CRC16_TABLE[index]);
  }
  p[len] = uint8_t(crc);
  p[len + 1] = uint8_t(crc >> 8);
  p[len + 2] = 0x0a;
}
static void frame(uint8_t* p, float gz)
{
  float a[] = {0, 0, 9.80665f}, g[] = {0, 0, gz}, e[] = {10, 20, 30}, q[] = {1, 0, 0, 0};
  packet(p, 1, a, 3);
  packet(p + 19, 2, g, 3);
  packet(p + 38, 3, e, 3);
  packet(p + 57, 4, q, 4);
}
