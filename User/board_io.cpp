#include "board_io.hpp"

#include <array>
#include <cstdint>
#include <cstdlib>
#include <cstring>

#include "libxr.hpp"

namespace
{
constexpr uint32_t HEARTBEAT_MODE = 0x01000000;
constexpr uint32_t POLL_MS = 10;
constexpr uint32_t DEBOUNCE_SAMPLES = 3;
}  // namespace

BoardIO::BoardIO(LibXR::GPIO& key, LibXR::SPI& led_spi, LibXR::RamFS& ramfs)
    : key_command_(LibXR::RamFS::CreateFile("key", KeyCommand, this)),
      led_command_(LibXR::RamFS::CreateFile("led", LedCommand, this)),
      key_(key), led_spi_(led_spi), spi_op_(spi_done_)
{
  ramfs.Add(key_command_);
  ramfs.Add(led_command_);
  thread_.Create(this, ThreadFunc, "board_io", 1536, LibXR::Thread::Priority::LOW);
}

void BoardIO::WriteLed(uint32_t rgb)
{
  // HSE 24 MHz / 4 = 6 MHz。沿用达妙编码：GRB，每位 8 个 SPI bit。
  // 末尾 64 个零字节提供约 85 us 复位低电平。
  std::array<uint8_t, 88> frame{};
  const uint8_t colors[] = {static_cast<uint8_t>(rgb >> 8),
                            static_cast<uint8_t>(rgb >> 16),
                            static_cast<uint8_t>(rgb)};
  for (size_t color = 0; color < 3; ++color)
  {
    for (size_t bit = 0; bit < 8; ++bit)
    {
      frame[color * 8 + bit] = (colors[color] & (0x80u >> bit)) ? 0x78 : 0x60;
    }
  }
  // 当前 SPI6 只有 TX DMA，LibXR 按轮询发送；frame 生命周期覆盖整次传输。
  if (led_spi_.Write(LibXR::ConstRawData(frame), spi_op_) != LibXR::ErrorCode::OK)
  {
    led_errors_.fetch_add(1);
  }
}

void BoardIO::ThreadFunc(BoardIO* board)
{
  bool candidate = false;
  uint32_t stable_samples = 0;
  uint32_t previous_rgb = UINT32_MAX;
  LibXR::MillisecondTimestamp wakeup = LibXR::Timebase::GetMilliseconds();
  while (true)
  {
    const bool pressed = !board->key_.Read();
    if (pressed != candidate)
    {
      candidate = pressed;
      stable_samples = 1;
    }
    else if (stable_samples < DEBOUNCE_SAMPLES)
    {
      ++stable_samples;
    }
    if (stable_samples == DEBOUNCE_SAMPLES &&
        candidate != board->key_pressed_.load())
    {
      board->key_pressed_.store(candidate);
      if (candidate)
      {
        board->key_presses_.fetch_add(1);
      }
    }

    const uint32_t request = board->led_request_.load();
    const uint32_t now_ms = LibXR::Timebase::GetMilliseconds();
    const uint32_t rgb = (request & HEARTBEAT_MODE)
                             ? ((now_ms / 500) % 2 ? 0 : 0x000800)
                             : request;
    if (rgb != previous_rgb)
    {
      board->WriteLed(rgb);
      previous_rgb = rgb;
    }
    LibXR::Thread::SleepUntil(wakeup, POLL_MS);
  }
}

int BoardIO::KeyCommand(BoardIO* board, int, char**)
{
  LibXR::STDIO::Print<"key pressed={} presses={} raw={}\r\n">(
      static_cast<unsigned>(board->key_pressed_.load()), board->key_presses_.load(),
      static_cast<unsigned>(board->key_.Read()));
  return 0;
}

int BoardIO::LedCommand(BoardIO* board, int argc, char** argv)
{
  if (argc == 2 && std::strcmp(argv[1], "auto") == 0)
  {
    board->led_request_.store(HEARTBEAT_MODE);
  }
  else if (argc == 4)
  {
    uint32_t rgb = 0;
    for (int i = 1; i < 4; ++i)
    {
      char* end = nullptr;
      const int64_t value = static_cast<int64_t>(std::strtol(argv[i], &end, 10));
      if (*argv[i] == '\0' || *end != '\0' || value < 0 || value > 255)
      {
        LibXR::STDIO::Print<"RGB must be integers 0..255\r\n">();
        return -1;
      }
      rgb = (rgb << 8) | static_cast<uint32_t>(value);
    }
    board->led_request_.store(rgb);
  }
  else if (argc != 1)
  {
    LibXR::STDIO::Print<"usage: led [auto|r g b]\r\n">();
    return -1;
  }
  const uint32_t request = board->led_request_.load();
  LibXR::STDIO::Print<"led auto={} r={} g={} b={} errors={}\r\n">(
      static_cast<unsigned>((request & HEARTBEAT_MODE) != 0),
      (request >> 16) & 255u, (request >> 8) & 255u, request & 255u,
      board->led_errors_.load());
  return 0;
}
