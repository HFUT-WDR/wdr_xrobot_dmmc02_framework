#pragma once

#include <atomic>

#include "gpio.hpp"
#include "ramfs.hpp"
#include "spi.hpp"
#include "thread.hpp"

// PA15 用户按键和 SPI6 WS2812；仅 board_io 线程操作 SPI6。
class BoardIO
{
 public:
  BoardIO(LibXR::GPIO& key, LibXR::SPI& led_spi, LibXR::RamFS& ramfs);

 private:
  static void ThreadFunc(BoardIO* board);
  static int KeyCommand(BoardIO* board, int argc, char** argv);
  static int LedCommand(BoardIO* board, int argc, char** argv);
  void WriteLed(uint32_t rgb);

  // RamFS 持有节点引用，必须与 BoardIO 同寿命。
  LibXR::RamFS::File key_command_;
  LibXR::RamFS::File led_command_;
  LibXR::GPIO& key_;
  LibXR::SPI& led_spi_;
  LibXR::Thread thread_;
  LibXR::Semaphore spi_done_;
  LibXR::SPI::OperationRW spi_op_;
  // bit24 表示心跳模式，低24位为 RGB；单个原子值保证模式与颜色同时切换。
  std::atomic<uint32_t> led_request_{0x01000000};
  std::atomic<uint32_t> led_errors_{0};
  std::atomic<uint32_t> key_presses_{0};
  std::atomic<bool> key_pressed_{false};
};
