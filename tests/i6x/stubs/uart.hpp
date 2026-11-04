#pragma once
#include <cstddef>
#include "semaphore.hpp"
namespace LibXR
{
enum class ErrorCode { OK, TIMEOUT };
struct UART
{
  enum class Parity { EVEN };
  struct Config { int baudrate; Parity parity; int bits; int stops; };
  struct Data { void* pointer; size_t size; };
  void SetConfig(Config) {}
  ErrorCode Read(Data, ReadOperation&) { return ErrorCode::TIMEOUT; }
};
}
