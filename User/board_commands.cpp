#include "board_commands.hpp"

#include "FreeRTOS.h"
#include "adc.h"
#include "libxr.hpp"
#include "main.h"
#include "message.hpp"
#include "spi.h"
#include "task.h"
#include "transform.hpp"

namespace
{
int status_command(void*, int, char**)
{
  LibXR::STDIO::Print<"WDR MC02 clock_hz={} uptime_ms={} heap_free={} tasks={}\r\n">(
      SystemCoreClock, HAL_GetTick(), xPortGetFreeHeapSize(), uxTaskGetNumberOfTasks());
  return 0;
}

int health_command(void*, int, char**)
{
  LibXR::STDIO::Print<
      "health heap_min={} spi2_err={} spi6_err={} adc3_err={} cfsr={}\r\n">(
      xPortGetMinimumEverFreeHeapSize(), HAL_SPI_GetError(&hspi2),
      HAL_SPI_GetError(&hspi6), HAL_ADC_GetError(&hadc3), SCB->CFSR);
  return 0;
}

int echo_command(void*, int argc, char** argv)
{
  if (argc != 2)
  {
    LibXR::STDIO::Print<"usage: echo token\r\n">();
    return -1;
  }
  LibXR::STDIO::Print<"{}\r\n">(argv[1]);
  return 0;
}
}  // namespace

void register_board_commands(LibXR::RamFS& ramfs)
{
  // 只保留板级观察命令：模块自己的状态与输出由各模块的命令提供。
  static auto status =
      LibXR::RamFS::CreateFile("status", status_command, static_cast<void*>(nullptr));
  static auto health =
      LibXR::RamFS::CreateFile("health", health_command, static_cast<void*>(nullptr));
  static auto echo =
      LibXR::RamFS::CreateFile("echo", echo_command, static_cast<void*>(nullptr));
  ramfs.Add(status);
  ramfs.Add(health);
  ramfs.Add(echo);
}
