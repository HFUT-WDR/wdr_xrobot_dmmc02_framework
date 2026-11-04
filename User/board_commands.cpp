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
  LibXR::STDIO::Print<"health heap_min={} spi2_err={} spi6_err={} adc3_err={} cfsr={}\r\n">(
      xPortGetMinimumEverFreeHeapSize(), HAL_SPI_GetError(&hspi2),
      HAL_SPI_GetError(&hspi6), HAL_ADC_GetError(&hadc3), SCB->CFSR);
  return 0;
}

int imu_command(void*, int, char**)
{
  using Vector = Eigen::Matrix<float, 3, 1>;
  // 订阅者长期存在，只在命令执行时等待下一次采样；不阻塞采样线程。
  static LibXR::Topic::ASyncSubscriber<Vector> gyro(
      LibXR::Topic(LibXR::Topic::FindOrCreate<Vector>("bmi088_gyro")));
  static LibXR::Topic::ASyncSubscriber<Vector> accel(
      LibXR::Topic(LibXR::Topic::FindOrCreate<Vector>("bmi088_accl")));
  // 上一次超时可能在返回后收到数据，先消费，保证本次等待新数据。
  if (gyro.Available())
  {
    gyro.GetData();
  }
  if (accel.Available())
  {
    accel.GetData();
  }
  gyro.StartWaiting();
  accel.StartWaiting();
  for (unsigned wait_ms = 0; wait_ms < 100; ++wait_ms)
  {
    if (gyro.Available() && accel.Available())
    {
      const Vector g = gyro.GetData();
      const Vector a = accel.GetData();
      const uint32_t age_us =
          (LibXR::Timebase::GetMicroseconds() - gyro.GetTimestamp()).ToMicrosecond();
      LibXR::STDIO::Print<"imu age_us={} gyro_rad_s=[{:.3f},{:.3f},{:.3f}] "
                          "accel_g=[{:.3f},{:.3f},{:.3f}]\r\n">(
          age_us, g.x(), g.y(), g.z(), a.x(), a.y(), a.z());
      return 0;
    }
    LibXR::Thread::Sleep(1);
  }
  LibXR::STDIO::Print<"imu no fresh sample within 100ms\r\n">();
  return -1;
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
  static auto status =
      LibXR::RamFS::CreateFile("status", status_command, static_cast<void*>(nullptr));
  static auto echo =
      LibXR::RamFS::CreateFile("echo", echo_command, static_cast<void*>(nullptr));
  static auto health =
      LibXR::RamFS::CreateFile("health", health_command, static_cast<void*>(nullptr));
  static auto imu =
      LibXR::RamFS::CreateFile("imu", imu_command, static_cast<void*>(nullptr));
  ramfs.Add(health);
  ramfs.Add(imu);
  ramfs.Add(status);
  ramfs.Add(echo);
}
