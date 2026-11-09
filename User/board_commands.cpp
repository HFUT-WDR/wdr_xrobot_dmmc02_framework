#include "board_commands.hpp"

#include "FreeRTOS.h"
#include "ImuFusion.hpp"
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
      LibXR::STDIO::Print<
          "imu age_us={} gyro_rad_s=[{:.3f},{:.3f},{:.3f}] "
          "accel_g=[{:.3f},{:.3f},{:.3f}]\r\n">(age_us, g.x(), g.y(), g.z(), a.x(), a.y(),
                                                a.z());
      return 0;
    }
    LibXR::Thread::Sleep(1);
  }
  LibXR::STDIO::Print<"imu no fresh sample within 100ms\r\n">();
  return -1;
}

// 只等待一份新姿态，不周期打印，不用旧缓存冒充在线。
template <typename State>
int attitude_command(void* context, int, char**)
{
  const auto* name = static_cast<const char*>(context);
  static LibXR::Topic::ASyncSubscriber<State> subscriber(
      LibXR::Topic(LibXR::Topic::FindOrCreate<State>(name)));
  if (subscriber.Available())
  {
    subscriber.GetData();
  }
  subscriber.StartWaiting();
  for (unsigned i = 0; i < 100; ++i)
  {
    if (subscriber.Available())
    {
      const State state = subscriber.GetData();
      const uint64_t age =
          static_cast<uint64_t>(LibXR::Timebase::GetMicroseconds()) - state.time_us;
      LibXR::STDIO::Print<
          "{} seq={} age_us={} yaw/pitch/roll_rad=[{:.3f},{:.3f},{:.3f}]\r\n">(
          name, state.sequence, static_cast<uint32_t>(age), state.yaw, state.pitch,
          state.roll);
      return 0;
    }
    LibXR::Thread::Sleep(1);
  }
  LibXR::STDIO::Print<"{} no fresh sample within 100ms\r\n">(name);
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
  static char board_topic[] = "board_attitude";
  static char dm_topic[] = "dm_attitude";
  static auto attitude = LibXR::RamFS::CreateFile(
      "attitude", attitude_command<wdr_imu::BoardState>, static_cast<void*>(board_topic));
  static auto dm_attitude = LibXR::RamFS::CreateFile(
      "dm_attitude", attitude_command<wdr_imu::Attitude>, static_cast<void*>(dm_topic));
  ramfs.Add(attitude);
  ramfs.Add(dm_attitude);
  ramfs.Add(health);
  ramfs.Add(imu);
  ramfs.Add(status);
  ramfs.Add(echo);
}
