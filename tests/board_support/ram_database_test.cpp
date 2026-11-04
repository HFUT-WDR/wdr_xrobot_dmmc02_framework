#include <array>
#include <cassert>
#include <cstdio>
#include <cstdlib>

#include "imu_ram_database.hpp"

extern "C" void libxr_fatal_error(const char*, uint32_t, bool)
{
  std::abort();
}

int main()
{
  using Offset = std::array<float, 3>;
  using Key = LibXR::Database::Key<Offset>;
  const Offset zero{0, 0, 0};
  const Offset offset{0.01f, -0.02f, 0.03f};
  ImuRamDatabase database;
  Key key(database, "bmi088_gyro_data", zero);
  assert(key.data_ == zero);
  assert(key.Set(offset) == LibXR::ErrorCode::OK);
  key.data_ = zero;
  assert(key.Load() == LibXR::ErrorCode::OK);
  assert(key.data_ == offset);
  Key second_reader(database, "bmi088_gyro_data", zero);
  assert(second_reader.data_ == offset);

  // 新后端模拟重新上电，不能恢复旧零偏。
  ImuRamDatabase rebooted;
  Key after_reset(rebooted, "bmi088_gyro_data", zero);
  assert(after_reset.data_ == zero);

  // 不接受其他键名或错误布局，失败不能覆盖已有零偏。
  Key wrong_name(database, "other", zero);
  assert(wrong_name.Save() == LibXR::ErrorCode::ARG_ERR);
  LibXR::Database::Key<uint32_t> wrong_size(database, "bmi088_gyro_data", 42);
  assert(wrong_size.Save() == LibXR::ErrorCode::ARG_ERR);
  assert(key.Load() == LibXR::ErrorCode::OK);
  assert(key.data_ == offset);
  std::puts("RAM zero-offset lifecycle and invalid-key checks passed");
}
