#pragma once
#include <cstdint>
namespace wdr_imu { struct BoardState { float yaw{},pitch{},roll{};uint64_t time_us{}; uint32_t sequence{}; }; }
