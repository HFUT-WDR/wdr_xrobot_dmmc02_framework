#include <atomic>
#include <cassert>
#include <cstdio>
#include <cstdlib>

#include "HostLink.hpp"
#include "HostRos.hpp"
#include "rclcpp/rclcpp.hpp"
#include "wdr_interfaces/msg/aim_intent.hpp"
#include "wdr_interfaces/msg/timed_imu.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  new HostRos;
  auto node = std::make_shared<rclcpp::Node>("wdr_test_observer");
  std::atomic<bool> received_intent{}, received_imu{};
  auto callback = LibXR::Topic::Callback::Create(
      [](bool, std::atomic<bool>* seen, const wdr_host::AimIntent& input)
      {
        assert(input.aim.mode == 2 && input.aim.id == 3 && input.aim.back_find == 1);
        assert(input.aim.fire_control == 1 && input.steady_time_us != 0);
        seen->store(true);
      },
      &received_intent);
  LibXR::Topic(LibXR::Topic::Find("aim_intent")).RegisterCallback(callback);
  auto publisher =
      node->create_publisher<wdr_interfaces::msg::AimIntent>("/wdr/aim_intent", 1);
  auto subscriber = node->create_subscription<wdr_interfaces::msg::TimedImu>(
      "/wdr/timed_imu", rclcpp::SensorDataQoS(),
      [&](wdr_interfaces::msg::TimedImu::ConstSharedPtr input)
      {
        assert(input->mcu_time_us == 987654);
        assert(input->sync_valid && input->imu.orientation.w == 1);
        assert(input->steady_time_ns != 0 && input->imu.header.stamp.sec > 0);
        received_imu.store(true);
      });
  LibXR::Topic telemetry(LibXR::Topic::Find("aim_state"));
  const auto end = wdr_host::steady_us() + 3000000;
  while ((!received_intent.load() || !received_imu.load()) && wdr_host::steady_us() < end)
  {
    wdr_interfaces::msg::AimIntent intent;
    intent.steady_time_ns = wdr_host::steady_us() * 1000;
    intent.mode = 2;
    intent.id = 3;
    intent.back_find = 1;
    intent.fire_control = true;
    publisher->publish(intent);
    wdr_host::TimedAimState state{};
    state.data.imu_time_us = 987654;
    state.steady_time_ns = intent.steady_time_ns;
    state.sync_valid = true;
    telemetry.Publish(state);
    rclcpp::spin_some(node);
    usleep(20000);
  }
  assert(received_intent.load() && received_imu.load());
  std::puts("PASS: C++ ROS intent -> LibXR, LibXR -> ROS TimedImu with both timestamps");
  std::fflush(stdout);
  std::_Exit(0);
}
