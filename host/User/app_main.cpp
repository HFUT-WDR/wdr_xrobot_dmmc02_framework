#include <cstdlib>

#include "linux_uart.hpp"
#include "xrobot_main.hpp"
#ifdef WDR_HOST_ROS
#include "rclcpp/rclcpp.hpp"
#endif

int main(int argc, char** argv)
{
  LibXR::PlatformInit();
#ifdef WDR_HOST_ROS
  rclcpp::init(argc, argv, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
#else
  (void)argc;
  (void)argv;
#endif
  const char* path = std::getenv("WDR_HOST_DEVICE");
  static LibXR::LinuxUART& host_uart =
      path ? *new LibXR::LinuxUART(path, 115200, LibXR::UART::Parity::NO_PARITY, 8, 1, 1,
                                   512, 65536)
           : *new LibXR::LinuxUART(std::string("1d50"), std::string("6199"),
                                   std::string("Host"), 115200,
                                   LibXR::UART::Parity::NO_PARITY, 8, 1, 1, 512, 65536);
  host_uart.SetRealtimeTxLifetime(100);
  XR_REGISTER(host_uart, LibXR::LinuxUART);
  XROBOT_MAIN();
}
