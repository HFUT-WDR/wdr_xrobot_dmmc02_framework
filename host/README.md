# WDR 上位机

Linux C++20上位机BSP，采用LibXR与XRobot。HostLink负责USB Topic通信、自瞄与导航独立发送和MCU时间同步；HostRos只负责ROS2与本地Topic的转换。运行时代码全部为C++。

目录遵循[XRobot BSP约定](https://xrobot.work/docs/proj_man)：

- `Modules/modules.yaml`、`sources.yaml`、`xrobot.lock`：模块请求、来源及固定提交。
- `User/app_main.cpp`：创建LibXR LinuxUART、注册硬件、初始化平台。
- `User/*.yaml`：配置模块及其参数；`xrobot_main.hpp`由工具生成，不手改。
- `Modules/HFUT-WDR/HostLink`、`HostRos`：独立模块仓，检出目录不提交到BSP。
- `module_sources/*.bundle`：本地模块快照，允许未推送时恢复和构建。
- `ros2/wdr_interfaces`：ROS消息定义，没有Python通信节点。

本BSP与下位机共用当前检出的HostData协议头，以及父工程固定的LibXR。
`patches/libxr-host-topic.patch`包含精确payload长度、断连清半包，以及LinuxUART实时发送策略；
setup.sh只在补丁可完整应用时安装，不覆盖依赖中的其他改动。
先恢复父工程模块，再进入host；Linux依赖见[官网Linux配置](https://xrobot.work/docs/env_setup/env-setup-linux)。

## 构建与监听

首次在本目录执行（先初始化父工程LibXR子模块；模块来自本地bundle）：

```sh
./tools/setup.sh
xrobot gen -c User/xrobot.yaml
cmake --preset Debug
cmake --build --preset Debug -j4
ctest --test-dir build/Debug --output-on-failure
./build/Debug/wdr_host
```

默认按VID/PID `1d50:6199`及接口名`Host`寻找第二路CDC；也可显式选设备：

```sh
WDR_HOST_DEVICE=/dev/serial/by-id/实际Host接口 ./build/Debug/wdr_host
```

默认仅接收状态与发送对时请求，不发送控制。OnMonitor每秒输出同步状态、a、RTT及残差。
Release使用对应的Release preset。构建结果不代表真实USB收发或机构闭环已经验证。

## ROS2共享数据

先构建类型包，再选择ROS配置（在host目录执行）：

```sh
. /opt/ros/jazzy/setup.sh
colcon build --base-paths ros2 --packages-select wdr_interfaces
. install/setup.sh
xrobot gen -c User/ros.yaml
cmake -S . -B build/ROS -G Ninja -DCMAKE_BUILD_TYPE=Debug -DWDR_HOST_ROS=ON
cmake --build build/ROS -j4
./build/ROS/wdr_host
```

`User/ros.yaml`同样仅监视；实际接受ROS控制意图时选择`User/ros_control.yaml`并重新生成/构建。
上位机不能切换板端OP/AUTO或触发CALI，最终许可仍由遥控及机构决定。

| ROS Topic | 方向 | 用途 |
| --- | --- | --- |
| `/wdr/aim_intent` | 输入 | 视觉目标及导数、mode、开火、back_find、id、装甲数和目标元数据 |
| `/wdr/nav_intent` | 输入 | vx/vy、nav_state、navi_task、sentry_state、big_yaw_angle |
| `/wdr/aim_state` | 输出 | 自瞄遥测，不含mode |
| `/wdr/nav_state` | 输出 | 导航遥测，含big_yaw_angle，不含mode |
| `/wdr/timed_imu` | 输出 | 同一姿态样本的ROS、steady、MCU时间 |
| `/wdr/imu` | 输出 | 从同一TimedImu派生的标准sensor_msgs/Imu |

直接开发LibXR模块时使用本地`aim_intent`/`nav_intent`和`aim_state`/`nav_state`，无需经过ROS。
输入生产者填写`std::chrono::steady_clock`微秒时间。ROS输入对应字段为纳秒，桥接层换算；
`target_steady_ns`是目标预测参考时刻，不是相机曝光时间。字段为0时使用本帧生产时刻。

视觉与导航分别以生产时刻和真实接收时刻判定100ms时效，分别发送host_aim/host_nav，互不续期。
无目标但视觉健康时生产者发送mode=1；视觉进程断流则停止发送aim，板端超时后停止并撤销开火。
mode仅0停止、1扫描、2自瞄。导航不控制wz。旧3/4/5无效。

## 时间与状态约定

MCU与steady映射为`MCU_us = a * host_steady_us + b`，先以8个四时间戳样本估计offset，
再在至少30s跨度的低RTT样本上估计漂移。当前锚点连续，offset残差逐步修正。
没有硬件测量前不宣称具体微秒精度；链路不对称产生的偏置仍存在。

同步未就绪或超过3s未成功对时，停止发送aim。断开或MCU时间回退时重建映射；
输入缓存一并撤销。LinuxUART使用单个待发送请求和100ms实时期限，断开/超时失败，不跨连接重放。
普通LinuxUART默认行为保持不变，这一策略仅由本BSP显式启用。

DM姿态时间目前是软件消费完整帧的时刻，不能冒称传感器硬件采样时间。
ROS时间跳变只重建ROS锚点；硬件运行不支持use_sim_time。

## 当前边界

back_find、id、armors_num、enemy_center_yaw和导航任务字段都保留在线协议中，
不额外缓存或发布metadata；其中尚无消费者的旧战术不因字段存在就自动执行。
当前生效的是0/1/2、云台目标与导数、开火电平和导航vx/vy；大Yaw仍由本地双Yaw协调；distance参与下位机自动旋转策略。

sp_vision源码本轮未改；其CBoard接入应订阅TimedImu、按steady时间插值，并发布AimIntent，
不能继续让旧CAN接收时间覆盖这里的采样时间。相机时间域和原固定-1ms补偿仍需实测核对。
