# 公共LPF统一与算法仓库整理

## 生产代码

ControlAlgorithms根目录保留XRobot入口 `ControlAlgorithms.hpp`、CMake和说明文件，公共头集中到 `include/wdr_control/`：

- `control_pid.hpp`：通用PID。
- `gimbal_pid.hpp`：26_sentry云台PID和角度工具。
- `low_pass_filter.hpp`：统一一阶LPF与二阶Butterworth。

删除 `algorithm/translation_filter.c/h` 及旧C接口，不增加兼容转发文件。Pitch、DualYaw、Launcher按具名公共路径引用头文件。源码目录调整与LPF合并在ControlAlgorithms中分开提交。

## 一阶LPF行为

`LowPassFilter(alpha)` 保持原有固定权重和首样本启动，PID导数调用不变。

底盘持有 `LowPassFilter(1.0f, InitialState::ZERO)`，每拍调用 `Update(input, dt, tau)`；两个权重分别为 `dt/(tau+dt)` 与 `tau/(tau+dt)`，dt和tau单位为秒。构造、Reset及底盘精确零目标后均从零历史递推；不采用通用LPF默认的首拍直通。

底盘仍在自身命令处理阶段执行零目标清历史，滤波器本身不解释速度、来源、RELAX或故障。轮速PID、制动、模式切换与机构许可不随本次整理修改。时间常数模式和固定权重模式共用一份状态及递推逻辑，二阶LPF保留独立实现。

## 回归与构建

冻结C源码继续保存在 `tests/chassis/reference/control_algorithms/algorithm/`，SHA-256不变；生产构建不编译这些参考文件。

`tests/control_algorithms/run_tests.py` 将冻结平移滤波编入主机对照，验证10000拍变dt输入、周期清零、重新起步和tau=0。底盘回归继续覆盖零目标、滤波历史、失联恢复、模式和实际dt；云台、发射、I6x、功率及IMU回归检查公共依赖的影响。

实际构建保持选中 `User/RobotConfig/sentry_remote.yaml`，验证Debug/Release完整整车固件。模块提交、主工程锁文件和本地Git bundle一同更新，另做冻结离线恢复检查。本次不烧录、不进行设备收发或机构闭环验证。

本次结果见[验证记录](validation/lpf_unification_20261107.json)。Launcher远端Module CI在私有CI仓库认证阶段返回HTTP 401，未进入编译；本地主机回归与整车构建通过，二者分别记录。
