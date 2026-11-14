# IMU模块接入（2026-11-05）

> **2026-11-14 起板载与外置解算合并为 `AttitudeEkf`（一个仓库一种算法），`BoardAttitude`、`DmAttitude`、`AttitudeFusion`、`ImuEkfFusion` 均已删除。**
>
> 以下保留首版接入记录。外置DM的当前协议、功能与配置以[模块补全记录](dm_imu_complete_20261105.md)为准；首版“仅RS485旧连续流”不再代表当前实现。

## 范围

按已确认的“功能参考26_sentry、形式参考QDU”接入板载BMI088和一颗外置DM；第二颗DM不实例化。
底盘、云台、机械校准和功率控制尚未接入。本轮不烧录、不启动电机。

| 模块 | 职责 | 默认实例 |
| --- | --- | --- |
| HFUT-WDR/BMI088 | SPI采样、传感器零偏；原始accel(g)/gyro(rad/s) Topic | 已有 |
| YunMo-yanyu/ImuFusion | 公共算法、强类型数据、DM协议与流组帧；无线程 | 依赖 |
| YunMo-yanyu/BoardAttitude | 同一采样时间戳配对，g→m/s²，安装变换、板载EKF和在线Z零偏 | 默认启动 |
| YunMo-yanyu/DmImu | RS485连续流解析、DM专用EKF、发布姿态 | 仅独立配置 |

全部为标准XRobot模块依赖，通过ModuleIndex与xrobot.lock锁定；源码在个人私有仓库维护。
ImuFusion继承了MC02_Framework_CPP已做过冻结源对照的移植。必要源差异：逆平方根使用uint32_t/memcpy，
卡方测试转置残差向量，DM滤波改为实例持有上下文，固定存储复位无堆分配；板载算法初始化才分配。
参考与移植中的算法参数、首拍、Z轴在线偏置、EKF自适应和运动加速度保留，不用框架算法替换。
板载EKF静止检测9.794与运动加速度9.80665分别保留。公共CMSIS-DSP宏PI/INDEX_MASK不泄漏到LibXR。

## 数据契约

- 板载实例输出 `wdr_imu::Attitude`，默认Topic `board_attitude`。
- DmImu输出 `wdr_imu::DmMeasurement`（默认Topic `dm_measurement`），姿态由 AttitudeEkf 实例订阅后发布 `dm_attitude`。
- yaw/pitch/roll单位rad，gyro三轴rad/s，accel三轴m/s²（含重力），四元数顺序w,x,y,z。
- BoardState另带yaw_total与原算法定义的机体/导航系运动加速度。
- time_us与sequence同姿态一起发布；Topic新消息只代表新计算结果，消费者仍需检查时效与机构条件。
- 板载时间来自BMI088硬件采样通知时间戳。DM时间来自任务消费完整流帧的时刻，不是设备采样/硬件收包时刻。
- 仅完整合法新数据推进融合；不重复融合/重发旧快照。无数据时不伪造零姿态。
- 保留原1kHz固定融合步长，不把任务排队后的消费间隔当作真实采样间隔。设备必须配置1kHz，未验证其他速率。

## 安装矩阵

参数installation为行优先3×3：`v_body = R_body_sensor * v_sensor`，默认单位阵。
在融合之前同时旋转accel/gyro，姿态也由该机构坐标的数据解算，避免只旋转一个向量。
拒绝非有限、非正交或左手矩阵；错误配置停止初始化，不静默退回单位阵。
当前BMI088原有rotation仍为单位四元数，调整时避免在两个模块重复施加同一旋转。

```yaml
- module: YunMo-yanyu/BoardAttitude
  id: board_attitude
  args:
  - gyro_topic_name: "bmi088_gyro"
  - accel_topic_name: "bmi088_accl"
  - output_topic_name: "board_attitude"
  - installation: '{1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f}'
```

## 接收与任务

BoardAttitude依赖当前BMI088同线程、同时间戳的accel→gyro发布顺序。
回调配对后复制入16样本LibXR SPSC队列，满时丢新并计数；不同时间戳不配对，重复采样时间戳不重复融合，GetUnpaired/GetDrops可读。
独立High线程每个入队样本融合一次，4096字节栈。没有在ISR或Topic回调里做EKF。

DmImu首版运行路径：USART2，4Mbps、8N1、硬件DE、设备ID默认1，57/80字节CRC连续输出。
通过LibXR UART字节队列在High线程解码、融合、发布，4096字节栈。USART2接收缓冲从128扩至2048，
保持AXI SRAM与32字节对齐，libxr_config.yaml与app_main.cpp同步。不存在旧HAL全局接收回调冲突。
57字节帧等下一子包头确认，保留第57字节的时间；坏80字节帧不能降格成57字节发布。
没有自动猜帧长的超时，也不为坏CRC或半帧刷新姿态时间。

DM启动不发送模式切换或写Flash命令。需先用原工具把实物配置到1kHz连续输出。
CAN/主动请求编解码接口在公共依赖保留，但首版DmImu线程只支持RS485连续输出，不声称其余链路运行通过。
模块声明的time_us只能衡量消费/发布后的年龄；UART队列内积压可能使真实样本更旧，不能用它替代控制截止时间验收。

## 生成与验证

默认选择仍为User/xrobot.yaml；可选User/xrobot_i6x.yaml也带板载姿态。
User/xrobot_imu.yaml只实例化一颗DM，启用需执行 `xrobot gen -c User/xrobot_imu.yaml`，恢复默认使用 `xrobot gen -c User/xrobot.yaml`。
普通Debug/Release编译检查12个构造接口及三个完整配置，DmImu检查对象不链接进默认固件。
主机测试 `python3 tests/imu/run_tests.py`：板载5000组、DM8100组冻结源对照，独立状态/复位、CRC、任意分包、
57字节完成时间、坏帧恢复、板载与DM安装矩阵对照、NaN拒收。冻结源散列固定，必要修复仅对临时参考副本适配。
主机矩阵使用测试替身，ARM使用基座CMSIS-DSP；主机数值对照不等价于真实DSP/RTOS性能验收。
另执行既有I6x、板级存储与默认ELF隔离回归，完整结果见validation/imu_modules_20261105.json。

姿态命令由 `AttitudeEkf` 实例在构造时按 `command_name` 注册，只等待一份新数据后输出，不持续刷屏。
原板级 `attitude`/`dm_attitude`/`imu` 命令已删除，板级 `board_commands` 只保留 `status/health/echo`；
采样观察改由模块自带的 `bmi088` 命令提供（见 [AGENTS.md](../AGENTS.md) 的模块职责边界）。
本次未烧录，新模块启动、线程栈余量、4Mbps连续接收、姿态实际轴向与温控/零偏标定均未板测。
当前BMI088零偏仍使用RAM数据库，持久校准与恢复在后续校准阶段接入，不宣称已完成。

> 2026-11-07 更新：BMI088 零偏已改由内部 Flash 数据库保存，见[Flash 数据库与标定持久化](flash_database_20261107.md)；本文其余板测结论对应 2026-11-05 的 RAM 配置。

> 2026-11-06 清理更新：本文中的多份独立 YAML 为当时验证入口，现已删除。当前机器人入口统一为 `User/xrobot.yaml`；底盘、DM IMU 增量参数位于 `config/templates/chassis.yaml`、`config/templates/dm_imu.yaml`，由 `tools/generate_external_checks.py` 合并并生成回归组合，不直接运行旧生成命令。历史验证结果保留原样。
