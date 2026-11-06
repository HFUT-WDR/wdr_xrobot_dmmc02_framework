# DM IMU 模块补全（2026-11-05）

## 范围与资料

本记录更新首版[IMU接入记录](imu_modules_20261105.md)的外置DM部分。板载BMI088和BoardAttitude保持原实现。
根据用户指定的[厂商资料](https://gitee.com/kit-miao/dm-imu)，核对V1.3（2026-09-22）说明书及MC02 CAN例程，
资料仓库提交为 `ba758fd6106c2b210713bf1ef05800f1688f73c5`。
用户明确指出CAN主动上报模式已取消，本模块只实现CAN请求/应答，不提供主动上报或模式切换选项；
资料中残留的主动上报描述不作为实现要求。厂商未公开的配置、校准和升级控制协议不根据旧代码猜测。

当前RS485为24字节A5/5A请求应答，默认2 Mbps、8N1；57/80字节CRC连续流保留为显式旧固件兼容配置，
不再把旧连续流当作现版RS485协议。模块不会启动时写设备Flash、恢复出厂或发送配置命令。
设备地址、波特率和校准由配套工具管理，模块参数必须与实物匹配。

## 功能与配置

| 配置 | 传输 | 默认绑定 | 输出 |
| --- | --- | --- | --- |
| `User/xrobot.yaml` | 不实例化外置DM | 板载BMI088 | 板载姿态 |
| `User/xrobot_imu.yaml` | RS485_REQUEST | USART2，2 Mbps | FUSED |
| `User/xrobot_imu_can.yaml` | CAN_REQUEST | CAN1，slave=1，master=0x11 | FUSED |
| `User/xrobot_imu_rs485_stream.yaml` | RS485_STREAM_LEGACY | USART2，4 Mbps、1 kHz | FUSED |

这三份外置配置互斥选用，每份仅一颗DM；第二颗不实例化。默认生成配置仍为板载配置。
CAN1为配置示例，接线、总线速率及ID冲突需按实物核对；模块不擅自重设共享CAN总线速率。
启用例：`xrobot gen -c User/xrobot_imu_can.yaml`，再使用既有Debug/Release构建。
恢复默认：`xrobot gen -c User/xrobot.yaml`。VS Code编译和烧录路径不变，使用当前选中的生成配置。

参数在 `DmImu::Param` / `wdr_imu::DmConfig`：

- `transport`选择上述链路；`can`、`uart`恰好传入对应的一项指针。
- `device_id`为设备接收ID，默认1；`feedback_id`为CAN应答ID，默认0x11。CAN使用标准帧，串口ID为8位。
- `baudrate`为串口波特率，默认2000000。硬件DE沿用基座USART2配置。
- `period_ms`为完整采集周期下限，默认10；`timeout_ms`单次请求超时，默认50；`retries`默认2，最多3。
- `data_mask`低四位分别为加速度、角速度、Euler、四元数，默认全部。当前姿态模块必须读取前两组；DEVICE另外至少选择一种姿态组。
- `output=FUSED`运行原DM融合算法；`DEVICE`输出设备自身姿态，优先使用本周期选中的四元数，否则使用Euler。
- `installation`为行优先3×3 `v_body=R_body_sensor*v_sensor`，默认单位阵；同时旋转向量和姿态，拒绝非旋转矩阵。

CAN请求为8字节 `CC RID 00 DD 00 00 00 00`，RID=1..4；RS485请求及应答均24字节，RID=0..3。
每颗设备最多一笔在途请求，按四组顺序读取；验证ID、类型/长度、帧结构、操作/应答和数值后才接受。
无在途请求、错组、超时或非法应答均不更新采集周期。合法全零串口数据仍可接受，不能把它误判为请求回显。
协议没有事务序号，重试后的同组迟到应答不能在协议层完全区分；共享总线负载及实际响应时间须板测。

## 输出与时间

- `dm_attitude`：`wdr_imu::Attitude`，rad、rad/s、m/s²，wxyz四元数，时间和序号；原终端 `dm_attitude` 继续适用。
- `dm_measurement`：`wdr_imu::DmMeasurement`，传感器坐标系原始向量、Euler（度）、四元数，CAN温度及其存在标志、数据掩码和每组接收时间。
- `dm_diagnostics`：`wdr_imu::DmDiagnostics`，请求/应答、超时/重试、失败周期、发布次数、收发错误、队列丢新、TX卡住计数及最后应答/输出时间。

请求模式仅完整新周期发布原始数据与姿态，不周期重发旧结果；各组不代表同时采样。
CAN使用回调收帧时间，UART为任务消费字节时间，两者都不是设备内部采样时钟。
请求融合使用相邻新gyro组的接收间隔；首次使用配置周期，失败周期复位融合历史。
旧连续流保持原1 kHz固定步长与57/80字节CRC组帧；57字节帧没有本帧四元数时不把旧四元数标为新数据。
消费者仍须检查时间和机构运行条件，Topic存在不能作为在线或控制就绪的依据。

CAN回调仅复制帧与时间到16槽SlotPool并通知，解析/融合/发布均在High优先级4096字节线程执行。
UART异步RX/TX使用持久缓冲，操作结束前不复用；TX卡住报告诊断，不覆盖在途缓冲强行重发。
USART2底层2048字节AXI SRAM缓冲和硬件DE保持不变。

## 验证边界

主机回归包含冻结算法13100组对照、CAN四组/位域/地址/请求字节、串口24字节任意分包、零数据、坏帧恢复、
截止边界与有限重试、安装变换、旧流CRC/57字节时间，以及实际DmImu头文件配合LibXR替身的收发测试。
替身保留异步缓冲借用，覆盖回调不发布、队列满丢新和TX未结束不覆盖；不声称替身证明真实RTOS时序。
Debug/Release还检查12个模块构造和五份完整YAML配置，检查对象不进入默认ELF；外置三种配置另做完整固件链接。
具体锁定提交及运行结果见[验证记录](validation/dm_imu_complete_20261105.json)。

本轮不烧录。CAN实收发、现版485设备响应、DE转向、共享总线带宽、栈余量、实时截止时间、姿态轴向及动态精度均未板测。
代码/编译完成不能替代上述硬件验收，也不表示后续底盘、云台、机械校准或功率控制已接入。

2026-11-06：配置文件由xrobot_imu_legacy.yaml更名为xrobot_imu_rs485_stream.yaml，仅明确RS485连续流用途；RS485_STREAM_LEGACY协议枚举、4Mbps和1ms参数保持不变。历史验证记录中的旧产品名不回写。
