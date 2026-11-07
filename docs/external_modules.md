# 基础设备与 i6x 模块

主工程使用 YunMo-yanyu 名下的私有组件仓库。CMD、Motor、RMMotor、DMMotor、DR16、Referee、
HostData、I6x、EventBinder 均通过 XRobot 管理；组件由 WDR 独立维护。LibXR、SharedTopic、
SharedTopicClient 继续使用官方来源；BMI088、BuzzerAlarm 于 2026-11-07 迁移到 HFUT-WDR
自维护，见 [xrobot-org 模块迁移](module_migration_20261107.md)。

模块索引由 `ModuleIndex` Git 子模块提供，依赖请求在 `Modules/modules.yaml`，实际提交固定在
`xrobot.lock`。需要对主工程、索引及各组件都有读取权限的 GitHub 账号。

## i6x 的 YAML 用法

`User/xrobot_i6x.yaml` 是包含已验收板级功能、CMD、I6x 和 EventBinder 的独立配置。其外接输入为：

```yaml
- module: YunMo-yanyu/I6x
  id: i6x
  args:
  - uart: uart5
  - cmd: cmd
  - topic_name: "i6x_input"
```

前面必须先创建 CMD。默认选中仍为 `User/xrobot.yaml`，其中没有 CMD 或 I6x，不会启动外接接收。
确认接线后才显式执行 `xrobot gen -c User/xrobot_i6x.yaml` 并构建、烧录。
恢复板级配置使用 `xrobot gen -c User/xrobot.yaml`。本轮不烧录。

```sh
xrobot module show YunMo-yanyu/I6x
xrobot instance -c User/<配置>.yaml add YunMo-yanyu/I6x --id i6x
xrobot instance -c User/<配置>.yaml set i6x args.uart uart5
xrobot instance -c User/<配置>.yaml set i6x args.cmd cmd
```

无需再手写 I6x 构造或给 app_main 增加专门的 i6x 启动代码。CMD 的独立 i6x 输入槽与通用分机构来源选择已
提交到个人 CMD 仓库，不再由构建脚本打补丁。旧补丁仅在 `docs/archive` 留存迁移记录。

## 接口与语义

MC02 原理图第5页：P6第1脚信号经Q1反相进入PD2/UART5_RX，第2脚5V，第3脚GND。
使用专用P6接口时不再启用软件RX反相。原厂使用手册表格的PC02与原理图不符，采用原理图与
实机26_sentry一致的PD2。接收机必须输出SBUS，不能拿iBUS/PWM代替。

I6x模块复用LibXR UART的DMA字节队列，在任务内组25字节SBUS帧，配置100000、8E2。
HAL侧8数据位加偶校验应为9B字长；锁定版LibXR驱动会正确转换。

通道与拨杆沿用26_sentry：六通道映射到±660，交换左摇杆通道，四拨杆保持+1/0/-1。
+1对应实际下拨、-1对应实际上拨。

| 输入 | 意图 |
| --- | --- |
| s[2] | -1云台无力、0手动、+1上位机 |
| s[3] | -1底盘无力；+1时按s[2]选择手动／上位机 |
| s[0] | +1开火意图 |
| s[1]进入+1 | 发出 SW1_UP 边沿，后续由发射模块处理摩擦轮切换 |
| s[1]=-1且s[3]=+1 | 上位机发射许可 |
| ch[4] | 小陀螺旋钮默认映射到[0,12] rad/s，量程可配置 |

`i6x_input` 只发布 `I6xInput::State` 原始输入快照，供诊断使用；运动量经 CMD 发布，
拨杆位置和组合边沿经 EventBinder 绑定。CMD 不再包含 I6xPolicy 或摩擦轮计数。

### 连续命令与事件分工

- 连续量：I6x → CMD → chassis_cmd / gimbal_cmd / launcher_cmd。
- 来源选择：I6x.GetEvent → EventBinder（YAML）→ CMD.SourceEvent，三个机构独立选择 RC/PC。
- 运行模式：后续机构模块自己的 GetEvent / SetMode 持有 RELAX、FOLLOW、ROTOR 等模式。
  坐标变换归机构模式；本轮没有添加坐标系字段，也没有启动机构任务。

可选 YAML 已绑定云台 s[2]、底盘 s[2]/s[3]组合、发射 s[1]/s[3]组合的来源选择。
UP/MID/DOWN 沿用逻辑值 +1/0/-1，因此 UP 对应物理下拨。启动或离线恢复首帧只建立事件基准；
默认来源为 RC，必须出现后续边沿才切换。保持拨杆和无效帧不会重复触发事件。
CMD 的 START/LOST 继续恢复 RC 来源；未来机构需将这两个事件绑定到 RELAX。

单个失控／丢帧标志帧沿用解码器清摇杆处理，不额外撤销 CMD 开火或 PC 命令，也不刷新有效接收时间；连续3帧失控或严格超过30ms无
有效帧时离线。CMD 仅按 input_online 判断遥控链路；不新增单帧 failsafe 有效位。PC 三路各自超过100ms归零，不回退到遥控运动目标。
按机构选择 PC 发射来源代表操作许可；旧全局 AUTO 路径仍保留 RC 开火与 PC 开火同时有效的约束。

原26_sentry还有20ms上线稳定窗口，本模块尚未迁移，不能宣称完整实车行为等价。
**零运动目标不等于机构无力。** 当前只完成输入和来源链路，实际机构仍须绑定无力、跟随、
小陀螺及摩擦轮事件并执行相应输出。原参考发射模块没有摩擦轮 toggle 事件；未来 WDR 发射模块
应自己持有开关状态，不能恢复接收端切换计数。PID、到速／热量许可、校准和 PASSING_MODE 未接入。
EventBinder 沿用参考实现，找不到名称会忽略绑定；修改 YAML 后必须做生成和编译验证。

## 其他基础模块

`config/templates/external_devices.yaml.in` 提供待填写配置。电机型号、方向、ID和接线没有
确认的实例保留null，不加入默认启动。DR16和I6x通常只启用其中一个，接收UART不能共用。
裁判模板的`cmd: nullptr`表示不将图传键鼠混入遥控源。

- RMMotor构造只登记CAN反馈；同组所有已登记电机提交一轮命令后才发帧，Relax不代表立即发帧。
  MODE_CURRENT复用MotorCmd.velocity表示归一化电流，不是rad/s；没有单独的6020电压模式配置。
- DMMotor调用Enable才发使能帧；反馈ID为0x10+can_id。上游过温分支先失能后仍发控制帧的
  行为尚未改写，不可视为完整机构停机保护。
- Referee为源版本所述的2025协议，未来赛季协议需核对。其接收解析有CRC与长度检查。
- HostData只创建接收Topic并汇总数据；SharedTopic接收、SharedTopicClient发送。
  可绑定USB第二路`usb_otg_hs_cdc2`，日志继续使用第一路。协议为LibXR Topic包格式，非任意裸结构体。

## 生成、编译和测试

```sh
xrobot setup --frozen --offline
cmake --preset Debug
cmake --build --preset Debug -j 4
cmake --preset Release
cmake --build --preset Release -j 4
python3 tests/i6x/run_tests.py
python3 tests/external_modules/run_tests.py --preset Debug
python3 tests/external_modules/run_tests.py --preset Release
```

普通构建自动通过官方`xrobot check-module`编译12个具体模块／依赖的构造接口；还在临时工程
分别生成、编译基础设备组合配置、完整i6x YAML和单颗DM YAML，不替换主工程的已选配置。
所有检查对象不链接进默认固件。主机测试对照本机26_sentry冻结解码源，并覆盖失控、超时和CMD
路由；这不替代真实接收机、电机或机构硬件验收。

另已在独立检出中选择 `User/xrobot_i6x.yaml` 并链接出完整ARM固件，未改变默认工作区配置，未烧录。

## 输入在线与 PC 时效

`CMD::Data::input_online` 仅表达遥控输入链路状态，不表达机构或电机健康。
旧 chassis_online、gimbal_online、controls_valid 及 GetAIGimbalStatus 已删除。
HostData 每个 Topic 回调调用 `FeedAI(Channel, Data)`，不再汇总并刷新其他通道。
底盘、云台、开火分别记录是否收到过和接收时间；未收到或距离上次接收达到100ms即输出零。
超时在 CMD 处理输入时重新判断；I6x 会至少每5ms提交状态，这不是独立的100ms硬实时看门狗。
正常零目标完整传递；云台包含 roll、pitch、yaw 及其导数。遥控失联仍撤销全部命令。

IMU模块新增 BoardAttitude、DmImu 和公共算法依赖 ImuFusion，见 [IMU接入](imu_modules_20261105.md)。默认板级配置现包含 BoardAttitude，外置 DM 仍未默认启动。

## DM IMU完整链路

DM模块支持CAN请求/应答、当前24字节RS485请求/应答及显式旧固件连续流；没有CAN主动上报模式。
正常编译检查12个构造接口和五份完整配置（外接组合、I6x、DM485、DM CAN、DM旧流），均不链接进默认固件。
参数、单位、配置切换和验证边界见[DM模块补全记录](dm_imu_complete_20261105.md)。

## 四轮底盘

`YunMo-yanyu/Chassis`已加入私有索引及依赖锁定。正常编译现检查13个模块构造和六份完整配置；默认ELF不含底盘控制器。
可选 `User/xrobot_chassis.yaml`复用CAN1四颗M3508（0x201..0x204），缺少云台反馈时保持无力。
命令单位、模式事件及未完成的功率/实车验证见[底盘记录](chassis_module_20261105.md)。

> 2026-11-06 清理更新：本文中的多份独立 YAML 为当时验证入口，现已删除。当前机器人入口统一为 `User/xrobot.yaml`；底盘、DM IMU 增量参数位于 `config/templates/chassis.yaml`、`config/templates/dm_imu.yaml`，由 `tools/generate_external_checks.py` 合并并生成回归组合，不直接运行旧生成命令。历史验证结果保留原样。
