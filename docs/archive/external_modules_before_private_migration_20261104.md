# 基础设备模块接入（2026-11-04）

本轮在 QDU MC02 基座中完成外接基础模块的依赖解析、配置模板和 ARM 编译检查。
默认 `User/xrobot.yaml` 仍只有 BuzzerAlarm、BMI088；RGB、按键、日志命令保留在板级入口。
没有新增底盘、云台、发射业务，也没有烧录或操作外接设备。

## 已接入的模块

| 用途 | 模块 | 依赖／接口 |
| --- | --- | --- |
| DJI 电机 | QDU-Robomaster/RMMotor | Motor、LibXR CAN；M2006、M3508、GM6020 |
| 达妙电机 | QDU-Robomaster/DMMotor | Motor、LibXR CAN；DM4310、DM8009 |
| DR16 | QDU-Robomaster/DR16 | CMD、LibXR UART；100000 bit/s、8 数据位、偶校验、1 停止位 |
| 裁判系统 | QDU-Robomaster/Referee | CMD 类型、UART；模板 115200 bit/s、8N1 |
| 上位机数据 | QDU-Robomaster/HostData | CMD；创建三路接收 Topic |
| 上位机接收 | xrobot-org/SharedTopic | UART 接口、RamFS；在任务内解析 Topic 包 |
| 上位机发送 | xrobot-org/SharedTopicClient | UART 接口；订阅 Topic 并打包发送 |

CMD 仅作为 QDU 输入数据依赖，不包含机构控制任务。Motor 是抽象接口，不创建实例。
主机链路可使用已注册的 USB 第二路 CDC（`usb_otg_hs_cdc2`），该对象实现 LibXR UART 接口。
第一路 CDC（`usb_otg_hs_cdc`）留给日志终端。SharedTopic 接收与 SharedTopicClient 发送
可共用第二路 CDC 的独立读写端，但不能再交给另一个接收消费者。

依赖请求在 `Modules/modules.yaml`，实际源码提交在 `xrobot.lock`。
模块目录由工具拉取，不在忽略的外部源码中打补丁。
复现使用 `xrobot setup --frozen`；已拉取时可加 `--offline`，避免改变锁定版本。

## 模板及启用步骤

`config/templates/external_devices.yaml.in` 是待填写模板，位于 User 目录之外，扩展名也不是
`.yaml`，不会被 setup 当作可启动配置。电机型号、方向、ID、CAN 口及串口绑定均留空。
模板未选定硬件；`tests/external_modules/compile_bindings.json` 内的 CAN1/CAN2、串口、
M3508/DM4310 和示例 ID **仅用于编译，不代表板子的实际外接接线**。

以后确认接线时：

1. 以当前 `User/xrobot.yaml` 为底复制一份配置，只合并实际需要启用的模板条目。
   不要用仅包含外接设备的模板覆盖板级配置。
2. 填写所选实例全部 `null`。电机型号、反馈／控制 ID、方向须与设备配置一致；不同设备
   独占各自接收 UART，核对 TTL／RS485 电气接口。DR16 的串口还需核对 HAL 生成配置
   对 8 数据位加偶校验的字长支持，不能把 USB CDC 当成 DR16 接收口。
3. 依赖顺序保持 CMD 在 DR16／HostData 前，HostData 在 SharedTopic 前；BMI088 在
   发送其遥测的 SharedTopicClient 前。Referee 的 `cmd: nullptr` 表示只接裁判数据，
   不将图传键鼠混入遥控来源。
4. 模板的双重引号如 `'"target_euler"'` 表示 C++ 字符串字面量，保留它们。
   SharedTopic 的 Topic 必须已经存在，接收列表和发送列表不能相同，否则可能回环。
5. 用 `xrobot gen -c User/<实际配置>.yaml` 显式选中，再构建并检查硬件参数。
   这一步会让相应构造函数启动接收线程／登记电机；不再属于本轮的默认板级配置。
   恢复板级固件用 `xrobot gen -c User/xrobot.yaml`。

主机接收 Topic 为 `target_euler`、`host_chassis_data`、`host_fire_notify`，类型由
`HostData.hpp` 定义；示例发送 `bmi088_gyro`、`bmi088_accl`。这采用 LibXR Topic 包格式，
并非现有其他工程的裸结构体串口协议。对端必须匹配消息格式、类型布局、Topic 名称与单位。
模板没有添加机构或发送运动指令的任务。

## 源码审查确认的运行约定

- RMMotor、DMMotor 构造只登记 CAN 反馈，不自动使能或周期发送。
- RMMotor 同 CAN、同控制 ID 的实例共用四槽控制帧，必须让组内所有已登记实例都提交一轮
  命令才发帧。Disable／Relax 也只是提交零电流，不能当作立即独立发帧。
  不能照搬其他工程的 DjiGroup::Send 规则。
- RMMotor 的 MODE_CURRENT 从 MotorCmd.velocity 读取归一化电流 `[-1,1]`，不是 rad/s；
  MODE_TORQUE 使用力矩和减速比。当前接口没有单独的 GM6020 电压模式配置。
- DMMotor 明确调用 Enable 才发送 `0xFC`，Disable／Relax 发送 `0xFD`；反馈 ID 固定按
  `0x10 + can_id` 登记，必须与电调配置一致。上游的超过 90℃ 分支先失能但随后仍发控制帧，
  不能把它当成完整的停机保护；本轮不启用输出或修改外部驱动。
- DR16 构造即配置串口并创建接收线程，源代码有约 100 ms 离线处理。
- Referee 构造创建接收线程，CRC8／CRC16、长度检查由原模块处理；发送 UI、交互数据等
  需要显式调用，不因引入依赖自动发送。锁定版本说明为 2025 裁判协议，未证明与后续赛季
  协议全部一致，实车使用前按赛季协议核对。
- HostData 的零速度、零 pitch/yaw 含有“离线”语义，虽然记录接收时间但没有周期超时清理。
  CMD 也不能替代机构失控保护；不能把最后一次非零命令当成持续有效的上位机指令。
  后续接机构前需单独确认零目标、有效位、时效和 STOP 契约。

## 自动编译检查

普通 Debug／Release 构建同时构建 `external_module_checks` OBJECT 目标：

- `xrobot check-module --offline` 为 8 个具体模块／依赖生成构造接口检查。
- 用待填写模板加独立示例绑定，在临时 BSP 中运行 `xrobot gen`，再编译完整生成头，检查
  参数类型、实例引用及整份配置。原工程的已选配置和生成头不被替换。
- 所有检查代码位于 `build/<preset>/external_modules`，不链接进运行固件。生成函数中的
  空指针占位属于工具的编译检查接口，绝不能执行。
- `Motor.hpp` 通过两个具体驱动覆盖；检查不需要抽象 Motor 实例。

```sh
xrobot setup --frozen --offline
cmake --preset Debug
cmake --build --preset Debug -j 4
cmake --preset Release
cmake --build --preset Release -j 4
python3 tests/external_modules/run_tests.py --preset Debug
python3 tests/external_modules/run_tests.py --preset Release
python3 tests/board_support/run_tests.py
```

需要 Python3、PyYAML 和 PATH 中的 xrobot。`requirements-tools.txt` 记录工具依赖。
VS Code 的普通 Debug／Release 编译会覆盖这些检查；也可运行“检查外接模块（仅编译）”。
烧录任务仍然只烧录正常板级 ELF，选项字节参数未改。

本轮通过 Debug／Release 固件链接、外接模块单独与组合配置编译、上游裁判结构布局断言、
板级内存数据库回归，以及两种 ELF 中没有外接模块符号的隔离检查。验证记录见
`docs/validation/external_modules_20261104.json`。
这些结果不等于电机收发／停机、遥控实际帧、裁判系统、USB 主机协议互通或机构闭环验证。

## 本地 i6x 输入扩展（2026-11-04）

`User/remote/i6x.hpp` 提供 `I6x(uart, cmd, topic_name)`，使用与 26_sentry 相同的
25 字节 SBUS、100000 bit/s、8E2 接口，六路通道映射到 ±660，并交换左摇杆两个通道。
四路拨杆保持原编码：+1 是实际下拨、0 中位、-1 实际上拨。

- s[2]：-1 云台无力、0 手动、+1 上位机。
- s[3]：-1 底盘无力；+1 时依 s[2] 选择手动不跟随／上位机不跟随。
- s[0]：+1 开火意图；s[1] 进入 +1 的边沿增加摩擦轮切换序号。
- s[1]=-1 且 s[3]=+1：保留原工程上位机发射控制许可。
- 原始六通道、四拨杆、序号、最后有效接收时间、failsafe/frame_lost，以及以上意图通过
  `i6x_input` Topic 保存。CMD 输出为归一化输入；实际速度、云台弧度增量系数、摩擦轮切换、
  到速／热量许可和上位机 PASSING_MODE 等机构行为不在输入模块实现。没有接入机构任务。

CMD 原接口无法表达独立的底盘／云台来源；本地补丁新增独立 `RC_INPUT_I6X` 和输入策略。
原 DR16／VT13 路径保留，不冒用它们的槽。i6x 的无力档不退回上位机或遥控目标；上位机数据
超过 100 ms 时 i6x 的上位机分支归零，不回退到摇杆。此 100 ms 是本地链路时效配置，非原工程
业务状态机的完整迁移。当前 QDU Topic 仅含目标，不含完整机构使能；后续机构任务必须使用
`i6x_input` 中的无力意图执行停机，不能把零速度目标等同于零力矩。

单帧丢帧／失控时清摇杆和开火许可，保留最后有效拨杆；这些帧不刷新有效接收时间。
连续 3 帧带失控标志或超过 30 ms 无有效帧时离线。重新上线首帧只建立摩擦轮边沿基准，不自动
触发开轮。信号反相和 STM32 HAL 对 8 数据位加校验的字长必须结合实际 UART/接线核对。

本地 CMD 修改由 `patches/cmd_i6x.patch` 跟踪，`tools/apply_module_patches.py` 校验修改前后
SHA256 后应用；遇到其他修改会停止，不覆盖。CMake 配置会执行此脚本。升级 CMD 前必须审查
补丁及回归，不在被忽略的模块副本中留下无记录修改。

重新拉取模块后按以下顺序准备（生成器和编译检查均应读取已应用补丁的接口）：

```sh
xrobot setup --frozen --offline
python3 tools/apply_module_patches.py
xrobot gen -c User/xrobot.yaml
cmake --preset Debug
cmake --build --preset Debug -j 4
python3 tests/i6x/run_tests.py
```

`config/templates/i6x_input.cpp.in` 提供本地构造模板。它不是远程索引里的 XRobot 模块，
不伪造模块锁记录。明确串口后，在 app_main 的 USER CODE 区创建单个 CMD 和 I6x，并用
XR_REGISTER 注册 cmd，使 HostData 等 YAML 模块绑定同一个中枢；移除 YAML 内重复的 CMD
构造项。默认配置没有合并此模板，也没有烧录。本轮编译检查目标会编译其真实构造函数。

主机回归逐一对照未修改的 26_sentry i6x.c 的 1568 组通道样例，并覆盖半帧、错位、错误尾、
失控计数、时间回绕、超时及真实 CMD 的 STOP、来源切换与 AI 过期。LibXR 端口／Topic 在主机
测试中使用替身；ARM Debug／Release 另用真实 LibXR 头编译，不能替代接收机硬件验收。


### MC02 原厂资料核对

资料根目录：`/home/yunmo/文档/设备手册/DM/控制板/dm-mc02`。
`2D图纸/原理图/CtrBoard-H7_V1.0-240124.pdf` 第5页明确给出 P6：

| P6 引脚 | 信号 | 板内路径 |
| --- | --- | --- |
| 1 | SBUS/DBUS S | R21 → Q1 反相 → PD2 / UART5_RX，R19 上拉到3.3 V |
| 2 | 5 V | VCC_5VS |
| 3 | GND | 地 |

使用专用 P6 接口时，外部信号已由 Q1 反相，MCU UART 不应再次启用 RX 反相。
接收机必须输出 SBUS；iBUS/PWM 输出不能直接套用本解码器。插头方向按板上标记和引脚编号
核对，不能以照片左右猜测。

`说明书/达妙科技DM-MC-Board02电机开发板使用说明书V1.1.pdf` 的“6、SBUS接口”写明
100000 bit/s、8数据位、偶校验、2停止位，但表格将 MCU 引脚写成 PC02；这与原理图 PD02
及当前 IOC 的 `PD2.Signal=UART5_RX` 不一致，当前绑定按相互吻合的原理图和 IOC 采用 UART5。

当前 Cube 初始化是 100000、HAL 9B字长、偶校验、1停止位（DR16配置）。本地 I6x 构造通过
LibXR `SetConfig({100000, EVEN, 8, 2})` 切为8E2；已核对锁定版 STM32 UART 驱动将
8数据位+偶校验正确转换为 HAL 9B字长，并支持2停止位。启用前应检查 SetConfig 返回值和
实际接收，硬件收包尚未验收。模板已绑定 `uart5`，但仍不加入默认构造／烧录。


### 与实机 26_sentry 的再次对照

已核对参考工程 `Core/Src/usart.c` 和 IOC：UART5/PD2、100000、HAL 9B + EVEN、2停止位、
只接收、无高级反相配置。其 `remote_control_init()` 使用 huart5、25字节缓冲，
`BSP_UART_Init` 用 ReceiveToIdle DMA；完整帧回调解包，然后刷新 detect。
当前接入复用 LibXR 的 DMA/字节队列，在接收任务中组帧，不重复接管 HAL UART 回调。

参考 `detect.c` 第一项对应 DBUS_TOE，离线参数30ms、上线稳定参数20ms。
本地接收超时已从100ms修正为严格大于30ms；5ms维护节奏带来扫描延迟，不能认为恰好30ms
完成停机。现有本地首个合法帧立即置接收在线，没有迁移原 detect 的20ms恢复稳定窗口；
接机构前应明确复用对应健康判据，当前不是完整实车行为等价证明。

参考 `chassis_no_follow_yaw_control()` 将 ch[4] 从[-660,660]映射到[0,1]后乘业务最大转速。
当前 CMD 输入已修正为相同[0,1]比例，实际最大转速仍由后续机构层设置。
原始 SBUS 解码仍与26_sentry逐项对照；旧验证记录对应修正前版本，本次复测结果见
`docs/validation/i6x_reference_recheck_20261104.json`。
