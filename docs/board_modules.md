# 板级模块接入

## 来源与边界

- 工程采用 WDR MC02 基座，LibXR 提交不变。
- BMI088 原复用 `xrobot-org/BMI088`（上游基线 `e4e64d705f5b61685e629a91e123ad2febe75fc7`），2026-11-07 迁移到自维护的 `HFUT-WDR/BMI088`，保留上游历史和许可证，并加入可选校准蜂鸣器。迁移与锁来源见 [xrobot-org 模块迁移](module_migration_20261107.md)。
- 用户按键及 RGB 硬件接法、WS2812 字节编码核对了达妙官方仓库 [dmBots/DM-MC02](https://github.com/dmBots/DM-MC02) 的 `examples/CtrBoard-H7_ALL/Bsp/bsp_user_key.c`、`ws2812.c` 及 `App/imu_task.c`。本工程用 LibXR GPIO/SPI 接口实现板级逻辑，没有引入另一套 HAL 驱动。
- 自研文件采用当前工程 WDR 代码规范。`BoardIO` 只负责 RGB 与按键，`ImuRamDatabase` 只负责 BMI088 临时零偏，设备资源仍来自 WDR BSP 和 LibXR 生成代码。

## 硬件与行为

| 功能 | 配置 |
| --- | --- |
| RGB | SPI6 MOSI PA7，HSE 24 MHz / 4 = 6 MHz；GRB 顺序，0/1 分别编码 0x60/0x78，尾部 64 个零字节，低电平约 85 μs |
| SPI6 缓冲 | RX/TX 各 128 字节，位于 SRAM4 `.ram_d3`；仅 TX DMA 的配置被生成器转为轮询，不能宣称 BDMA 发送已验证 |
| 用户按键 | PA15，板载外部上拉、低有效；10 ms 采样，连续 3 个相同采样确认变化（约 20–30 ms），仅统计按下边沿 |
| BMI088 | SPI2；ACC_CS PC0，GYRO_CS PC3，GYRO_INT PE12；EXTI15_10 已启用，抢占优先级 5 |
| 采样 | 角速度 1000 Hz / ±2000 dps，加速度 1600 Hz / ±6 g；沿用模块的中断通知→采样线程→Topic 发布，ISR 不解析样本 |
| 坐标和单位 | 单位四元数，保持传感器坐标；角速度 rad/s、加速度 g；没有加入 EKF 或 AHRS |
| 加热 | TIM3_CH4，模块配置 30 kHz；PID 系数和输出限幅为零，输出零占空比 |
| 零偏 | RAM 中单独保存 `bmi088_gyro_data` 的三个 float，复位清除；不擦写 Flash，不改变链接分区 |
| 灯/键线程 | LOW，栈 1536 字节，10 ms；默认绿色心跳，`led r g b` 切换固定颜色，`led auto` 恢复心跳 |

`status/echo` 保持原接口。新增 `key/led/imu/health`，`imu` 在命令线程等待下一次陀螺仪和加速度 Topic 消息，最多约 100 ms；不是周期刷屏，不阻塞采样线程。两个消息各自取得下一帧，不保证严格同一帧配对。`health` 提供堆历史最小余量、SPI2/SPI6/ADC3 HAL 错误和 CFSR，不表示整机健康。

## 生成、构建与验证

```sh
libxr stm32 setup -d . --xrobot
xrobot setup --frozen --offline
cmake --preset Debug
cmake --build --preset Debug -j 4
cmake --preset Release
cmake --build --preset Release -j 4
python3 tests/board_support/run_tests.py
```

- Debug 和 Release 构建通过，现有 VS Code 任务继续使用这两个预设及原产物路径，无需更换入口。
- 已检查 LibXR 再生成保留 USER CODE 中的 BoardIO 和 RAM 数据库注册；XRobot 从 YAML 生成 BMI088 实例及监控调用，不手改生成头。
- RAM 存储主机测试使用真实 LibXR Database 接口，ASan/UBSan 通过：默认零偏、同次上电保存/读取、模拟重启清除，以及错误键名/数据长度不覆盖已有数据。使用静态 ASan 解决本机安全软件预加载库的加载顺序问题。
- 构建后 DTCMRAM 静态占用约 92.5 KB / 128 KB（94720 B，余 36352 B），只剩 90 KB RTOS heap 与 2.5 KB 堆栈预留；全局变量按 ST AN4891 放在 AXI SRAM（.data/.bss，可缓存）。这是链接期静态占用，不是运行时堆或线程栈余量测量。
- 初次模块接入时未烧录；启动故障修复后的新板测见下节。此前 `docs/validation/` 中的 550 MHz、USB 和蜂鸣器板测只对应上一版固件，不能当成本轮 BMI088、LED、按键的板测证据。

## 烧录后验收

1. 通过 VS Code 的 `烧录 Debug（J-Link）` 下载，等待终端枚举及 BMI088 初始化完成。
2. `status`、`health`，记录任务数、堆余量、SPI/ADC 错误；重复检查运行时间推进且无新故障。
3. `led 8 0 0`、`led 0 8 0`、`led 0 0 8`、`led 0 0 0`、`led auto`，目视确认颜色、熄灭与心跳；确认 `led` 的错误数不增加。
4. `key`，按住、松开、重复按键，确认状态和按下计数；没有实物按键操作不能声称消抖验收。
5. 连续运行 `imu`，静止时角速度接近零、加速度模长约 1 g；缓慢改变板姿态，检查各轴变化；`bmi088 show 1000 100` 查看温度。加速度计和陀螺仪真实更新率、时间戳、丢样、偏置和温漂仍需进一步测量。
6. 检查 `bmi088_thread`、`board_io`、终端和默认线程的栈余量后，再逐步接其他模块。

上游 BMI088 的原始轴读数全零时会保留上一数值，但仍由采样线程发布 Topic，因此“收到新 Topic”不能独立用作传感器有效性或失控判据。本轮没有电机实例、运动控制输出、Flash 校准写入或温控调参。

## 启动故障修复与首次板测

用户烧录后报告灯未亮。J-Link 读取到 HardFault，CFSR=0x01000000、HFSR=0x40000000，调用链为 `register_board_commands → RamFS::AddNode → RBTree::Search → CompareStr`。

根因是 BoardIO 构造函数把局部变量 `key_command`、`led_command` 加入 RamFS。RamFS 保存节点引用，构造函数退出后节点存储失效；随后添加其他命令时读取了损坏的树。已改为 BoardIO 的 `key_command_`、`led_command_` 成员，与静态 BoardIO 实例同寿命，没有修改 LibXR。

修复版 Debug 已通过 J-Link 下载、校验并复位，Release 构建及 RAM 数据库 ASan/UBSan 回归通过。USB 双 CDC 恢复枚举，连续执行 10 轮 `ls/status/health/key/led/imu` 共 60 次命令均成功：

- 主频 550 MHz、6 个任务、初始化完成后空闲堆与历史最小值均 65056 字节。
- SPI2、SPI6、ADC3 HAL 错误均为 0，CFSR 为 0，LED 发送错误计数为 0。
- `imu` 连续取得新 Topic 数据，静置加速度约 `[0.04,-0.03,1.00] g`，陀螺仪读数接近零。这是静态采样初测，不代表姿态融合、时序或温漂验收。
- 按键原始电平 1、未按下；未代替用户实际按键验证。
- 为便于用户观察，当前运行时设置为固定绿色 `led 0 16 0`；复位后恢复默认绿色心跳。用户随后目视确认灯已亮起且显示绿色；其他颜色、心跳和按键实物操作尚未验收。

原始记录：`docs/validation/board_commands_regression.json`；本次固件哈希及烧录日志分别保存为 `board_modules_fixed_firmware.json`、`board_modules_fixed_flash.log`，不覆盖上一版基座验证记录。

## 实体按键验证补充

用户准备后采集 30 秒，共 250 次状态读取，记录见 `docs/validation/key_physical_test_retry.json`。捕获 4 段按下及松开，按下时 raw=0、松开时 raw=1；计数由 1 递增到 4，持续按住约 2 秒期间不重复计数。某一边沿先读取到 raw=0、pressed=0，下一采样变为 pressed=1，符合消抖延迟。此次按键基本状态、释放及边沿计数验证通过，不代表完整的抖动波形或极限输入测试。结束时 SPI2/SPI6/ADC3 错误和 CFSR 均为 0。

此前颜色序列用户未看到，因此除先前确认的固定绿色外，其他颜色和心跳仍待目视确认；现改为逐色保持，当前固定红色等待确认。

用户已目视确认固定红色正常；当前固定蓝色等待确认。

用户已目视确认固定蓝色正常，RGB 三色均已确认。已恢复绿色心跳，等待闪烁目视确认。

用户已目视确认绿色约半秒亮、半秒灭的心跳正常。板子保持 `led auto` 运行，RGB 三色、熄灭阶段（心跳中）及实体按键基本验收完成；IMU 仍仅完成静态采样初测，完整姿态与时序验收不包含在此结论中。
