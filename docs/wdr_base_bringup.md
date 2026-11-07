# WDR MC02 基座与验收

日期：2026-11-03。工程目录为 `wdr_xrobot_dmmc02_framework`。

## 基底与版本

- MC02 基座保留提交 `8c9ee21` 起的历史，第三方来源见根目录声明文件。
- 工作分支 `codex/wdr-mc02-base`，未提交或推送本轮改动。
- LibXR 子模块 `5161373dbe6d9d9fb1904d90e0ebd4344a44047a`，保持上游基线。
- 生成器 6.0.1，XRobot 1.0.1。
- `xrobot.lock` 锁定首个模块 BuzzerAlarm：`a7c17cf7b1d10409848c32bd420cb393515e7f27`。
- 旧 WDR 工程没有被覆盖。本工程直接继承 MC02 基座的引脚和板级资源组织，包括 USART2/3 RS485、更多 PWM 和 SPI6 BDMA；这些外设的接线和实际功能仍需逐项验收。

## 必要适配

1. 工程和 IOC 改名为 `wdr_xrobot_dmmc02_framework`，同步 CI、Ozone 工程路径。
2. 用官方工具切换到本机可用的 ARM GCC。Debug 应用 -Og、库 -O2，Release -O2。
3. 保留 MC02 基座的 550/275/137.5 MHz 时钟树、TIM4 时基和 98 KB RTOS heap；修正 CAN SJW，使其不超过 TimeSeg2，保持 1 Mbps 仲裁、5 Mbps 数据相位。未测试 CAN 总线收发。
4. USB 从一路 CDC 扩为 Log / Host 两路，采用 WDR 设备字符串。Log 由生成器绑定 STDIO 和独立 2048 字节终端线程，Host 供以后协议使用。
5. 增加 ADC3 温度/VREFINT 两路连续扫描，用 BDMA_Channel1（Channel0 保留给上游 SPI6_TX），缓冲位于 SRAM4 的 `.ram_d3`。
6. 沿用 MC02 基座的 `.axi_ram`、`.ram_d3`、`.ram_d1` 分段，去掉重复 `.ram_d3` 定义，增加 32 字节对齐和 DMA scratch 边界符号。NOLOAD scratch 在 main 的 USER CODE 区清零并清理 Cache；USB 句柄保持上游位于 `.ram_d1` 的实现，不在 scratch 清零时覆盖它。
7. 默认 Flash 数据库关闭：上游 `DatabaseRaw` 构造会初始化存储，而当前生成的 Flash 区域覆盖整个芯片，未划分独立数据区。没有迁入数据库写入或校准存储功能。
8. 引入 MC02 基座的命名/格式规范，增加 `status` 和 `echo token` 诊断命令；没有自研 USB 驱动或终端解析。

> 2026-11-07 更新：已把末尾两个 128 KB 扇区划为数据库区并启用 Flash 数据库，见[Flash 数据库与标定持久化](flash_database_20261107.md)。

## CPU_FREQ_BOOST

初次读取 `FLASH_OPTSR2_CUR=0`，先构建了未刷入的 520 MHz 适配版本。用户明确授权后，通过 OpenOCD 的带掩码选项写入开启 bit2，再恢复 MC02 基座的 550 MHz PLL 参数。

- 当前寄存器 `0x52002070 = 0x00000004`。
- 写入仅对 `0x52002074` 的 bit2 使用 mask `0x4`。
- 对比 OPTSR_CUR、PRAR_CUR1、SCAR_CUR1、WPSN_CUR1、BOOT_CUR、OPTSR2_CUR，只有 BOOST 位发生变化。
- 原值、结果和操作日志保存在 `validation/boost_before.json`、`boost_after.json`、`boost_program.log`。
- 未修改读保护、写保护、启动配置或其它选项。新板需重新核对，固件不会自动开启 BOOST。

依据：[ST STM32H723 数据手册](https://www.st.com/resource/en/datasheet/stm32h723vg.pdf)、[OpenOCD option_write 文档](https://openocd.org/doc-release/html/Flash-Commands.html)。

## 首个 XRobot 模块闭环

沿用上游默认的 `xrobot-org/BuzzerAlarm`，而不是新造测试模块：

`Modules/modules.yaml` 声明依赖 → `xrobot setup` 获取并锁定模块 → `User/xrobot.yaml` 将 buzzer 实例绑定 `pwm_tim12_ch2` → 生成 `User/xrobot_main.hpp` → 编译、烧录 → 板上完成构造。

调试器观测到 buzzer 的静态构造完成标志为 1，初始化提示音流程返回后 TIM12 输出已关闭（CCER=0），主任务进入正常休眠循环。本轮没有示波器/麦克风测量，不据此声称音频波形或响度验收完成。模块还注册了上游的致命错误报警回调。

## 验证结果

- 上游原工程切换 GCC 后基线构建通过；改动后的 Debug / Release 均通过。
- CubeMX、LibXR、XRobot 工具链重新生成成功，用户区诊断接入和 DMA 边界符号保留。
- Debug 已烧录，全部 Flash 装载节（不含节间填充）逐字节与 ELF 对照匹配；板上进入 `app_main`，没有进入 LibXR fatal 路径。
- `SystemCoreClock=550000000`，`xSchedulerRunning=1`，3 个任务，空闲堆 71336 字节。
- `CCR=0x00070200`，I/D Cache 已启用；采样的 CFSR/HFSR/DFSR/MMFAR 均为 0。
- ADC1/3 为 BUSY；ADC3 与 BDMA 无错误，DMA 目标位于 SRAM4。两次缓冲采样可见温度通道约 807–817、参考电压约 1500–1502 的原始值变化；未作精度或温度标定。
- 三路 FDCAN BUSY、ErrorCode=0、TXBRP=0。USART2/3 为 BUSY_RX、无错误；未验证实际协议收发。
- 约 2 s 的采样间隔中 HAL tick 增加 2008、RTOS tick 增加 2007；这只是调度/时基活动检查，不是控制截止时间测试。
- defaultTask 的 16 KB 栈本轮观测到 15548 字节未使用；不代表所有未来业务路径的栈余量。
- Log CDC 完成命令列表、status、21 次 token 回显与重复 status。接收与响应均由主机脚本检查，不只是设备枚举。
- 另完成约 24 秒、12 次 status 采样，运行时间单调增长，任务数恒为 3，空闲堆恒为 71336 字节。
- 证据在 `docs/validation/`。只对 Debug 做了板测，Release 只构建。

## 未验证边界

没有电机业务或电机控制帧；尚未测 CAN/UART 外部设备通信、IMU 读取、PWM 实际波形、Host CDC 数据协议、高频日志吞吐、满负载实时性或长期温升。550 MHz 为实测主频，不代表已经量化相对旧工程的性能收益。
