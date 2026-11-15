# WDR MC02 框架使用教程

本工程用于达妙 DM-MC02（STM32H723VGT6）机器人控制板，基于 FreeRTOS、LibXR 和 XRobot，使用 C++20 开发。

第一次使用时，先按本文完成 **拉取工程 → 恢复模块 → 生成入口 → 编译 → 烧录 → 查看板级状态**，再接入遥控器、电机和机构。当前 `User/xrobot.yaml` 实例化蜂鸣器、板载 BMI088，以及订阅同一采样源的 EKF、Madgwick、ESKF 三路姿态模块，不启动外接电机、底盘、云台或发射机构。

本文使用本工程的文件名和命令。XRobot 的通用机制见 [XRobot 官方文档](https://xrobot.work/)，学习路线可参考 [QDU-Robomaster 战队文档](https://qdu-robomaster.github.io/)。QDU 部分页面使用旧版工具命令，本工程以 `requirements-tools.txt` 固定的版本和本文命令为准。

## 板级基座与产品工程

本仓库是**纯板级基座**：只包含 MC02 板级代码、板载模块（蜂鸣器、BMI088、板载姿态）和构建/CI 基座。底盘、云台、发射、遥控、裁判、上位机通信等产品模块不在本仓库锁定，按需自行加入：

```sh
xrobot module add WDR-Xrobot-org/Chassis
xrobot module add WDR-Xrobot-org/Gimbal
xrobot setup
```

整车工程以本仓库为基座，各自维护独立 BSP 仓库；当前哨兵整车见 [HFUT-WDR/27_sentry](https://github.com/HFUT-WDR/27_sentry)。需要多份完整配置时放在 `User/RobotConfig/` 下，用 `xrobot gen -c User/RobotConfig/你的配置.yaml` 选择。

## 目录

- [1. 准备开发环境](#1-准备开发环境)
- [2. 拉取工程并恢复模块](#2-拉取工程并恢复模块)
- [3. 生成入口并编译](#3-生成入口并编译)
- [4. 烧录与首次运行](#4-烧录与首次运行)
- [5. 修改配置与接入模块](#5-修改配置与接入模块)
- [6. 日常开发与协作](#6-日常开发与协作)
- [7. 常见问题](#7-常见问题)
- [8. 专题文档与参考资料](#8-专题文档与参考资料)

## 1. 准备开发环境

Windows 和 Linux 均可使用；以下命令默认在工程根目录执行。先按 [XRobot STM32 环境配置](https://xrobot.work/docs/env_setup/env-setup-stm32) 安装工具，再核对本工程的要求。

| 工具 | 用途与要求 |
| --- | --- |
| Git | 拉取主工程、LibXR 子模块和 XRobot 模块 |
| Python 3 与 pip | 安装代码生成器、模块管理工具及检查脚本依赖 |
| CMake | 不低于 3.22，使用仓库中的 Debug / Release 预设 |
| Ninja | 预设指定的构建工具 |
| ARM GNU Toolchain | 提供 `arm-none-eabi-gcc`、`arm-none-eabi-g++`、`arm-none-eabi-objcopy` 等命令 |
| OpenOCD | 通过 CMSIS-DAP / DAPLink 或 J-Link 烧录、调试 |
| GDB | 可用 `arm-none-eabi-gdb`；Linux 也可使用 `gdb-multiarch` |
| GitHub CLI（`gh`） | 下文用于配置私有仓库的 HTTPS 认证 |
| STM32CubeMX | 修改芯片外设、引脚和时钟配置时使用 |

确认命令行能找到工具：

```sh
git --version
python --version
cmake --version
ninja --version
arm-none-eabi-gcc --version
openocd --version
```

Linux 若没有 `python` 命令，创建虚拟环境前使用 `python3`。Windows 安装工具后，重新打开终端，使 `PATH` 生效。

Python 包 `libxr` 是代码生成器，工程使用的 LibXR C++ 库位于 Git 子模块 `Middlewares/Third_Party/LibXR/`，两者都需要准备。

## 2. 拉取工程并恢复模块

### 2.1 配置仓库访问权限

主工程位于 `HFUT-WDR`；XRobot 模块与模块索引位于 `WDR-Xrobot-org`。先确认当前账号具有这些仓库的读取权限。使用 HTTPS 时：

```sh
gh auth login
gh auth setup-git
```

若已配置可访问组织仓库的 Git 凭据，可直接使用现有认证方式。不要把 Token 写入 YAML、仓库 URL 或 README。

### 2.2 拉取开发分支

```sh
git clone --branch dev --recurse-submodules https://github.com/HFUT-WDR/wdr_xrobot_dmmc02_framework.git
cd wdr_xrobot_dmmc02_framework
```

若已拉取主工程但未初始化子模块：

```sh
git submodule update --init --recursive
```

子模块包含 LibXR C++ 库和 `ModuleIndex` 模块索引。仅下载主工程 ZIP 不能替代完整的子模块初始化。

### 2.3 安装固定版本的 Python 工具

推荐在独立虚拟环境中安装。Linux / Bash：

```sh
python3 -m venv .venv
source .venv/bin/activate
python -m pip install -r requirements-tools.txt
```

Windows PowerShell：

```powershell
python -m venv .venv
.\.venv\Scripts\Activate.ps1
python -m pip install -r requirements-tools.txt
```

当前工具版本为 `libxr==6.0.1`、`xrobot==1.0.1`；其余 Python 依赖也由 `requirements-tools.txt` 维护。虚拟环境属于个人文件，在本机 `.git/info/exclude` 中加入 `.venv/`，不提交到仓库。

### 2.4 恢复锁定模块

```sh
xrobot setup --frozen
```

该命令按 `xrobot.lock` 恢复模块，并检查配置、生成入口。首次执行需要联网和私有模块仓库的访问权限。完成后，模块源码位于 `Modules/<组织>/<模块名>/`，并生成 `Modules/CMakeLists.txt`。

模块已在本地完整恢复、锁定提交可用时，可离线执行：

```sh
xrobot setup --frozen --offline
```

`--offline` 不会下载缺失依赖，不能用于空目录的首次安装。普通上手流程无需更新模块版本；需要升级依赖时再按[日常开发](#6-日常开发与协作)处理。

## 3. 生成入口并编译

### 3.1 选择默认板级配置

```sh
xrobot gen -c User/xrobot.yaml
```

该命令生成 `User/xrobot_main.hpp`。配置变更后重新生成，不要直接修改生成头文件。每次准备烧录时显式指定配置，可以避免沿用此前选中的其他配置。

本工程已经接入 LibXR，首次拉取后通常无需重新运行板级生成器；只有修改 CubeMX 或底层资源映射时才需要重新生成板级代码。

### 3.2 编译 Debug 或 Release

```sh
cmake --preset Debug
cmake --build --preset Debug -j4
```

Release：

```sh
cmake --preset Release
cmake --build --preset Release -j4
```

| 产物 | Debug 路径 |
| --- | --- |
| ELF（烧录和调试符号） | `build/Debug/wdr_xrobot_dmmc02_framework.elf` |
| HEX | `build/Debug/wdr_xrobot_dmmc02_framework.hex` |
| BIN | `build/Debug/wdr_xrobot_dmmc02_framework.bin` |
| C++ 索引数据库 | `build/Debug/compile_commands.json` |

Release 产物位于 `build/Release/`。纯板级基座只编译 `User/xrobot.yaml` 实例化的模块；未加入清单的模块不会进入构建，加入后由 `xrobot setup` 生成，仍需自行确认“模块编译通过”不代表它已经在板上启动。

## 4. 烧录与首次运行

### 4.1 连接控制板与调试器

将调试器的 SWDIO、SWCLK、GND 和电压参考按 MC02 与探针手册连接；控制板按硬件要求供电，USB 数据线连接主机以查看终端。烧录前关闭占用同一探针的 Ozone、GDB 或其他调试会话。

当前工程使用 **550 MHz** 时钟，目标芯片需启用对应的 `CPU_FREQ_BOOST` 选项。新板先按芯片手册核对这一条件；以下普通烧录命令不会修改选项字节。

### 4.2 使用 CMSIS-DAP / DAPLink 烧录

先完成 Debug 编译，再在工程根目录执行：

```sh
openocd -f interface/cmsis-dap.cfg -f target/stm32h7x.cfg -c "transport select swd" -c "adapter speed 1000" -c "program build/Debug/wdr_xrobot_dmmc02_framework.elf verify reset exit"
```

命令完成下载、校验并复位运行。使用 J-Link 时，将 `interface/cmsis-dap.cfg` 换为 `interface/jlink.cfg`；Release 烧录时将产物路径中的 `Debug` 换为 `Release`。核对输出中下载和校验均成功后，再检查板上运行状态。

也可在自己的 Ozone 或 IDE 调试配置中选择同一 ELF、STM32H723VGT6 和 SWD 接口。个人 `.vscode/`、`.settings/`、Ozone 工程及用户配置已停止入库，仓库不提供统一的快捷键或本机绝对路径。

### 4.3 打开 Log 终端

默认 USB 提供两个 CDC 接口：`if00` 为 Log，`if02` 为 Host 通信。Linux 可通过 `/dev/serial/by-id/` 找到设备；Windows 在设备管理器中确认对应 COM 端口。

使用支持 ANSI/VT100 的串口终端打开 Log CDC，设置为 115200、8N1。USB CDC 的波特率设置不决定 USB 总线速率。输入行最长 32 字符。

先执行：

```text
ls
status
health
echo WDR_READY
```

然后检查板级功能：

| 命令 | 应观察的内容 |
| --- | --- |
| `status` | 主频、运行时间、剩余堆空间、任务数；多次执行时运行时间增长 |
| `health` | SPI / ADC 错误及 CPU 故障状态 |
| `led 8 0 0`、`led 0 0 0`、`led auto` | 红灯、熄灭、恢复自动心跳 |
| `key` | 按下 / 松开时按键状态变化，按下计数递增 |
| `bmi088 show 1000 100` | BMI088 原始采样；原板级 `imu` 命令已删除，采样观察改由模块自带的 `bmi088` 命令提供 |
| `attitude` | 板载姿态序号和时间更新，角度单位为 rad（由 AttitudeEkf 实例注册） |
| `eskf show`、`eskf diag` | ESKF 姿态、更新/丢样与实测步长钳制计数 |
| `ahrs` | Madgwick 模块的命令帮助 |

默认固件没有外置 DM IMU 或底盘实例，`dm_attitude`、`chassis` 命令也不会注册；启用对应模块后由各实例按 `command_name` 注册。`bmi088 cali` 成功后的零偏写入内部 Flash 数据库，复位后保留；温控 PID 为零，默认不加热。Flash 数据区占用末尾两个 128 KB 扇区，布局和维护要求见 [Flash 数据库与标定持久化](docs/flash_database_20261107.md)。

构建成功、烧录校验成功、设备数据收发正常和机构闭环正常是不同验证层次。板级状态通过后，再按具体模块文档进行外接设备测试。

## 5. 修改配置与接入模块

### 5.1 先分清文件职责

| 文件或目录 | 修改内容 |
| --- | --- |
| `wdr_xrobot_dmmc02_framework.ioc` | CubeMX 引脚、外设、DMA 和时钟配置 |
| `User/libxr_config.yaml` | LibXR 外设封装、USB、终端等板级资源配置 |
| `User/app_main.cpp` | 板级入口，通过 `XR_REGISTER` 注册资源，再调用 `XROBOT_MAIN()` |
| `User/xrobot.yaml` | 实际启用的模块、实例顺序、设备参数和资源绑定 |
| `Modules/modules.yaml` | 模块依赖及 XRobot 工具版本 |
| `Modules/sources.yaml`、`ModuleIndex/` | 模块来源和索引 |
| `xrobot.lock` | 模块锁定提交，由工具维护 |
| `Modules/<组织>/<模块名>/` | 独立模块仓库的实现代码 |

电机型号、CAN 总线、ID、方向、PID、零位和几何参数放在机器人 YAML；模块中维护可复用功能。资源名称必须与 `User/app_main.cpp` 中的注册名称一致，例如 `can1`、`spi2`、`pwm_tim12_ch2`。

### 5.2 做一次最小配置修改

在 `User/xrobot.yaml` 的 `buzzer` 实例中，将 `alarm_freq` 从 `1500` 改为 `2000`，然后执行：

```sh
xrobot gen -c User/xrobot.yaml
cmake --preset Debug
cmake --build --preset Debug -j4
```

在生成的 `User/xrobot_main.hpp` 中核对蜂鸣器构造参数变为 `2000`，确认配置已进入构建链路。`alarm_freq` 只影响致命错误报警，上电提示音仍使用模块内固定旋律。改动保留在 YAML 中，生成头文件由工具更新。

### 5.3 接入外接设备与机构

本仓库只锁定板级模块，产品模块需要自行加入。接入顺序为：

1. 用 `xrobot module add <组织>/<模块名>` 添加所需模块，再执行 `xrobot setup`。
2. 阅读模块 README，确认初始化是否发送报文、控制模式及失控行为。
3. 按实际硬件核对总线、ID、方向、单位、限位和超时参数。
4. 在 `User/xrobot.yaml` 中实例化模块并填写真实参数。依赖实例应排在使用它的实例之前。
5. 重新生成、编译，先观察反馈和状态，再验证输出与闭环。

模块的完整说明和整车接线记录维护在各产品 BSP 仓库（如 27_sentry），基座仓库只保留板级接入说明。

如需多份完整配置，可放在 `User/RobotConfig/` 下，通过 `xrobot gen -c User/RobotConfig/你的配置.yaml` 选择。`xrobot setup` 会检查 `User/` 下的机器人配置，不要把未填写完整的草稿放入该目录。

## 6. 日常开发与协作

### 6.1 日常修改流程

修改 YAML 后重新生成，再编译；仅修改普通 C++ 实现时按需编译。开始新功能前检查主工程及模块的未提交改动，模块源码属于独立仓库，不会随主工程提交自动保存。

需要升级某个模块时，在保存好模块本地修改后执行，例如：

```sh
xrobot setup --update WDR-Xrobot-org/Chassis
```

审查模块版本、配置和 `xrobot.lock` 的变化，再构建验证。跨仓库改动应先将模块提交到对应模块仓库，再更新主工程锁文件，避免其他人无法恢复锁定提交。不要通过直接编辑锁文件改变依赖。

### 6.2 修改硬件配置

在 CubeMX 中修改 `.ioc` 并重新生成代码，核对 `User/libxr_config.yaml` 后执行：

```sh
libxr stm32 setup -d . --xrobot
xrobot setup --frozen
xrobot gen -c User/xrobot.yaml
cmake --preset Debug
cmake --build --preset Debug -j4
```

生成前保存当前修改，生成后审查差异。CubeMX 文件只维护 USER CODE 区；LibXR 生成入口与 XRobot 生成头通过工具更新，保留 DMA 链接段和边界符号。详细约定见 [AGENTS.md](AGENTS.md)。

### 6.3 验证与提交

修改板级存储或生成入口时运行 `tests/board_support` 回归：

```sh
python tests/board_support/run_tests.py
```

产品模块（底盘、云台、发射、IMU 采集与解算）的主机回归随各自产品 BSP 维护，例如 27_sentry 的 `tests/chassis`、`tests/power`、`tests/imu`。测试范围按改动选择，主机回归不能代替设备收发和实机闭环。提交前查看 `git diff` 与暂存区，确认只包含本次改动。

提交机器人 YAML、模块清单、锁文件、实现和必要文档；个人 IDE / 调试配置、虚拟环境、构建产物及下载的模块工作副本不应入库。个人目录可在本机 `.git/info/exclude` 中补充忽略规则。

框架的 CI 对 Debug / Release 构建及生成配置进行检查，模块 CI 检查模块依赖和构造接口。CI 不连接控制板；跨私有仓库的权限与维护方式见 [组织仓库与 CI 说明](docs/wdr_organization_ci.md)。

## 7. 常见问题

| 现象 | 检查方法 |
| --- | --- |
| 私有仓库提示无权限或无法找到 | 确认账号有组织仓库权限；核对 HTTPS / SSH 认证及 `ModuleIndex` 子模块是否完整 |
| `xrobot` / `libxr` 找不到或版本不符 | 激活本工程虚拟环境，重新按 `requirements-tools.txt` 安装 |
| `--frozen` 提示锁文件与清单不一致 | 检查是否混用了不同版本的清单与锁文件；需要升级时由维护者正常解析并提交一致版本 |
| 离线恢复失败 | 先联网执行 `xrobot setup --frozen`，确保模块目录和锁定提交都存在 |
| CMake 找不到编译器或 Ninja | 检查 `PATH`，确认使用仓库预设指定的 ARM GCC 工具链 |
| 找不到 `User/xrobot_main.hpp` 或模块 CMake 文件 | 先恢复模块，再运行 `xrobot gen -c User/xrobot.yaml` |
| YAML 资源名称、构造参数或类型报错 | 对照 `XR_REGISTER`、模块构造函数与同版本模板；不要手工修改生成头文件绕过错误 |
| 编译通过但新功能没有运行 | 检查所选配置是否实际实例化了模块，再确认已烧录本次生成的 ELF |
| 调试探针无法连接或被占用 | 关闭其他调试会话，核对板卡供电、SWD 接线、驱动权限和探针类型 |
| USB 无输出 | 确认打开的是 Log CDC，检查是否停在断点或异常；结束调试后复位再观察 |
| `attitude` 命令不存在或等待超时 | 检查 AttitudeEkf 实例是否启用、`command_name` 是否与调用名一致、Topic 名称和采样链路；单次超时不等于已定位硬件故障 |
| 配置修改后效果不变 | 重新执行生成、编译、烧录，并确认选择的是同一份配置和产物 |

## 8. 专题文档与参考资料

| 内容 | 入口 |
| --- | --- |
| LibXR / XRobot 环境、模块管理与生成工具 | [XRobot 官方文档](https://xrobot.work/)、[项目管理](https://xrobot.work/docs/proj_man)、[配置说明](https://xrobot.work/docs/proj_man/proj-man-config) |
| 从板级程序到机器人配置的学习路线 | [QDU-Robomaster 文档](https://qdu-robomaster.github.io/)、[快速上手](https://qdu-robomaster.github.io/电控组/quick-start) |
| 板级资源及验收记录 | [板级模块](docs/board_modules.md)、[基座迁移与验收](docs/wdr_base_bringup.md) |
| 模块 README、协议与迁移文档维护 | [模块文档约定](docs/module_documentation.md) |
| 开发规范 | [AGENTS.md](AGENTS.md)、[模块开发规范](docs/module_development_standard.md)、[逐模块 API 改造方案](docs/module_api_migration_plan.md) |
| 第三方来源与许可证 | [THIRD_PARTY_NOTICES.md](THIRD_PARTY_NOTICES.md)、[LICENSE](LICENSE)、[NOTICE](NOTICE) |

专题记录中的日期和验收结论对应当时的配置与固件；判断当前固件是否包含某功能，应以本次选择的 YAML、构建产物和实际测试为准。
