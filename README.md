# WDR XRobot DM-MC02 Framework

WDR MC02 机器人固件基座，使用 LibXR 与 XRobot 管理板级资源和功能模块。第三方来源与许可证见 [声明文件](THIRD_PARTY_NOTICES.md)。

当前：STM32H723VGT6，CPU **550 MHz**，I/D Cache；TIM4 时基；双 USB CDC（Log / Host）；ADC3 温度和参考电压 BDMA 采集；XRobot 模块接入 `BuzzerAlarm`、`BMI088` 和 `BoardAttitude`，另有板级 RGB 灯、按键及状态诊断。原有板级模块已烧录并通过终端和 IMU 采样初测，新姿态模块尚未板测；用户已目视确认 RGB 红/绿/蓝及绿色心跳正常，实体按键按下/松开及边沿计数验证通过。

## 生成和构建

工具版本：`libxr==6.0.1`、`xrobot==1.0.1`。本机使用 ARM GCC；原 starm-clang 配置保留，可用官方 `libxr stm32 toolchain` 切换。

```sh
git submodule update --init --recursive
libxr stm32 setup -d . --xrobot
xrobot setup
cmake --preset Debug
cmake --build --preset Debug -j4
cmake --preset Release
cmake --build --preset Release -j4
```

修改硬件后先由 CubeMX 重新生成 `wdr_xrobot_dmmc02_framework.ioc` 对应工程，再运行上述接入工具。HAL / LibXR 生成代码只改 USER CODE 区；DMA 链接段及边界符号必须保留。

产物：`build/Debug/wdr_xrobot_dmmc02_framework.elf`（及 HEX/BIN）；Release 同理。

## VS Code 编译、烧录和调试

在 VS Code 中直接打开本工程根目录。安装推荐的 CMake Tools、C/C++ 和 Cortex-Debug 扩展；本机工具为 CMake、Ninja、ARM GCC、OpenOCD 和 gdb-multiarch。

- `Ctrl+Shift+B`：配置并编译 Debug。
- “终端 → 运行任务”：选择 `编译 Release` 或 `烧录 Debug（J-Link）` / `烧录 Release（J-Link）`。
- 烧录任务自动先编译，再通过 J-Link / SWD 1 MHz 下载、校验、复位运行。
- `F5`：使用 Debug 固件启动 Cortex-Debug，停在 `main`；该操作会下载固件。

烧录和调试时，先关闭占用 J-Link 的 Ozone 或其他调试会话。F5 停在断点时 USB 可能暂时不可用；结束后若板子停住，可按复位键恢复运行。普通烧录不修改 CPU_FREQ_BOOST 等选项字节。配置中的 Linux 工具路径更换机器时需核对。

## 使用日志终端

USB 接主机后，接口 `if00` 为 Log，`if02` 为 Host。使用 ANSI/VT100 串口终端打开 Log CDC（115200 8N1），执行：

```text
ls
status
echo WDR_READY
```

输入行最大 32 字符，echo 接受单个 token。USB CDC 波特率设置不决定 USB 总线速率。

```sh
python3 tools/check_terminal.py --port /dev/serial/by-id/实际设备的-if00
```

## 板级模块

| 功能 | 接口 | 终端命令 |
| --- | --- | --- |
| RGB 灯 | SPI6 / PA7，WS2812；默认绿色心跳 | `led`、`led 8 0 0`、`led 0 0 0`、`led auto` |
| 用户按键 | PA15，低有效；10 ms 采样、连续 3 拍消抖 | `key`：按下状态、累计按下次数和原始电平 |
| BMI088 | SPI2、PC0/PC3 片选、PE12 数据就绪；TIM3_CH4 加热输出为零 | `imu`：等待新采样，角速度 rad/s、加速度 g；`bmi088 show 1000 100`：上游温度/数据查看 |
| 运行诊断 | 主频、运行时间、堆、任务数、SPI/ADC 错误和 CFSR | `status`、`health` |

BMI088 采用 XRobot 官方模块的固定提交，由 `xrobot.lock` 记录；参数和硬件绑定集中在 `User/xrobot.yaml`：陀螺仪 1000 Hz、±2000 dps，加速度 1600 Hz、±6 g，坐标旋转为单位四元数；原始六轴数据交给 `BoardAttitude` 做原26_sentry姿态融合。新姿态模块尚未板测。温控 PID 系数为零，因此不加热。

`ImuRamDatabase` 只保存当前上电周期的零偏，复位即清除；没有开启 Flash 数据库。上游 `bmi088 cali` 为约 3 分钟的阻塞命令，本轮未执行或验证。`imu` 的新 Topic 消息不等价于有效姿态或控制系统在线判断。

SPI6 目前仅有 TX DMA；LibXR 生成器按其规则选择轮询发送。单次灯帧 88 字节，正常线速下约 117 μs，未测真实调度耗时。

接入范围、来源和验证见 [板级模块记录](docs/board_modules.md)。

## 板级条件与验证

550 MHz 要求芯片已启用 CPU_FREQ_BOOST。本次经用户授权设置，采样选项寄存器前后对比确认只改变 BOOST 位；新板烧录前须先核对，应用不会自动改选项字节。

内部 Flash 数据库暂时关闭；上游生成的默认 Flash 映射覆盖整个芯片，需先划分独立数据库区域才能启用。

配置变化、首次模块闭环与板测证据见 [迁移与验收记录](docs/wdr_base_bringup.md)。命名与格式遵循 [AGENTS.md](AGENTS.md)。

保留上游 [LICENSE](LICENSE) 和 [NOTICE](NOTICE)。

## 外接基础模块（已接依赖，默认不启动）

已引入 WDR 维护的 DJI／达妙电机、DR16、裁判系统、HostData，以及 CMD／Motor 和
SharedTopic 双向通信依赖。型号、ID 和接线未确认的实例不加入默认启动配置。
配置模板、参数含义、驱动差异和后续启用步骤见 [基础设备模块接入](docs/external_modules.md)。

正常 Debug／Release 编译会同时检查外接模块和组合 YAML；检查对象不链接进板级固件。
VS Code 新增“检查外接模块（仅编译）”，原编译和 J-Link 烧录入口继续使用板级配置。


## 当前私有仓库与首次准备

主工程发布到 `YunMo-yanyu/wdr_xrobot_dmmc02_framework`，组件索引在
`YunMo-yanyu/xrobot-modules`，CMD、Motor、RMMotor、DMMotor、DR16、Referee、HostData、I6x
均由同账号的私有仓库独立维护。采用独立组件仓库，保留历史和第三方许可证。
LibXR 和 xrobot-org 官方模块继续使用上游。

```sh
gh auth login
gh auth setup-git
git clone --recurse-submodules https://github.com/YunMo-yanyu/wdr_xrobot_dmmc02_framework.git
cd wdr_xrobot_dmmc02_framework
python3 -m pip install -r requirements-tools.txt
xrobot setup --frozen
cmake --preset Debug
cmake --build --preset Debug -j 4
```

私有索引通过 `ModuleIndex` Git 子模块读取；新机器需要具备各私有仓库读取权限。
继承的 Actions 工作流归档于 `docs/archive/ci`，尚未配置私有跨仓库 CI 凭据，本地构建入口不变。

I6x现为正式XRobot模块，配置为`YunMo-yanyu/I6x`。`config/templates/chassis.yaml`包含 I6x 接入参数；
默认仍选择不含外接接收实例的`User/xrobot.yaml`。确认接线后才显式切换配置、构建并烧录。
详见[模块接入说明](docs/external_modules.md)与[私有仓库迁移记录](docs/private_repositories_20261104.md)。

## 姿态模块（2026-11-05）

默认 `User/xrobot.yaml` 增加板载姿态解算，单颗外置 DM 配置为 `config/templates/dm_imu.yaml`。
默认安装矩阵为单位阵；外置 DM 不默认启动。新增终端命令 `attitude` 与 `dm_attitude`。
设计、单位、配置、验证和未验证项见 [IMU接入记录](docs/imu_modules_20261105.md)。
VS Code 的“测试 IMU（主机）”执行冻结算法和协议回归；普通编译仍同时检查未启用模块。
本次未烧录，Flash 中仍为此前板级固件；原硬件验收不能代替新姿态模块验收。

DM模块已补齐CAN请求/应答、现版24字节RS485请求/应答与显式旧流兼容，支持原始测量、设备姿态/本地融合及诊断输出。
CAN 请求、RS485 请求和流式组合由编译检查工具从同一增量模板生成，默认不启用外置设备。
不含CAN主动上报模式；协议范围、配置与硬件未验证项见[DM模块补全记录](docs/dm_imu_complete_20261105.md)。

## 底盘模块（2026-11-05）

已引入 `YunMo-yanyu/Chassis`，增量模板为 `config/templates/chassis.yaml`；默认配置不启动底盘。
功能与参数沿用26_sentry，模块使用Topic、事件和YAML组装。云台反馈接口已定义，缺失或停更时保持无力。
终端 `chassis` 只读状态，VS Code“测试底盘（主机）”执行离线回归。
配置、保护、验证及功率控制尚未接入的边界见[底盘接入记录](docs/chassis_module_20261105.md)。本轮未烧录。

底盘命令现统一为m/s、rad/s，遥控死区在输入端处理；普通零目标保留闭环制动。接口与本轮验证见[底盘命令契约](docs/chassis_contract_20261105.md)。

DAPLink/CMSIS-DAP已提供“烧录 Debug（DAPLink）”“烧录 Release（DAPLink）”任务及对应调试入口。采用SWD 1MHz，先编译再烧录、校验和复位，不修改选项字节。2026-11-05板级烧录及运行检查见[验证记录](docs/validation/daplink_20261105.json)。

PID与LPF已提取到公共ControlAlgorithms依赖，底盘改用LibXR PID基座并补齐导数滤波/首拍处理。选型、旧实现问题、增益换算及验证见[公共控制算法记录](docs/control_algorithms_20261106.md)。VS Code新增“测试公共控制算法（主机）”任务。

当前只使用 `User/xrobot.yaml` 作为机器人配置；`User/libxr_config.yaml` 配置底层外设。
`config/templates/` 是未启用设备的增量参数，供编译回归使用，不能直接作为完整机器人配置生成。

底盘故障现为可在调试器展开的具名布尔字段，见[故障快照说明](docs/chassis_faults_20261106.md)。

底盘功率控制已补齐RLS、能量环、逐轮功率分配与26_sentry超电协议；默认固件不启用。参数、离线回退及硬件未验证项见[功率控制接入](docs/chassis_power_20261106.md)。
