# 26_sentry 整车遥控构建

完整机器人配置为 `User/RobotConfig/sentry_remote.yaml`。它同时实例化 I6x、四轮底盘、功率控制、双Yaw、DM4310 Pitch、外置DM IMU、裁判和发射机构，不依赖上位机通信。无需分别构建或烧录各机构。

## 构建与产物

```sh
xrobot gen -c User/RobotConfig/sentry_remote.yaml
cmake --preset Debug
cmake --build --preset Debug -j4
cmake --preset Release
cmake --build --preset Release -j4
```

整车 ELF、HEX、BIN 位于 `build/Debug/` 和 `build/Release/`，文件名均为 `wdr_xrobot_dmmc02_framework`。该路径与板级固件共用，烧录前核对生成头首行和所选配置；选择整车配置后生成入口会调用全部机构构造，而不是只做未启用模块编译检查。

`User/xrobot.yaml` 保留为板级配置；CI 同时构建 `default` 和 `sentry_remote` 的 Debug/Release。当前整车配置使用 I6x 的 OP 路径，上位机协议尚未接入。

## 零位与校准

- Pitch：DM4310 电机自身提供零位。`pitch.param.zero=0` 表示不再叠加软件偏移，位置反馈直接使用电机坐标；归中目标使用该电机零位。不增加Pitch端点扫描，不发送电机设零命令。
- Yaw：两个编码器零位通过现有 `Gimbal::Mode::CALI` 获取，校准过程同时更新零位和机械相对角限位。YAML中的初始值不视为已完成机械校准。
- `database=nullptr`：Yaw校准结果在本次上电期间生效，重启后重新校准；本次不新增Flash存储功能。

## 遥控绑定

下表使用I6x模块中的逻辑编号，与其他模板保持一致。

| 操作 | 行为 |
| --- | --- |
| SW2中位 | CMD选择OP，云台绝对角遥控 |
| SW2下位 | 云台RELAX，停止许可联动底盘和发射 |
| SW2上位 | 保留AUTO选择；当前无上位机有效目标时云台保持无力 |
| SW3上位，且SW2中位/上位 | 请求底盘NO_FOLLOW_YAW，仍受云台停止许可约束 |
| SW1下位 + SW3上位保持 | Yaw CALI；该保持事件在模式保持事件之后发出，覆盖本帧普通模式请求 |
| CALI完成且保持原组合 | 保持RELAX，不重复扫描；退出组合后按SW2当前档位恢复模式 |
| SW1切入上位 | 切换摩擦轮开启请求，使用边沿事件，不按帧重复切换 |
| SW0上位 | OP开火命令，仍经过到速、热量、在线与云台许可判断 |

CALI期间Pitch保持失能，底盘和发射被云台联锁停止。CALI是显式操作，不在上电时自动扫描机械端点。链路失联时保持已有保护；恢复后模式跟随拨杆电平，不要求重新拨杆。

## 初始化与验证范围

电机模块构造只绑定设备，不发送控制或使能帧；控制线程运行后会根据模式、反馈和许可发送输出。默认模式为RELAX；收到有效的遥控保持事件后会进入对应模式。底盘与云台各自持有控制线程，发射由云台线程同拍调用，CAN3的yaw和拨弹共用0x1FF完整组帧。

配置沿用已迁移模板的总线、ID、PID、方向及安装矩阵。本次启用整车构建，不代表已经实测这些参数与实车安装相符，也不代表已进行烧录、设备收发或机构闭环验收。

构建后可运行 `python tests/external_modules/run_tests.py --preset Debug`（Release同理），确认当前ELF包含底盘、功率、双Yaw、Pitch、云台和发射的实际控制符号，并检查裁判协议布局。
