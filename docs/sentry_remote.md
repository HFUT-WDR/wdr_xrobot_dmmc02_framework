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

`User/xrobot.yaml` 保留为板级配置；CI 同时构建 `default` 和 `sentry_remote` 的 Debug/Release。当前整车配置使用 I6x 的 OP 路径，Host Topic通信已接入，实机USB与自动控制仍待验证。

## 零位与校准

- Pitch：DM4310 电机自身提供零位。`gimbal.param.pitch.zero=0` 表示不再叠加软件偏移，位置反馈直接使用电机坐标；归中目标使用该电机零位。不增加Pitch端点扫描，不发送电机设零命令。
- Yaw：两个编码器零位通过现有 `Gimbal::Mode::CALI` 获取，校准过程同时更新零位和机械相对角限位。YAML中的初始值不视为已完成机械校准。
- `database`：Yaw校准结果通过 Flash 数据库的 `gimbal_yaw_calibration` 键保存，上电自动加载；写入成功时 `gimbal_status.calibration_saved` 为 true。存储布局和未验证项见[Flash 数据库与标定持久化](flash_database_20261107.md)。

## 遥控绑定

下表按拨杆实际物理方向（上拨/中位/下拨）描述。

| 操作 | 行为 |
| --- | --- |
| SW2中位 | CMD选择OP，云台绝对角遥控 |
| SW2下位 | CMD选择AUTO，云台AUTO；当前无上位机有效目标时云台保持无力 |
| SW2上位 | 云台RELAX，停止许可联动底盘和发射 |
| SW3下位，且SW2中位/下位 | 请求底盘NO_FOLLOW_YAW，仍受云台停止许可约束 |
| SW2上位、SW0下位＋两阶段摇杆手势 | 申请一次Yaw CALI，手势与原26_sentry一致 |
| CALI完成且SW2仍上位 | 回到RELAX，不重复扫描；再次校准须重新完成两阶段手势 |
| SW1切入下位 | 切换摩擦轮开启请求，使用边沿事件，不按帧重复切换 |
| SW1上位＋SW0下位 | OP拨弹意图，仍经过摩擦轮开启、到速、热量、在线与云台许可判断 |

AUTO底盘仅接收上位机平移，旋转由遥控旋钮提供（0～12 rad/s）；PC平移超时归零，遥控失联或云台停止许可仍停止整车。OP/AUTO互切会复位发射状态，返回OP须重新开启摩擦轮。

校准手势使用完成原车映射/交换后的ch[0..3]：先保持`(-,-,+,-)`、再保持`(+,+,-,+)`，每个分量严格超过统一宏门限±550，每阶段连续超过2000ms，第二阶段在20s选择窗口内完成。就绪拨杆为SW2上位、SW0下位；SW1/SW3不参与校准选择。确认后释放摇杆、保持SW2上位，扫描继续至结束，普通无力保持事件不覆盖已接受的校准。

第一阶段手势成功进入选择窗口后，蜂鸣器以4 kHz间歇提示（响200 ms、停200 ms）；前10 s为22%占空比，后10 s为32%。确认并实际进入CALI后以32%持续提示；超时、失联、校准完成或失败后关闭。

CALI期间Pitch保持零刚度、零阻尼、零力矩MIT输出，底盘和发射被云台联锁停止。CALI是显式操作，不在上电时自动扫描机械端点。链路失联时保持已有保护；恢复后云台和底盘模式跟随拨杆电平。发射在无力、INIT、CALI期间进入`SHOOT_STOP`，不记住摩擦轮开启操作；恢复OP后须重新拨动SW1，AUTO则按当前AIM状态重新起转。

## 初始化与验证范围

电机模块构造只绑定设备，不发送控制或使能帧；控制线程运行后会根据模式、反馈和许可发送输出。默认模式为RELAX；Pitch在云台线程首拍即尝试使能，掉线或失能按500 ms间隔重试，不等待遥控、Yaw或IMU就绪。无力时Pitch保留使能并发送零刚度、零阻尼、零力矩MIT，收到有效的遥控保持事件后才进入相应闭环模式。底盘与云台各自持有控制线程，发射由云台线程同拍调用，CAN3的yaw和拨弹共用0x1FF完整组帧。

配置沿用已迁移模板的总线、ID、PID、方向及安装矩阵。本次启用整车构建，不代表已经实测这些参数与实车安装相符，也不代表已进行烧录、设备收发或机构闭环验收。

构建后可运行 `python tests/external_modules/run_tests.py --preset Debug`（Release同理），确认当前ELF包含底盘、功率、双Yaw、Pitch、云台和发射的实际控制符号，并检查裁判协议布局。

云台结构：完整配置包含23个实例。DualYaw/Pitch已收进Gimbal内部，YAML只创建一个云台实例，注入三台电机；所有轴参数归入 `gimbal.param.yaw` 和 `gimbal.param.pitch`。校准数据库入口为 `gimbal.calibration_database`。
