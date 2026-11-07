# 双 yaw + pitch 云台模块移植

## 已确认需求

- 2026-11-07统一为一个 `Gimbal` XRobot 模块；`DualYaw`、`Pitch` 为其 `components/` 下的内部组件，由Gimbal按值持有。YAML直接注入三台电机，参数收进 `gimbal.param.yaw`、`gimbal.param.pitch`。
- 只有 `Gimbal` 创建控制线程，标称 1 ms；控制链同拍共用实际 dt。
- 上电保持 RELAX。构造阶段不发送电机控制或使能帧。
- 故障按拍判定、不锁定（2026-11-07按26_sentry赛场行为修订）：遥控断连、AI失效、自瞄STOP、任一yaw掉线、异常拍只令本拍无力；条件恢复后回到当前请求的模式并重新捕获目标。比赛开始后无法操作遥控器，阵亡断电复活后必须自行恢复。
- 模式跟随拨杆电平：模板绑定 I6x 保持事件（`SwitchHold`/`ChordHold`），每个有效帧按当前档位重发；边沿事件保留给开关类动作。
- Pitch 掉线或失能时按周期重发使能、不下发 MIT 控制，yaw 照常闭环（同原车）。
- 覆盖归中、校准、绝对角遥控、相对角、静止及AUTO；AUTO仅保留STOP/AIM/SCAN。
- 删除AUTO_TEST及DETECT/TRACK/CONFIRM搜寻状态机；遥控Pitch路径命名为REMOTE_CONTROL，自瞄力矩路径命名为AIM。
- AI导航仅控制底盘速度，速度在大yaw坐标系表达；不提供AI大yaw角度目标或导航云台模式。
- 校准的目的为标定编码器相对角零点。端点扫描是求零点和限位的手段，不调用电机设零命令，不把 IMU 航向归零作为编码器校准。

## 模块边界与调用

`Gimbal`：读命令/姿态快照 → 更新两个控制模块反馈 → 选择模式/处理模式切换 → 生成目标 → 调用双 yaw 与 pitch 控制 → 更新发射机构 → 发送 → 发布状态。

`DualYaw` 持有两台 RMMotor 的引用，处理机械相对角、双 IMU 航向对齐、pitch 下的 yaw 角速度投影、大 yaw 优先/小 yaw 优先规划、扫描速度和串级控制。按原车保留 INNER_YAW/OUTER_YAW 编号，前者是底盘所用机械角，后者参与小 yaw 机械限位。Pitch 的实际机械角由 Gimbal 显式传入，不互相读取控制类内部状态。

`Pitch` 持有 DMMotor 引用，处理编码器/DM IMU 反馈、MIT 目标、TD、跟踪力矩与 J/B/G/摩擦前馈、扫描。手动模式按原车实际发送最终编码器目标和 TD 速度，发送的附加力矩为零；不将原车计算后未使用的手动力矩误认为已参与输出。

调度阶段与底盘统一命名：`FeedbackUpdate`读取输入与反馈，`SetMode`检查故障并处理启动/切源等待，`ModeTransition`只处理模式边沿，`SetControl`生成目标，`ControlLoop`计算闭环，`SendOutput`发送，`PublishStatus`发布状态。拍内到达的模式请求下一拍生效，标称周期为1 ms，实际调度延迟需上板测量。校准和归中完成拍跳过普通闭环。

两个内部控制组件不创建线程、不重复订阅 CMD。Gimbal callback 仅提交原子模式请求，控制线程只读该请求、从不改写。INIT/CALI 结束后记录本次请求：请求不变期间 INIT 按当前命令来源映射为 AUTO 或绝对角，CALI 保持 RELAX，不重复扫描端点；标定被故障打断即结束本次请求并置 `calibration_failed`，不在恢复后自动重新驱动到端点。

IMU 首帧到达前 `waiting_for_enable` 置位并保持无力。命令来源与模式不匹配时 `waiting_for_source` 置位、本拍无力，来源匹配即继续，不设等待时限。CALI独立驱动yaw并保持Pitch失能。

## 命令与失效

`CMD::GimbalCMD` 追加来源、有效性、AI 原始接收时间和自动行为字段。

- OP：yaw/pit 为原有归一化操纵量；Gimbal 按 YAML 中的 rad/s 量程映射并用 dt 积分。
- AUTO：yaw/pit 为 rad，导数分别为 rad/s、rad/s²；不提供AI大yaw导航角输入。
- `valid=false` 或 AI 原始时间戳达到 100 ms，即无力；重复发布不会刷新 AI 接收时间。
- 零角度消息可有效，不能用全零值代替失效状态。
- 自动行为为STOP=0、AIM=1、SCAN=4。STOP本拍无力，视觉切回AIM/SCAN即恢复。旧导航值2/3/5视为非法输入、本拍无力，避免将旧导航指令误解为扫描。
- ASync 仍捕获重新等待后的第一帧。模式切源时若尚持有旧来源快照，保持无力直到收到匹配来源。

`HostData::HostGimbalTarget` 保留原有九浮点、36字节结构和 `target_euler` Topic。普通旧目标消息默认AIM。扩展消息 `HostData::HostGimbalCommand` 从独立 `host_gimbal_command` Topic 输入，仅包含旧目标和自动行为；删除big_yaw、force_big_yaw、target_found。扩展结构布局发生变化，通信两端须同步更新；不把新结构发到旧Topic。新消息不自动改变全局OP/AUTO。

AIM使用上位机小yaw瞄准目标，由DualYaw本地拖拽策略协调大yaw；SCAN使用本地大yaw扫描速度及Pitch扫描轨迹。两者都能与底盘导航并行。HostData的 `host_chassis_data` 通道独立接收vx/vy/w，显式标记为大yaw坐标系，不改变云台目标或控制路径；导航报文也不会刷新云台AI指令时效。

Pitch的三个角度分别为：编码器位置、机械相对角（编码器减zero）、DM惯性角。INIT复用RELATIVE路径保持机械相对零点，MIT位置为zero，速度字段为 `init_pitch_velocity=0.5 rad/s`（不是速度上限），kp/kd沿用Pitch的MIT配置，附加力矩为零。底座倾斜不改变机械归中目标。

AIM/SCAN共用惯性到机械目标约束：先限惯性角，再用“当前机械角＋目标惯性角－当前惯性角”转换并限机械角。不可同时满足两类范围时优先机械限位。SCAN下发MIT位置时加回zero；AIM仍使用力矩控制。`requested_target`记录限位前惯性目标，`target`为实际可达惯性目标，`relative_target`为对应机械目标。目标被截断后撤销运动速度/加速度前馈，SCAN保留重力补偿。此处使用已对齐正方向的编码器与IMU，真实安装方向仍需机构验证。

## 电机协议与实际参数

原车路径：`rm_26_sentry/26-sentry_-dm/User/application/task/gimbal`，参数来自 `User_Config/gimbal_params.h`。

| 对象 | 原车配置与本次接口 |
|---|---|
| yaw | CAN3；反馈 0x205/0x206；GM6020 电压控制 0x1FF，协议量 ±30000 |
| GM6020 电流模式 | 保留原默认 0x1FE/0x2FE、±16384；与电压控制显式区分 |
| pitch | CAN3；命令 0x03，反馈 0x13；DM4310，位置量程 ±12.5 rad |
| pitch 旧默认 | DMMotor 未指定 position_max 时沿用原驱动型号量程，保持其他模块行为 |
| 相对角零点 | yaw 的 offset_ecd 是电机逻辑方向、8192 计数单圈编码器偏置；Pitch 的 zero 单位 rad |
| 时间/方向 | 输出角度 rad，角速度 rad/s；方向由电机 reverse 和 IMU installation 共同确定 |

`RMMotor::SetVoltage()` 接收协议整数，不能将电压量通过归一化电流接口传入；电压模式不接受 MODE_CURRENT/MODE_TORQUE。其 Relax 为零协议输出。DMMotor 的 Relax 发送 0xFD 失能，不能以 MIT 零力矩代替。

修正 DMMotor 原 16 位位置原码的有符号转换，避免半圈分界后出现错误负位置。真实 CAN 接收记录时效；错误长度/ID 不刷新在线状态。Pitch失能或掉线期间限频重发使能帧，不发送MIT控制，yaw仍可闭环；任一yaw掉线时本拍整云台无力，反馈恢复后自动继续。

## 标定结果与存储

CALI 先扫描 OUTER_YAW 正/反端点，再扫描 INNER_YAW 正/反端点。仅单轴在线时跳过另一轴，保留其既有校准参数。Pitch 保持无力。

端点低角速度持续超过 2 s 后记录编码器值。按单圈正向弧的中点求 offset_ecd，计算相对角上下限，再各留 0.1 rad 余量。端点重合/跨度不足不覆盖原参数。每步 15 s 超时退出，避免一直顶住机械端点；这是新增的关键校准超时，不声称与旧无限等待逐拍等价。

完成后先提交零输出，再应用标定并回到 RELAX。`gimbal_status.calibration` 导出两轴偏置/限位；`calibration_complete` 和 `calibration_saved` 分别表示完成、存储成功。

Gimbal 可通过 `calibration_database` 注入 `LibXR::Database*`，以 `gimbal_yaw_calibration` 为默认键加载/保存结果。数据库必须比模块存活更久；Flash 后端写入只在已停止输出后执行。`database=nullptr` 时结果只在 RAM，复位后使用 YAML，不能声称掉电保存成功。当前模板保留此默认值，尚未接入实际 Flash 后端。

Yaw编码器零位通过CALI获取，模板初始偏置不视为校准结果。Pitch使用DM4310电机提供的零位，软件zero=0，不额外要求Pitch机械置零。整车配置与校准操作见[整车遥控构建](sentry_remote.md)。

## 对外联动与组合

`gimbal_chassis_feedback` 发布实际 INNER_YAW 相对角、时间戳、序号和 `stop_chassis` 电平许可，使用从 Chassis 控制头拆出的同一消息类型。

停止许可为电平：RELAX/INIT/CALI/MOTIONLESS 置 `stop_chassis`，RELAX/INIT/CALI 置 `stop_launcher`（位于 `gimbal_status`）。底盘每拍读取并作为 `faults.gimbal_stop` 无力，撤销后自动继续；不再用事件改写底盘模式请求，避免与每帧重发的拨杆保持事件互相覆盖。可选 `Launcher` 由Gimbal在同一线程内调用，运行许可直接取本拍 `stop_launcher`，不增加独立发射线程。

当前增量模板已接入两台摩擦轮和一台拨弹电机，Gimbal的 `launcher` 参数绑定该实例。CAN3的两台yaw（0x205/0x206）和拨弹（0x207）共用0x1FF组帧，均在Gimbal线程同拍更新后发送；CAN2摩擦轮单独组帧。主机组合测试检查每拍仅一帧0x1FF及拨弹槽位。构造阶段不发送，默认板级YAML仍不实例化这些设备。

`config/templates/gimbal.yaml` 为增量模板，默认 User/xrobot.yaml 保持板级配置。模板接入 I6x SW2 中位→OP/手动、上位→AUTO、下位→RELAX，均使用保持事件；首次上线和链路恢复后按当前拨杆档位直接给出模式。INIT/CALI 等操作通过显式模式事件触发，模板不绑定会误触发的校准手势。

比赛阶段/剩余时间订阅 `robot_game_ref`；敌方前哨站状态通过 `Gimbal::Context` Topic 注入，当前 Referee 摘要尚不提供该字段。未注入时不声称完成实车比赛状态联动。原车编译单元中实际使用的扫描参数以 pitch_ctrl/dual_yaw_ctrl 可见的参数为准，例如 yaw 常速 1.5 rad/s，不能误取 gimbal_task.c 中另一个编译单元的局部宏。

## 参考差异与验证边界

- 框架与订阅/事件方式参考 QDU；双 yaw 和 pitch 算法来自本地 26_sentry，未用 QDU 普通两轴闭环替代。
- 云台角度 PID 使用原车显式角速度 D、积分分离及速度环回算语义，独立于现有底盘公共 PID；1 ms 下有冻结代码逐步对照，变 dt 的积分/差分按原标称周期换算。
- 反馈统一在周期入口采集；与旧任务“模式→切换→反馈”存在一拍时序差异。
- 上电无力、DM 明确失能为已确认行为。故障恢复语义与原车一致（按拍无力、恢复自动继续、Pitch 离线重发使能），2026-11-07 起取代早期的锁定 RELAX 方案。
- 原车RELATIVE/MOTIONLESS的pitch输出分支未明确发送，本次提供显式MIT相对位置保持；此处不能声称发送序列等价。
- 按本次确认删去AUTO_TEST状态机和导航/过点云台模式；原车导航Pitch归中分支随模式一起删除。底盘导航与云台AIM/SCAN独立，AI不直接控制大yaw。
- Pitch扫描将原车直接发送惯性角的位置输出改为编码器坐标，AIM/SCAN新增统一机械可达目标约束；不声称限位场景与冻结原车逐拍等价。未触限位的力矩算法仍对照冻结参考。
- 原车关闭的识别实验、ESO、保护宏及未参与实际输出的中间补偿，不作为已经开启的功能复制。自动开火命令经CMD传递给Launcher，具体发射状态机和与原车的差异见[Launcher说明](https://github.com/HFUT-WDR/Launcher/blob/dev/README.md)。
- 主机测试、构建、完整组合链接仅证明软件范围。CAN 收发、模式切换瞬态、标定端点受力、实际闭环、1 ms 截止时间及掉电存储均需分别实测；本次未烧录或驱动机构。

验证入口：`python3 tests/gimbal/run_tests.py`；`python3 tests/gimbal/run_firmware_link.py`；`python3 tools/check_module_snapshots.py`；Debug/Release 预设构建。冻结参考及 SHA-256 位于 tests/gimbal/reference。

组合链接另可执行 `python3 tests/gimbal/run_firmware_link.py --combined`，在临时工程同时实例化底盘、功率、云台和发射机构，停底盘许可经 `gimbal_chassis_feedback` 传递。当前软件验证记录见 [验证结果](validation/gimbal_modules_20261106.json)。默认板级 Debug/Release ELF 已检查不包含云台控制符号。

## 发射机构接入与验证入口（2026-11-07）

OP摩擦轮开关使用I6x边沿事件SW1_UP，保持事件仅用于模式跟随；AUTO按AIM/STOP/SCAN决定摩擦轮请求。开火命令来自 `launcher_cmd`，RELAX/INIT/CALI不许可发射，MOTIONLESS仍可许可。撤销许可时拨弹立停，摩擦轮闭环降速至阈值后撤电流。参数、卡弹反转、热量及弹速管理见模块README。

`python3 tests/launcher/run_tests.py` 覆盖行为、热量、弹速、卡弹和滤波，并与冻结26_sentry源码逐拍对照；云台边界测试覆盖发射联锁和同拍组帧。测试不代表实际发射、弹速、热量或机构闭环验证。

本次框架同步的验证结果见[2026-11-07验证记录](validation/framework_sync_20261107.json)。
