# 双 yaw + pitch 云台模块移植

## 已确认需求

- `DualYaw`、`Pitch`、`Gimbal` 分别为 XRobot 模块，YAML 创建对象并注入引用。
- 只有 `Gimbal` 创建控制线程，标称 1 ms；控制链同拍共用实际 dt。
- 上电保持 RELAX。构造阶段不发送电机控制或使能帧。
- AUTO 的 AI 指令失效时整套云台锁定 RELAX；重新收到消息不会自动恢复。
- 遥控断连、恢复都锁定 RELAX，必须明确操作模式才能运行。
- 覆盖归中、校准、绝对角遥控、相对角、静止及AUTO；AUTO仅保留STOP/AIM/SCAN。
- 删除AUTO_TEST及DETECT/TRACK/CONFIRM搜寻状态机；遥控Pitch路径命名为REMOTE_CONTROL，自瞄力矩路径命名为AIM。
- AI导航仅控制底盘速度，速度在大yaw坐标系表达；不提供AI大yaw角度目标或导航云台模式。
- 校准的目的为标定编码器相对角零点。端点扫描是求零点和限位的手段，不调用电机设零命令，不把 IMU 航向归零作为编码器校准。

## 模块边界与调用

`Gimbal`：读命令/姿态快照 → 更新两个控制模块反馈 → 选择模式/处理模式切换 → 生成目标 → 调用双 yaw 与 pitch 控制 → 兑现 RELAX → 发送 → 发布状态。

`DualYaw` 持有两台 RMMotor 的引用，处理机械相对角、双 IMU 航向对齐、pitch 下的 yaw 角速度投影、大 yaw 优先/小 yaw 优先规划、扫描速度和串级控制。按原车保留 INNER_YAW/OUTER_YAW 编号，前者是底盘所用机械角，后者参与小 yaw 机械限位。Pitch 的实际机械角由 Gimbal 显式传入，不互相读取控制类内部状态。

`Pitch` 持有 DMMotor 引用，处理编码器/DM IMU 反馈、MIT 目标、TD、跟踪力矩与 J/B/G/摩擦前馈、扫描。手动模式按原车实际发送最终编码器目标和 TD 速度，发送的附加力矩为零；不将原车计算后未使用的手动力矩误认为已参与输出。

调度阶段与底盘统一命名：`FeedbackUpdate`读取输入与反馈，`SetMode`检查故障并处理启动/切源等待，`ModeTransition`只处理模式边沿，`SetControl`生成目标，`ControlLoop`计算闭环，`SendOutput`兑现迟到的RELAX并发送，`PublishStatus`发布状态。校准、等待和归中完成拍沿用原先的跳过普通闭环语义。

两个控制模块不创建线程、不重复订阅 CMD。Gimbal callback 仅提交原子模式请求，控制状态由一个任务持有。INIT完成通过比较交换提交后续模式，期间到达的RELAX/其他模式请求不能被内部跳转覆盖。

启动就绪只在一次运行周期内保持，普通模式变化不清除就绪。运行期先检查关键电机故障，再处理模式或来源变化；掉线直接锁定RELAX，不能进入首次使能重试。切源等待主动失能后才撤销就绪，来源匹配后重新准备；CALI独立驱动yaw并保持Pitch失能。

## 命令与失效

`CMD::GimbalCMD` 追加来源、有效性、AI 原始接收时间和自动行为字段。

- OP：yaw/pit 为原有归一化操纵量；Gimbal 按 YAML 中的 rad/s 量程映射并用 dt 积分。
- AUTO：yaw/pit 为 rad，导数分别为 rad/s、rad/s²；big_yaw 为导航/强制大 yaw 目标。
- `valid=false` 或 AI 原始时间戳达到 100 ms，即无力；重复发布不会刷新 AI 接收时间。
- 零角度消息可有效，不能用全零值代替失效状态。
- 自动行为为STOP=0、AIM=1、SCAN=4。保留已有扫描协议值，旧导航值2/3/5视为非法输入并锁定RELAX，避免将旧导航指令误解为扫描。
- ASync 仍捕获重新等待后的第一帧。模式切源时若尚持有旧来源快照，保持无力，最多等待 100 ms 获取匹配来源；已收到 AUTO 失效帧立即锁定 RELAX。

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

修正 DMMotor 原 16 位位置原码的有符号转换，避免半圈分界后出现错误负位置。真实 CAN 接收记录时效；错误长度/ID 不刷新在线状态。显式模式操作后才允许限频发送 DM 使能，等待使能期间 yaw 无力；已运行后的关键电机反馈掉线，整云台锁定 RELAX。

## 标定结果与存储

CALI 先扫描 OUTER_YAW 正/反端点，再扫描 INNER_YAW 正/反端点。仅单轴在线时跳过另一轴，保留其既有校准参数。Pitch 保持无力。

端点低角速度持续超过 2 s 后记录编码器值。按单圈正向弧的中点求 offset_ecd，计算相对角上下限，再各留 0.1 rad 余量。端点重合/跨度不足不覆盖原参数。每步 15 s 超时退出，避免一直顶住机械端点；这是新增的关键校准超时，不声称与旧无限等待逐拍等价。

完成后先提交零输出，再应用标定并回到 RELAX。`gimbal_status.calibration` 导出两轴偏置/限位；`calibration_complete` 和 `calibration_saved` 分别表示完成、存储成功。

DualYaw 可注入 `LibXR::Database*`，以 `gimbal_yaw_calibration` 为默认键加载/保存结果。数据库必须比模块存活更久；Flash 后端写入只在已停止输出后执行。`database=nullptr` 时结果只在 RAM，复位后使用 YAML，不能声称掉电保存成功。当前模板保留此默认值，尚未接入实际 Flash 后端。

模板零位 0 和限位是编译占位，不能当作该机构已验证标定。实际使用前从有效标定或原车保存数据填入 YAML，核对电机参数与物理限位。

## 对外联动与组合

`gimbal_chassis_feedback` 发布实际 INNER_YAW 相对角、时间戳和序号，使用从 Chassis 控制头拆出的同一消息类型，不改变既有底盘字段。

`GetOutputEvent()` 提供 STOP_CHASSIS / STOP_LAUNCHER：RELAX/INIT/CALI 要求两者停止，MOTIONLESS 要求底盘停止。限制状态每拍触发事件，接收方应只记录轻量模式请求，不打印高频日志。组合示例可通过 EventBinder 把 STOP_CHASSIS 绑定到 Chassis::RELAX。发射机构本身不在本次移植中，需接此停止事件及状态许可；不在云台模块里执行发射闭环。

同总线同一个 0x1FF 帧如还含拨弹电机，必须完整更新已注册槽位后提交。当前模板只注册两台 yaw，剩余槽位为零。后续接入原车同组拨弹时需要统一该帧的周期提交者，不得把两个任务异步缓存当作同拍输出验证。

`config/templates/gimbal.yaml` 为增量模板，默认 User/xrobot.yaml 保持板级配置。模板接入 I6x SW2 中位→OP/手动、上位→AUTO、下位→RELAX。I6x 首次上线和恢复只建立拨杆基准，不因当前拨杆位置自动启动；AI 恢复后也要明确改变模式。INIT/CALI 等操作通过显式模式事件触发，模板不绑定会误触发的校准手势。

比赛阶段/剩余时间订阅 `robot_game_ref`；敌方前哨站状态通过 `Gimbal::Context` Topic 注入，当前 Referee 摘要尚不提供该字段。未注入时不声称完成实车比赛状态联动。原车编译单元中实际使用的扫描参数以 pitch_ctrl/dual_yaw_ctrl 可见的参数为准，例如 yaw 常速 1.5 rad/s，不能误取 gimbal_task.c 中另一个编译单元的局部宏。

## 参考差异与验证边界

- 框架与订阅/事件方式参考 QDU；双 yaw 和 pitch 算法来自本地 26_sentry，未用 QDU 普通两轴闭环替代。
- 云台角度 PID 使用原车显式角速度 D、积分分离及速度环回算语义，独立于现有底盘公共 PID；1 ms 下有冻结代码逐步对照，变 dt 的积分/差分按原标称周期换算。
- 反馈统一在周期入口采集；与旧任务“模式→切换→反馈”存在一拍时序差异。
- 上电无力、AI 失效锁定、恢复不自启、DM 明确失能为已确认行为；不沿用原车离线时反复使能的输出路径。
- 原车RELATIVE/MOTIONLESS的pitch输出分支未明确发送，本次提供显式MIT相对位置保持；此处不能声称发送序列等价。
- 按本次确认删去AUTO_TEST状态机和导航/过点云台模式；原车导航Pitch归中分支随模式一起删除。底盘导航与云台AIM/SCAN独立，AI不直接控制大yaw。
- Pitch扫描将原车直接发送惯性角的位置输出改为编码器坐标，AIM/SCAN新增统一机械可达目标约束；不声称限位场景与冻结原车逐拍等价。未触限位的力矩算法仍对照冻结参考。
- 原车关闭的识别实验、ESO、保护宏及未参与实际输出的中间补偿，不作为已经开启的功能复制。自动开火门限与发射控制不属于本轮云台闭环。
- 主机测试、构建、完整组合链接仅证明软件范围。CAN 收发、模式切换瞬态、标定端点受力、实际闭环、1 ms 截止时间及掉电存储均需分别实测；本次未烧录或驱动机构。

验证入口：`python3 tests/gimbal/run_tests.py`；`python3 tests/gimbal/run_firmware_link.py`；`python3 tools/check_module_snapshots.py`；Debug/Release 预设构建。冻结参考及 SHA-256 位于 tests/gimbal/reference。

组合链接另可执行 `python3 tests/gimbal/run_firmware_link.py --combined`，在临时工程同时实例化底盘、功率、云台并绑定 STOP_CHASSIS→RELAX。当前软件验证记录见 [验证结果](validation/gimbal_modules_20261106.json)。默认板级 Debug/Release ELF 已检查不包含云台控制符号。
