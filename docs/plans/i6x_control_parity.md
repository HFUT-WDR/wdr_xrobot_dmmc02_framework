# I6x 控制切换与 26_sentry 对齐修改规划

状态：本地实现已完成（2026-11-08），验证结果见文末；未进行实车验证。

工程：`/home/yunmo/WorkingSpace/STM32_Project/Xrobot/wdr_xrobot_dmmc02_framework`

参考：`/home/yunmo/WorkingSpace/STM32_Project/rm_26_sentry/26-sentry_-dm`

## 目标与已定边界

补齐 OP/AUTO 切换时的发射复位，禁止本车上位机控制底盘 wz，明确无力与校准的优先级，并恢复原车 OP 的拨杆操作手法。

- I6x 解码与事件值保持上拨 +1、中位 0、下拨 -1，不互换 UP/DOWN。
- SW 编号从 0 开始；SW1 控制摩擦轮开关，SW0 请求拨弹。
- CALI入口按用户2026-11-08确认恢复为原车无力档下的两阶段摇杆手势，移除拨杆快捷组合。
- 保留目前的发射状态机；已合并的 SHOOT_READY_BULLET 不恢复。
- 保留 STOP/AIM/SCAN 三种自动行为，不恢复旧导航、过点或 AUTO_TEST。
- 保留启动和失联恢复首帧只建立摩擦轮边沿基准的约定。
- 无有效 AUTO 云台目标时保持无力；恢复 AIM 后按现有约定重新起转。
- 不改变电机总线、ID、方向、IMU 安装、PID、滤波及机械参数。
- 完成整套修改后统一验证整车 Debug/Release，不安排逐模块烧录。

## 已定校准入口与旋转来源

CALI入口已确认：SW2实际上位（无力）时，通过原26_sentry的两阶段摇杆手势申请双Yaw校准，不采用SW2中位或SW1/SW3快捷组合。

按用户后续“执行规划”的指令，AUTO的wz使用规划默认的遥控旋钮来源，范围沿用0～12 rad/s。这是对本车的简化，不复制原车根据受击、距离、比赛阶段等条件决定自动转速的完整策略。

当前实现采用上述校准入口和旋转来源。

## 1. 补齐 OP/AUTO 切换时的发射复位

修改 `Modules/HFUT-WDR/Gimbal/gimbal.cpp`、`Modules/HFUT-WDR/Launcher/Launcher.hpp` 和 `launcher.cpp`。

原车依据：`gimbal_task.c::gimbal_mode_change_control_transit` 在进入、退出 AUTO 时，将发射模式重置为 SHOOT_STOP，并清除 open_fric。

- 在 Gimbal 实际模式进入或离开 AUTO 的边沿，同拍显式复位 Launcher，不依赖切源期间恰好出现一拍 RELAX。
- 提供由控制线程调用的复位入口；不经会被 AUTO 分支忽略的 FRIC_OFF 事件间接复位。
- 发射模式置 SHOOT_STOP，清除待消费的摩擦轮操作、拨弹 PID 和卡弹计时；摩擦轮 PID 按原车停止转换清理。
- 保留当前转速反馈、热量、发射计数和弹速管理结果；摩擦轮沿用速度环刹停，不能把所有状态整体清零。
- 退出 AUTO 后，即使 SW0 仍下拨，也必须重新操作 SW1 才能启动摩擦轮。
- 进入 AUTO 且 AIM 有效时，从 READY_FRIC 重新起转，不继承 OP 的到速、击发或连发阶段。
- 复位应覆盖本次模式切换之前暂存的操作，避免同一 I6x 帧中的旧事件跨控制来源开启发射。

## 2. 本车 AUTO 不接受上位机 wz

修改 `Modules/HFUT-WDR/CMD/CMD.hpp`，同步机器人 YAML 与模板。

- 在命令来源选择处实现：AUTO 的 x/y 来自有效的 PC 底盘通道，z 来自本车配置指定的本地来源。
- 通用 CMD 通过明确的旋转来源配置支持该选择；本车配置为 RC，其他已有接入保持原默认行为，避免顺带改变 DT7/DR16 的 AUTO 行为。
- 不只在 HostData 中把 w 置零：那会让 AUTO 丢失本地旋转，而且无法覆盖其他 FeedAI 调用者。
- HostChassisTarget 的既有三浮点布局保留；本车选用 RC 旋转来源时，上位机 w 不参与最终命令。
- PC 平移通道超时则 x/y 归零，本地 z 仍按旋钮给出；整车是否允许输出仍由遥控在线、云台许可和电机在线判断。
- 遥控失联、云台 STOP/普通无力/CALI 时，底盘停止联锁优先于本地 z；进入CALI只允许校准所需的Yaw驱动力。

## 3. 恢复原车无力档下的摇杆校准手势

修改 `Modules/HFUT-WDR/I6x` 内的手势识别与事件入口、`Modules/HFUT-WDR/Gimbal` 内的校准申请处理、`User/RobotConfig/sentry_remote.yaml` 和 `config/templates/gimbal.yaml`。

原车依据为 `calibrate_task.c::RC_cmd_to_calibrate` 和 `calibrate_task.h`。注意原工程I6x分支的switch_is_up/down命名与实际方向相反。

校准申请的就绪条件：遥控有效在线，SW2实际上位（+1，无力），SW0实际下位（-1）。不增加SW1或SW3位置限制。

下面的ch[0..3]均指完成原车映射和通道交换后的±660通道值，不是SBUS原始字节：门限统一由 `I6X_CALIBRATION_GESTURE_THRESHOLD` 定义，当前为550，以适配实车ch[3]内扣约-598的行程。

| 阶段 | ch[0] | ch[1] | ch[2] | ch[3] | 行为 |
| --- | --- | --- | --- | --- | --- |
| 进入校准选择 | < -550 | < -550 | > 550 | < -550 | 连续保持超过2000ms，进入校准选择窗口 |
| 确认云台校准 | > 550 | > 550 | < -550 | > 550 | 在选择窗口内连续保持超过2000ms，提交一次双Yaw校准申请 |

- 校准选择窗口从第一阶段完成开始计时，严格超过20000ms后失效，重新执行完整两阶段手势。
- 任一阶段未持续满足其手势时，清除该阶段的连续保持计时；遥控失联时清除未完成手势，不能把离线时间计入保持时长。
- 用单调时间计算保持时长，不能把每个SBUS新帧当成1ms；原车校准任务标称周期为1ms，而当前I6x按新帧处理事件。
- 手势确认只发布一次申请，不按SBUS帧连续请求；不顺带迁入陀螺仪校准或底盘电机ID重置手势。
- 删除实际配置和模板中的SW1_DOWN＋SW3_UP到CALI的绑定；保留旧组合事件编号和物理含义供兼容，不能让旧快捷方式仍能触发校准。
- Gimbal区分普通模式请求与一次性校准申请：普通SW2_UP保持事件继续表示RELAX，但经完整手势确认后允许从RELAX进入CALI，不能被下一帧同一SW2_UP保持事件打断。
- 校准实际开始后，由现有CALI状态保持扫描至完成，释放摇杆不终止扫描；已在上位的SW2也不构成退出条件。校准申请是普通无力的明确例外，不能继续使用此前“SW2上位始终停止Yaw”的规划。
- 完成后消费本次申请，不因手势或拨杆保持重复扫描；保持SW2上位则回到RELAX，之后按正常拨杆切换恢复工作。
- 校准期间Pitch保留使能、输出零刚度/零阻尼/零力矩MIT，底盘和发射停止；沿用现有失联、电机故障保护及标定算法，不另增一套校准控制逻辑。

## 4. 恢复原车 OP 的 SW1 拨弹许可

修改 `Modules/HFUT-WDR/I6x/I6x.hpp::MakeCommand` 及相关测试。

- SW1 切入实际下位：切换摩擦轮开关。
- SW1 回到实际上位，且 SW0 实际下位：产生 OP 拨弹意图。
- 输入侧只表达拨杆组合；摩擦轮是否开启、是否到速、热量与电机在线仍由 Launcher 判断。
- 不增加新的摩擦轮开启锁存，不把机构运行状态放进 I6x。
- AUTO 使用 AI 开火命令，不叠加该 OP 拨杆组合限制。

原车的 switch_is_down 在 I6x 分支对应实际上位；实现和文档必须按实际方向表达，不能照抄宏名称。

## 5. 配置、文档与验证

同步整车配置、底盘/云台模板、I6x 与 Launcher README、迁移说明和 `docs/sentry_remote.md`。旧规划中与本次方案冲突的 CALI 优先级说明应注明被本规划取代，不继续保留互相矛盾的现行说明。

测试必须读取实际 YAML 绑定，不能只验证手写的另一份绑定。冻结的 26_sentry 参考文件不修改；明确区分原车行为、已经批准的简化以及本规划提出的调整。

重点覆盖：

- OP/AUTO 双向切换，包含 SW0 保持下拨、同帧 SW1 操作、摩擦轮已到速和正在连发。
- 模式复位清掉旧操作，退出 AUTO 后不会自动开启摩擦轮或拨弹。
- AUTO 中 PC w 取正、负、零均不能改变本地 z；旋钮两端与中位正确，PC 平移超时只撤销平移。
- SW2上位、摇杆未形成完整手势时，枚举其他拨杆组合均不能触发CALI；旧快捷组合不再触发。
- 对照原车两阶段手势、SW0/SW2实际方向、统一宏门限±550的严格比较、超过2000ms的连续保持及超过20000ms的窗口超时。
- 按真实SBUS帧间隔验证计时；手势中断、失联、恢复、未解锁直接确认均不能误触发。
- 确认后保持SW2上位、释放摇杆，CALI仍可完成；正常无力保持事件不能覆盖已接受的校准申请。
- CALI完成后保持原输入不重复扫描，重新申请须走完整手势；整车配置和模板绑定一致。
- OP 摩擦轮下拨开启、上拨准备、SW0 下拨射击；AUTO 不受 OP 的 SW1/SW0 组合限制。
- 保留解码对照、拨杆组合切换、热量、卡弹、滤波与发射状态回归。

验证命令：

```sh
python tests/i6x/run_tests.py
python tests/launcher/run_tests.py
python tests/gimbal/run_tests.py
python tests/chassis/run_tests.py
xrobot gen -c User/RobotConfig/sentry_remote.yaml
cmake --preset Debug
cmake --build --preset Debug -j4
cmake --preset Release
cmake --build --preset Release -j4
python tests/external_modules/run_tests.py --preset Debug
python tests/external_modules/run_tests.py --preset Release
```

交付时列出文件、最终操作表、验证结果和未验证项。保留当前各仓库未提交修改；实施前记录主工程及各模块状态，避免覆盖 DeepSeek 或其他人的工作。模块版本发布后，再同步主工程锁文件与离线 bundle，并验证冻结离线恢复；当前未提交的模块不能直接通过 setup 覆盖重建。

## 已知独立问题

此前确认的“Flash 保存前 Yaw 零输出未组成完整 CAN 帧”仍需单独修复。未修复前，不能宣称校准结束保存期间已保证零驱动力。本轮遥控规划不以构建或离线测试替代烧录、设备收发和机构闭环验证。

## 本地实施结果（2026-11-08）

- 已实现本规划中的切源复位、本地旋转来源、原车两阶段校准手势、OP拨弹位置许可及配置/文档同步。
- I6x、Launcher、Gimbal、Chassis离线回归均通过；已验证实际YAML/模板绑定、手势计时、带发射机构的完整校准流程，以及同帧旧操作不会跨来源开启发射。
- 整车Debug/Release配置、构建、模块构造检查、实际链接符号与裁判布局检查均通过。
- 未新增Inf/NaN数值检查。原有未提交修改保留；本轮源码改动仍位于CMD、I6x、Gimbal、Launcher子仓库工作树，尚未提交/推送，锁文件和离线bundle未更新到本轮。
- 未烧录，未进行设备收发或机构闭环验证。上述Flash组帧待办未在本轮修改。
- 详细验证范围见[验证记录](../validation/i6x_control_parity_20261108.json)。
