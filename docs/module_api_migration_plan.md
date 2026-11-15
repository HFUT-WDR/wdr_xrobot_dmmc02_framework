# WDR 模块 API 逐项改造方案

状态：P0/P1 七模块已获准并完成本地改造；P2 保持待实施。日期：2026-11-15。配套规则：[模块开发规范](module_development_standard.md)。

以下调查表保留实施前接口作为迁移基线，并非当前新接口说明。当前接口、软件验证及跨工作区待应用补丁见[实施记录](validation/module_api_p0_p1_20261115.md)。ESKF 已按用户后续要求接入实际步长，早期“保持固定1ms”的候选不再采用。

## 1. 本次范围与依据

- 交付开发规范、21 个索引模块的改造方案和参考依据；不修改技能、模块实现、YAML、生成代码或锁文件。
- 范围来自当前 ModuleIndex/index.yaml 的 21 个条目；组织内未入该索引的 ImuEkfFusion、主机应用及 CI 仓库不隐含纳入。
- 已读取官方索引全部 30 个模块的主头文件，以及 WDR 全部 21 个模块的主头文件和仓库根目录清单。该覆盖不等于已审计所有 .cpp、协议实现、README、CI 和生产调用者。
- WDR 读取先用连接器，因私有仓库返回 404，随后用本机已认证的 gh 只读访问成功。基线保存在文末提交表；不将 404 判断为仓库不存在。
- 实施前仍需按实际基线提交补齐调用者、测试、配置清单；当前方案不声称已经遍历全组织所有 C++ 调用处。

## 2. 已发现的共性问题

| 问题 | 实际例子 | 处理原则 |
| --- | --- | --- |
| 配置穿插在对象依赖前后 | AttitudeEkf、ESKF、DmImu、Chassis、Gimbal | 对象依赖在前，配置在后；复杂配置集中，保留有效值 |
| 两套配置入口并存 | Gimbal、Launcher 的机构 Param 与外层 Topic；PowerControl 的算法、超电配置与 Topic | 新接口集中实例配置；不改控制/发送顺序 |
| 默认值可见性不同 | 官方具名默认实参、WDR 字段默认成员、无默认的 Param 混用 | 以生成器可解析且能完整生成示例为准，不只追求 C++ 简短 |
| “参数多”与“覆盖默认”混淆 | ESKF | 算法调参表区分默认和覆盖；参数数量不作为正确性判据 |
| 分支与工程版本不同 | ESKF 默认 main 是旧长参数构造，本工程锁定 dev 为 Param 构造 | 以本工程锁定 dev 作为现有接口基线，main 单独记载 |

### 当前 ESKF 的两项独立问题

1. 工程 YAML 只列 3 个 Param 字段，但 XRobot 1.0.1 要求完整映射。在独立临时副本中运行 `xrobot gen -c User/xrobot.yaml`，返回缺少 18 个字段，未改变原工程配置和生成头。不能将“C++ 字段有默认值”解释成“该 YAML 已能生成并使用默认值”。
2. dev 模块暴露 dt_source、nominal_period_s 和 SampleDt()，但 Run() 直接调用 Filter::Update(Sample)，核心仍以固定 DT=0.001f 积分；SampleDt() 未接入。接口表达了实际未提供的可变步长能力。修正步长会影响预测、噪声离散化、滤波及样本计数含义，必须独立做行为方案，不能夹带在 API 整理中。

现有生成头也未实例化新增 ESKF。本提案不修复上述问题、不声明新固件已经运行。

## 3. 逐模块方案

以下 21 个模块均须按[模块文档约定](module_documentation.md)补齐 README 内的使用教学。可实例化模块直接展示 module add、setup、instance add、完整实例 YAML、参数表和 gen 步骤；纯库展示添加、获取、include 与最小调用。example/examples 目录只能补充，不能替代 README 教学。此项覆盖已有接口无需重写的 P2 模块；当前交付仅更新要求和方案，尚未修改各模块 README。

优先级：P0 为先解决会误导生成或运行含义的契约；P1 为公开接口整理；P2 为基本合规、文档补齐。优先级不表示已经获准实施。以下目标签名均为设计示意，需生成与编译验证。

| 模块 | 当前接口/类型事实 | 拟改造与保留边界 | 调用方迁移与验证重点 |
| --- | --- | --- | --- |
| **AttitudeEkf · P1** | 四个 Topic 名、Param、RamFS、command_name；Param 仅步长来源、名义周期、栈 | 目标 `AttitudeEkf(RamFS&, const Param& = 具名默认值)`；把实例命名与任务配置集中。内部 Q/R 不因统一外观全部公开 | 当前框架 YAML、所有外置 IMU 实例及测试构造一起迁移；保持输入配对、MEASURED 行为、消息与命令不变 |
| **ESKF · P0** | 本工程 dev 为四个 Topic 名、Param、RamFS、command_name；main 仍为大量标量 | 以 dev 为基线，目标与 EKF 同类组织；保留全部算法参数与默认值。先记录固定 1 ms 与 dt 配置不符，独立决定移除虚假配置或实现可变步长 | 先用完整映射验证生成；后续 API 迁移需同输入对比、初始化/恢复/静止窗口检查。分支收敛与步长修复单独决策 |
| **DmImu · P1** | Param 在 CAN*/UART*/Database/RamFS 前；Topic、安装矩阵、命令名在后 | 依赖移前；CAN/UART 可继续显式传 nullptr，维持传输互斥。将采集配置、安装变换、命名集中为可解析配置，避免重复外层入口 | 覆盖 CAN_REQUEST、CAN_PUSH、RS485 与旧流入口；保留采集侧只变换一次、零偏键隔离、构造副作用契约。注释仍提 DmAttitude，应核对后更新 |
| **BMI088 · P2** | GPIO/SPI/PWM/Database/RamFS、可选 BuzzerAlarm*、具名默认 Param，已接近官方 | 保留接口骨架及蜂鸣器扩展；仅补可选对象、Topic 时戳、线程和标定行为说明；不强制改参数名与传感器量程 | 校验现有 YAML 与官方差异是有意覆盖；保留 1 kHz、量程、安装、温控和标定设置 |
| **BuzzerAlarm · P2** | PWM& 后三个默认标量，符合官方简单形式 | 保留标量接口；补构造约 800 ms 阻塞、致命错误回调与持续音所有权说明，不增加 Param 包装 | 保留开机曲、告警频率/时长、StartTone/StopTone 及现有音量 |
| **CMD · P2** | 无对象依赖，Mode、三个 Topic 名、auto_chassis_wz_source 均有默认 | 当前形式合规；保留公开枚举与命令类型。增长到复杂配置时再引入 Param，不为一致外观立即重写 | 必须保留 AI/遥控仲裁、模式、权限、wz 来源与事件含义；用真实消费方检查默认 Topic |
| **Chassis · P1** | 电机/CMD/PowerControl 后 Param，随后 RamFS 与长串 Topic/任务参数 | 先把 RamFS 移到配置之前；实例配置集中，保留既有控制参数类型的领域含义；不额外建立通用控制框架 | 真实整车 YAML、相关构造迁移；保留轮序、方向、PID、auto-spin、功率调用、故障门控与组帧发送 |
| **ControlAlgorithms · P2** | standalone:false，纯 PID/滤波库 | 保留纯库，不新增 YAML 实例或线程；完善参数单位、离散时间假设、Reset 对历史状态的含义 | 查其直接 C++ 消费者；参数搬迁不能改变离散公式、滤波初态与积分清零 |
| **DMMotor · P2** | CAN& 后具名默认 Param；依赖 Motor | 保留形状，补型号、限幅、ID、反向、构造/使能/发送及反馈时效契约 | 检查 DM4310/DM8009 真实调用；不改变 MIT 字段、CAN 帧或上电使能行为 |
| **DR16 · P2** | UART&、CMD& 后具名默认 Param | 保留接口；明确串口任务配置、遥控量程、失联与 CMD 事件；不要为了加命令引入 RamFS | 保留 DBUS 解码、事件触发与失控恢复；配置示例包含 CMD 前置实例 |
| **EventBinder · P2** | modules 列表及默认空 event_binding_groups；事件映射类型 | 保留 initializer_list 方案；补列表/引用的保存方式与生命周期、事件回调上下文 | 核对构造后是否保存列表引用再规定寿命；保留事件编号及一次触发语义，不改为状态快照 |
| **Gimbal · P1** | 电机与 CMD、Param，之后穿插 Database*、Launcher*、BuzzerAlarm* 及 Topic/栈/优先级 | 所有对象依赖移到配置前，可选指针显式 nullptr；实例配置集中，保留 DualYaw/Pitch 分域，不扁平化机构参数 | 迁移整车 YAML 和标定配置；保留校准时序、Flash 键、RELAX、模式/许可、与 Launcher 同拍控制与发送 |
| **HostData · P2** | CMD& 后多个默认 Topic 名和 telemetry | 当前顺序合规；保留接口，补连续目标、一次性发射、online 有效性和时效契约；复杂度增加时再考虑 Param | 保留 AI 仲裁输入，不把 online 改为链路连接，不擅自加过期清零或修改发射事件消费 |
| **I6x · P2** | UART&、CMD&、默认 topic_name 与 RCChassisParam | 保留当前简洁接口；明确外部类型默认值、SBUS 输入与失联规则 | 保留通道/开关/校准事件映射、失控输出、底盘参数取值 |
| **Launcher · P1** | 三电机依赖后无默认 Param 和三个默认 Topic；无独立线程 | 配置集中；机构 PID 等必须显式填写，不为了可默认实例化给出看似可直接上机的配置；保留同步组件形态 | 保留 Update 计算、Send 写入、Reset 边界及与 Gimbal 同拍共享 0x1FF 行为；不得新增独立发送任务 |
| **Motor · P2** | standalone:false，电机抽象接口库 | 保留抽象库，核对公共方法单位、错误和发送含义；不添加实例构造或强制所有电机统一协议 | 以 RMMotor、DMMotor 和机构调用为真实消费者；不在风格整理中改变虚函数或反馈布局 |
| **PowerControl · P1** | 默认 Param 后 cap_can*、cap_param、裁判 Topic；外层 Param 为别名 | 可选 CAN 对象移前；算法参数、超电配置、Topic 形成明确实例配置。别名类型先核对工具解析，不机械移除合法默认写法 | 保留无线程、Chassis 驱动、Calculate/Send 分离、能量状态与超电协议；CAN=nullptr 的原含义须保留 |
| **RMMotor · P2** | CAN& 后具名默认 Param；依赖 Motor | 保留接口；补组帧协作、feedback_id 与 GM6020 模式、发送触发和共享状态契约 | 保留同 CAN 组帧、反馈解析、CURRENT/VOLTAGE 协议和成员槽位；不为多实例“统一”重复发组帧 |
| **Referee · P2** | UART&、CMD* 后具名默认 Param | 保留骨架；注明 CMD* 可空且当前没有默认实参，不擅自改变必填性；Topic 后缀不单独批量重命名 | 保留 2025 协议数据布局、串口收发、UI/决策、副作用与裁判摘要；不得把规范化写成协议升级 |
| **HostLink · P2** | HostSerial& 后 enable_control=false、enable_ros=false；Linux 模块 | 保留平台依赖及简单开关形式；补时钟域、连接代次、Topic 与控制开关；不强加 MCU 外设类型 | Linux 后端验证，分别覆盖控制关闭/开启和 ROS 关闭/开启；默认控制关闭，不以 STM32 编译代替 |
| **rm_decision · P2** | 无对象依赖；文件路径、周期、端口等默认参数；Linux 行为树运行时 | 保留当前构造；类名不因 CamelCase 规则立即破坏性重命名，另列可选迁移。补文件解析、默认树、监视端口与任务说明 | 核对行为树、节点注册与 int Topic 消费者；保留空路径、端口 0 与已有默认值的实际含义 |

## 4. 建议实施批次与停止条件

1. **配置契约**：先单独处理 ESKF 完整映射和步长含义；当前只记录，不改。决定仍固定 1 ms 时也需显式迁移误导配置，不能保留同名空效果设置。
2. **采集与融合**：AttitudeEkf、ESKF、DmImu；BMI088/BuzzerAlarm 仅必要文档修订。先证明接口和有效值等价，再扩大构建。
3. **控制装配**：Chassis、Gimbal、Launcher、PowerControl，联动各自真实调用者。保留事件/控制/计算/发送边界。
4. **其余模块**：以补契约和示例为主；已有合规接口不重写。HostLink、rm_decision 采用 Linux 验证。

每批实施前补全生产用户和受影响配置；缺失消费者、默认值来源或控制副作用记录时，不进入该批实现。当前默认只覆盖本框架的已知配置，完整整车调用清单尚待实施阶段确认。

## 5. 实施验收清单（尚未执行）

- 模块头文件可由锁定工具解析，README 示例与新构造一一对应；解析通过不能代替 C++ 编译。
- 逐模块检查 README 本身具备完整使用教学、参数示例和参数表，不依赖 example/examples 目录才能首次接入；纯库明确不执行 instance add。
- 在隔离工程验证 instance add / sync / gen，核对每个有效参数与迁移前相同，不手改生成头。
- 检查同类模块的 Topic/命令多实例，以及可选依赖 nullptr 路径；库模块进行 include/消费者编译。
- 算法接口搬迁使用同输入回放，关注姿态、零偏、协方差和重置；步长修复另设预期，不能套用接口等价验收。
- 受影响 MCU BSP 的 Debug/Release 构建；Linux 模块采用对应后端和现有测试。
- 若后续改动触及设备收发、失控门控或机构时序，分别提供板上和机构证据，不用历史 CI 或编译冒充。

本次实际执行：源码与工具行为核对、隔离配置生成失败复现、文档覆盖与链接检查。未进行实现改造、构建、烧录、推送或 PR。

## 6. 本批已定决策与后续事项

- ESKF 按用户明确要求使用实测时间，保留与EKF相同的步长来源和钳制配置；预测、噪声离散化与低通同步接入。
- 已采用直接迁移，不保留旧构造重载；本工程配置已迁移，兄弟工作区的配置、测试与工具以已验证补丁交付，未修改其现有工作文件。
- Chassis/PowerControl 保留既有算法类型作为Param.control，DmImu保留采集类型作为Param.device；Gimbal/Launcher扩展现有Param。
- 七模块dev已提交推送，主框架锁文件已升级并通过冻结恢复；P2、发布分支合并和硬件验证仍不在已执行范围。

具体提交基线、补丁适用性、验证命令及软件证据见实施记录。

## 7. 源码基线

WDR 表中的提交来自读取时的默认分支；ESKF 另以本框架锁定的 dev 为主要基线，见表内注记。主头文件检查覆盖公开构造和 manifest，不代表全仓库行为审计。

| WDR 模块 | 已读版本 | 主头文件 |
| --- | --- | --- |
| AttitudeEkf | `278489d16f115ea27da00d061f4379d81d6c6022` | [AttitudeEkf.hpp](https://github.com/WDR-Xrobot-org/AttitudeEkf/blob/278489d16f115ea27da00d061f4379d81d6c6022/AttitudeEkf.hpp) |
| BMI088 | `1bfbc124cefb31559633bfc33b9e9289759d1880` | [BMI088.hpp](https://github.com/WDR-Xrobot-org/BMI088/blob/1bfbc124cefb31559633bfc33b9e9289759d1880/BMI088.hpp) |
| BuzzerAlarm | `a6f70132d657ae5d83dd6ebc1c0d6c4a68c71ab4` | [BuzzerAlarm.hpp](https://github.com/WDR-Xrobot-org/BuzzerAlarm/blob/a6f70132d657ae5d83dd6ebc1c0d6c4a68c71ab4/BuzzerAlarm.hpp) |
| CMD | `73f2bf4c97b3d37e169360f17e02a8926724f817` | [CMD.hpp](https://github.com/WDR-Xrobot-org/CMD/blob/73f2bf4c97b3d37e169360f17e02a8926724f817/CMD.hpp) |
| Chassis | `882251d6920c0a463e3069d6af6e8ab6adb09c9d` | [Chassis.hpp](https://github.com/WDR-Xrobot-org/Chassis/blob/882251d6920c0a463e3069d6af6e8ab6adb09c9d/Chassis.hpp) |
| ControlAlgorithms | `06eae1291ec5d756b947566b41287d557ef394bb` | [ControlAlgorithms.hpp](https://github.com/WDR-Xrobot-org/ControlAlgorithms/blob/06eae1291ec5d756b947566b41287d557ef394bb/ControlAlgorithms.hpp) |
| DMMotor | `3a93aa587d7a1747298d07a7ae6a30c116740729` | [DMMotor.hpp](https://github.com/WDR-Xrobot-org/DMMotor/blob/3a93aa587d7a1747298d07a7ae6a30c116740729/DMMotor.hpp) |
| DR16 | `8b91341c29b094e46aa656ba921d36afee690f19` | [DR16.hpp](https://github.com/WDR-Xrobot-org/DR16/blob/8b91341c29b094e46aa656ba921d36afee690f19/DR16.hpp) |
| DmImu | `386c06e8f6335966a0a9821ac53d3ee06d6dfddb` | [DmImu.hpp](https://github.com/WDR-Xrobot-org/DmImu/blob/386c06e8f6335966a0a9821ac53d3ee06d6dfddb/DmImu.hpp) |
| ESKF | `c157fec50799e52a663ff8f147bfa3addb794d02` | [ESKF.hpp](https://github.com/WDR-Xrobot-org/ESKF/blob/c157fec50799e52a663ff8f147bfa3addb794d02/ESKF.hpp) |
| EventBinder | `6698fd4d02f55b80e32ccece27e4e544d002584b` | [EventBinder.hpp](https://github.com/WDR-Xrobot-org/EventBinder/blob/6698fd4d02f55b80e32ccece27e4e544d002584b/EventBinder.hpp) |
| Gimbal | `cf77adeb383ca4add4780b6954812123e420bd96` | [Gimbal.hpp](https://github.com/WDR-Xrobot-org/Gimbal/blob/cf77adeb383ca4add4780b6954812123e420bd96/Gimbal.hpp) |
| HostData | `3bea9d36130ae03b40ced42fc7434fd80683d1b4` | [HostData.hpp](https://github.com/WDR-Xrobot-org/HostData/blob/3bea9d36130ae03b40ced42fc7434fd80683d1b4/HostData.hpp) |
| HostLink | `01a559fa6dc7a25ccd5b41475c7c623b88b69c02` | [HostLink.hpp](https://github.com/WDR-Xrobot-org/HostLink/blob/01a559fa6dc7a25ccd5b41475c7c623b88b69c02/HostLink.hpp) |
| I6x | `ddeaba83d432d1ad281f5b9383edb5e25fa8fc40` | [I6x.hpp](https://github.com/WDR-Xrobot-org/I6x/blob/ddeaba83d432d1ad281f5b9383edb5e25fa8fc40/I6x.hpp) |
| Launcher | `071971a8c47491997b0cc3ea60aead7215711f46` | [Launcher.hpp](https://github.com/WDR-Xrobot-org/Launcher/blob/071971a8c47491997b0cc3ea60aead7215711f46/Launcher.hpp) |
| Motor | `cf91a88d2414a0c2ffb05d5dd05e40cb742bb2bc` | [Motor.hpp](https://github.com/WDR-Xrobot-org/Motor/blob/cf91a88d2414a0c2ffb05d5dd05e40cb742bb2bc/Motor.hpp) |
| PowerControl | `9c1e0757fc283df5b1538b3b9fe380cc128e7cf3` | [PowerControl.hpp](https://github.com/WDR-Xrobot-org/PowerControl/blob/9c1e0757fc283df5b1538b3b9fe380cc128e7cf3/PowerControl.hpp) |
| RMMotor | `b8570cb05f554883ffbbbe81c30b7674da7f93d6` | [RMMotor.hpp](https://github.com/WDR-Xrobot-org/RMMotor/blob/b8570cb05f554883ffbbbe81c30b7674da7f93d6/RMMotor.hpp) |
| Referee | `91ed2ad8f758cbeacc55380a82b23c8bed48bbba` | [Referee.hpp](https://github.com/WDR-Xrobot-org/Referee/blob/91ed2ad8f758cbeacc55380a82b23c8bed48bbba/Referee.hpp) |
| rm_decision | `37c6caa87cca6504577ad8430b1d7a4d286b8b55` | [rm_decision.hpp](https://github.com/WDR-Xrobot-org/rm_decision/blob/37c6caa87cca6504577ad8430b1d7a4d286b8b55/rm_decision.hpp) |
| ESKF（工程锁定 dev） | `870106438e5030b759dba400149298742e8dc8e6` | [ESKF.hpp](https://github.com/WDR-Xrobot-org/ESKF/blob/870106438e5030b759dba400149298742e8dc8e6/ESKF.hpp)、eskf_core.hpp、eskf_core.cpp；本机已读 |

## 8. 官方模块参考覆盖

以下覆盖官方索引 30 个主头文件，源码默认分支只读获取；blob SHA 标识本次实际读取的文件。按实际构造形状归纳，不把示例或纯库的特殊写法升级为所有模块必须遵守的规则。

| 官方模块 | 已读主头文件 blob SHA | 构造形状 |
| --- | --- | --- |
| [BlinkLED](https://github.com/xrobot-org/BlinkLED/blob/master/BlinkLED.hpp) | `c4ef63a4860c2ec2bc2687e204e557947157fbd6` | 对象依赖（如有）及独立默认配置 |
| [BMI088](https://github.com/xrobot-org/BMI088/blob/master/BMI088.hpp) | `a3d7f6bf3d1ced209ca042abb7c9d839bf5b847b` | 对象依赖及具名默认 Param |
| [ICM42688](https://github.com/xrobot-org/ICM42688/blob/master/ICM42688.hpp) | `2a7b545654c7de72929a73919307073dee4f9625` | 对象依赖及具名默认 Param |
| [ICM20948](https://github.com/xrobot-org/ICM20948/blob/master/ICM20948.hpp) | `d8373e1a61de3d202b097b99ca2b6d47dfa71f9f` | 对象依赖及具名默认 Param |
| [MadgwickAHRS](https://github.com/xrobot-org/MadgwickAHRS/blob/master/MadgwickAHRS.hpp) | `48acb00ae408e48944e46282b05b4a3f36875866` | 对象依赖及具名默认 Param |
| [IST8310](https://github.com/xrobot-org/IST8310/blob/master/IST8310.hpp) | `27f250a93801d603dd334906e68d4620c5370f14` | 对象依赖（如有）及独立默认配置 |
| [BuzzerAlarm](https://github.com/xrobot-org/BuzzerAlarm/blob/master/BuzzerAlarm.hpp) | `06bd4713903eb4c43d1b1af1e18a1ee0de120596` | 对象依赖（如有）及独立默认配置 |
| [TestModule](https://github.com/xrobot-org/TestModule/blob/master/TestModule.hpp) | `97077e865cfc770451693e85d0957dec1e03593a` | 对象依赖（如有）及独立默认配置 |
| [CanfdIMU](https://github.com/xrobot-org/CanfdIMU/blob/master/CanfdIMU.hpp) | `047b9657052ce80ff7730c86892e4bca675da31a` | 对象依赖及具名默认 Param |
| [CanIMU](https://github.com/xrobot-org/CanIMU/blob/master/CanIMU.hpp) | `a846d10ea69dc7937af95109cfa2915d1c1ba4c1` | 对象依赖及具名默认 Param |
| [SharedTopicClient](https://github.com/xrobot-org/SharedTopicClient/blob/master/SharedTopicClient.hpp) | `148784ac8b0c5e06791240c042c990fff5536e76` | 对象/列表依赖及默认配置列表 |
| [SharedTopic](https://github.com/xrobot-org/SharedTopic/blob/master/SharedTopic.hpp) | `5b45f491d46c019193d8c804dcec42eda569d049` | 对象/列表依赖及默认配置列表 |
| [ST7735](https://github.com/xrobot-org/ST7735/blob/master/ST7735.hpp) | `60bdee95c5d98874e55dd1f99c8addf18c7cb842` | 对象依赖（如有）及独立默认配置 |
| [W25QXX](https://github.com/xrobot-org/W25QXX/blob/master/W25QXX.hpp) | `99f959f182c6ac67d36e465018e133bc6ff72aa0` | 仅对象依赖 |
| [QMI8658](https://github.com/xrobot-org/QMI8658/blob/master/QMI8658.hpp) | `4dc65f6907ccb4f9f4b0857c831dd4787e2b2f7d` | 对象依赖及具名默认 Param |
| [BMI270](https://github.com/xrobot-org/BMI270/blob/master/BMI270.hpp) | `60e1eb2c15d7341e55e0586aef348b0eb09f8673` | 对象依赖及具名默认 Param |
| [QMC5883L](https://github.com/xrobot-org/QMC5883L/blob/master/QMC5883L.hpp) | `853adbb3762f0f6ca87b0a37e3c58b45cc7d2065` | 对象依赖（如有）及独立默认配置 |
| [QMC5883P](https://github.com/xrobot-org/QMC5883P/blob/master/QMC5883P.hpp) | `1f87071cd50c5f839304dae6132fa0419e7d2ffb` | 对象依赖（如有）及独立默认配置 |
| [SPL06](https://github.com/xrobot-org/SPL06/blob/master/SPL06.hpp) | `a3a4208810aea3b07ce8786e60dfbddc53f6f8f7` | 对象依赖（如有）及独立默认配置 |
| [AK8975](https://github.com/xrobot-org/AK8975/blob/master/AK8975.hpp) | `93c929c2fd584a79e95f83ae1a9f0d1c109cf750` | 对象依赖（如有）及独立默认配置 |
| [R9DS](https://github.com/xrobot-org/R9DS/blob/master/R9DS.hpp) | `361115363011d80c7ab528468fbd4eba1005db4d` | 对象依赖（如有）及独立默认配置 |
| [ANOFlow](https://github.com/xrobot-org/ANOFlow/blob/master/ANOFlow.hpp) | `d6c15160542721607793faebfea7b0643176baaa` | 对象依赖（如有）及独立默认配置 |
| [SX1281](https://github.com/xrobot-org/SX1281/blob/master/SX1281.hpp) | `09eac7b122d06a7525fac5426f6679a6c0bf8d71` | 对象依赖及可选指针 |
| [MLX90640](https://github.com/xrobot-org/MLX90640/blob/master/MLX90640.hpp) | `89e5811b544fb0f058025ad7e22511bfd372dbc4` | 对象依赖及具名默认 Param |
| [INA228](https://github.com/xrobot-org/INA228/blob/master/INA228.hpp) | `6ede9ba9e26c697b05e0f1e557f1f5708f0579ca` | 对象依赖及具名默认 Param |
| [LC307](https://github.com/xrobot-org/LC307/blob/master/LC307.hpp) | `fa84a4f1b74444f01807e5543ea2eb54fcc5c86b` | 对象依赖（如有）及独立默认配置 |
| [STP23L](https://github.com/xrobot-org/STP23L/blob/master/STP23L.hpp) | `0637c96523a1f52253bbe54b00522a4ae90fb0e3` | 对象依赖（如有）及独立默认配置 |
| [GreySensor](https://github.com/xrobot-org/GreySensor/blob/master/GreySensor.hpp) | `ca447e83a5e6583229e794695f25858181e6e1cd` | 对象/列表依赖及默认配置列表 |
| [DurationStatistics](https://github.com/xrobot-org/DurationStatistics/blob/master/DurationStatistics.hpp) | `f175953628e68d9c90619e0f095b284203281306` | 统计工具类，非硬件配置构造 |
| [LSM6DSV16X](https://github.com/xrobot-org/LSM6DSV16X/blob/master/LSM6DSV16X.hpp) | `18d8907d6d2aa21c664f65d756cdc41f1a007f32` | 对象依赖及具名默认 Param |
