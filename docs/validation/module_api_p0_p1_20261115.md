# P0/P1 模块 API 实施记录

## 任务契约

- 目标：实施已批准方案中的 ESKF、AttitudeEkf、DmImu、Chassis、Gimbal、Launcher、PowerControl，补齐各模块 README 内教学。
- 范围：本目录中的七个独立模块工作副本、本框架配置、迁移工具、相关文档；其他框架仅只读查调用者，并在本目录的隔离副本验证和生成迁移补丁。
- 不做：改技能、P2 接口改造、改控制逻辑（ESKF实际步长为用户另行授权的变化）、PR/合并、烧录、写设备、修改兄弟目录的现有工作区。
- 接口决策：直接迁移已知调用者，不加旧构造兼容层；复杂配置归入末尾 Param。采集/控制已有独立参数类型保留为 device/control 子字段，不继承、不另建公共骨架库。
- ESKF 决策（按用户后续明确要求更新）：接入实际采样步长，MEASURED 使用相邻时间戳及现有 ClampDt 规则，首拍使用名义周期。保留与 EKF 相同的步长来源配置；预测、噪声离散化和低通均使用同一个 dt。初始化、恢复、静止窗口仍按样本数，不改计数参数含义。
- 保持：有效算法参数、Topic 类型与名称、消息布局、事件编号、复位、输入配对、队列策略、任务周期与优先级、控制门控、发送顺序与持久化键。
- 验证预算：模块解析与 README 示例生成；实际配置有效值对照；已有算法/协议与机构主机回归；板级 Debug/Release；隔离整车配置生成与实际涉及模块的固件链接。运行失败先定位原因，禁止改变产品行为让检查通过。
- 停止条件：上述软件检查完成并记录局限；不声称板上或机构已验收。
- 执行面：本机软件与交叉编译。实施前当前框架 User/xrobot.yaml 已有用户添加的 ESKF 实例，保留意图并迁移。

## 决策记录

1. 用户批准 P0/P1 实施；之前的“只写方案”范围由本次授权更新。
2. 官方允许简单标量与复杂 Param 并存，因此 P2 接口不扩改。
3. 初步按固定 1 ms 保留行为的选择已被用户“ESKF参考EKF和madgwick都用实际时间”覆盖；改为实际步长，并单独验证此项获准的算法行为变化。
4. 27_sentry_work 存在其他未提交的子模块/锁文件状态，只读使用，不修改或恢复。

## 接口前后与生产消费者

| 模块 | 旧接口 | 新接口 | 已知消费者 |
| --- | --- | --- | --- |
| AttitudeEkf | Topic、Param、RamFS、命令名 | RamFS、Param | 本框架 User/xrobot.yaml；27_sentry_work 板载和外置 IMU 配置/模板 |
| ESKF | Topic、Param（原步长未接入）、RamFS、命令名 | RamFS、Param（实际步长已接入） | 本框架新增实例；27_sentry_work 锁定该仓库，默认配置未必实例化 |
| DmImu | Param、CAN*/UART*、Database/RamFS、Topic/安装/命令 | Database/RamFS、CAN*/UART*、Param | 27_sentry_work 外置 IMU 配置/模板及四种传输编译检查 |
| Chassis | 电机/CMD/Power、Param、RamFS、命名与任务参数 | 电机/CMD/Power/RamFS、Param | 27_sentry_work 底盘/整车配置、模板、主机测试 |
| Gimbal | 电机/CMD、Param、可选对象和命名交错 | 电机/CMD、Database*/Launcher*/Buzzer*、Param | 27_sentry_work 云台/整车配置、模板、主机测试 |
| Launcher | 电机、Param、Topic | 电机、Param | Gimbal 同拍驱动；27_sentry_work 云台配置、模板、主机测试 |
| PowerControl | Param、CAN*、超电参数、Topic | CAN*、Param | Chassis 同拍调用；27_sentry_work 底盘配置、模板、主机测试 |

十维契约对照：源兼容性有意改变、已知配置/构造调用需迁移，配置结构改变但线上的消息布局不变；返回码与容量、数据顺序与部分消费、阻塞/超时、完成/回调、缓冲所有权、复位/恢复、调用上下文、硬件副作用不因接口搬迁改变。参数存储大小与 Flash/RAM 由最终构建记录核对，不预先承诺零增长。

## 进度与验证

七模块的实现、README教学、主配置迁移与软件检查已完成。用户随后明确授权提交并推送；七模块已分别提交到远端dev，本框架锁文件由XRobot正常解析升级为新AttitudeEkf/ESKF提交，其余三个已锁定模块保持原提交。五模块冻结离线恢复、重新生成和姿态布局检查均通过。

### 已执行的软件检查

| 检查 | 实际范围与结果 |
| --- | --- |
| 构造解析 | 七模块均可由XRobot 1.0.1读取新构造；不保留旧构造重载 |
| README配置 | 七份README直接包含module add、setup、instance add、完整YAML、参数表与gen；逐份嵌入隔离整车配置，生成全部通过；ESKF原examples/xrobot.yaml已移除 |
| 原配置对照 | 对照未改动的整车工作区，16个旧实例中每个原先显式填写的配置值均保留；YAML迁移工具再次运行无重复改动 |
| ESKF | ASan/UBSan通过；既有1ms回归通过；0.6/2.4/10ms混合采样累计13s，滤波关闭/开启最大倾斜误差分别0.0000059/0.0000062rad；零角速一步预测解析解核对dt、dt²、dt³及协方差交叉项 |
| EKF | 冻结DM参考源8100次比较、逐样本解算、拒绝无效输入、后处理、独立实例与复位回归通过 |
| DmImu | CAN请求/主动上报、RS485请求、旧连续流、组帧、安装、超时/重试与错误帧主机回归通过 |
| 底盘 | 冻结轮系、控制器、真实模块装配、构造不发帧、超时/恢复、云台许可、RAW保护和功率约束回归通过 |
| 云台/发射 | 冻结PID、1000组双Yaw规划与1000组Pitch力矩对照；模式、AI失效/恢复、校准持久化、边界与同拍Launcher联动通过；发射模式、热量、弹速、卡弹、滤波回归通过 |
| 消息共存 | 修复EKF与ESKF共同类型在同一翻译单元重复定义；比较共同声明一致性、分别核对104字节姿态布局、两种头文件包含顺序均编译通过；算法独有的Ekf类不受共享guard屏蔽 |
| 板级固件 | 实际启用EKF、Madgwick、ESKF，Debug/Release均链接通过 |
| 隔离整车固件 | sentry_remote的Debug/Release均链接通过，ELF包含Chassis、Gimbal、Launcher、PowerControl、DmImu实际路径；额外编译检查覆盖四种DM传输配置 |
| 补丁 | 七模块补丁在各自干净基线克隆上通过git apply --check；整车补丁对原工作区只读检查通过，未应用 |

主机测试使用模块已有入口与整车原有回归，仅迁移测试构造和工具字段路径，不改变冻结参考或断言预期。整车测试桩的姿态字段顺序同步为已锁定的roll,pitch,yaw契约。ESKF测试启动最初遇到本机ASan加载顺序限制，使用`ASAN_OPTIONS=verify_asan_link_order=0`运行后通过，未关闭ASan/UBSan插桩。

### 资源与证据

| 固件 | DTCM | Flash使用 |
| --- | --- | --- |
| 本框架Debug | 94,720 B / 128 KiB | 243,860 B |
| 本框架Release | 94,720 B / 128 KiB | 234,320 B |
| 隔离整车Debug | 94,720 B / 128 KiB | 270,308 B |
| 隔离整车Release | 94,720 B / 128 KiB | 257,928 B |

这些是最终绝对占用，不作为改造前后的净增量；本框架原生成头没有ESKF实例，不能用那一份旧产物宣称接口搬迁的大小变化。

- [结构化结果与ELF摘要](module_api_p0_p1_20261115/results.json)
- [七模块基线与补丁摘要](module_api_p0_p1_20261115/baseline.json)
- [EKF回归](module_api_p0_p1_20261115/ekf-tests.log)、[DM协议回归](module_api_p0_p1_20261115/dm-tests.log)、[底盘回归](module_api_p0_p1_20261115/chassis-tests.log)、[云台回归](module_api_p0_p1_20261115/gimbal-tests.log)、[发射回归](module_api_p0_p1_20261115/launcher-tests.log)
- [本框架Debug链接](module_api_p0_p1_20261115/debug-build.log)、[Release链接](module_api_p0_p1_20261115/release-build.log)、[隔离整车Debug链接](module_api_p0_p1_20261115/consumer-debug-build.log)、[Release链接](module_api_p0_p1_20261115/consumer-release-build.log)

### 已知调用者的迁移交付

本框架User/xrobot.yaml已迁移并生成入口。其余真实整车配置位于用户指定操作目录之外，且该工作区原本有改动，因此仅在本目录`.agents/api_normalization/consumer`的隔离副本落地和验证；对应[27_sentry_work补丁](module_api_p0_p1_20261115/27_sentry_work.patch)包含User配置、模板、测试构造、测试桩及generate_external_checks.py字段路径迁移。原工作区未应用补丁，不能把隔离验证称为该工作区已经升级。

可重复使用的[配置迁移工具](../../tools/migrate_module_api.py)默认只显示差异；显式传入文件并加`--apply`才写入。对旧Param的任意C++表达式不做猜测转换；源码构造调用按各模块README与迁移说明手工迁移。

七个模块不属于本框架跟踪的源码，分别在Modules/WDR-Xrobot-org内的独立Git仓库中提交并推送。为便于审阅和后续提交，在记录目录保存了[AttitudeEkf](module_api_p0_p1_20261115/AttitudeEkf.patch)、[ESKF](module_api_p0_p1_20261115/ESKF.patch)、[DmImu](module_api_p0_p1_20261115/DmImu.patch)、[Chassis](module_api_p0_p1_20261115/Chassis.patch)、[Gimbal](module_api_p0_p1_20261115/Gimbal.patch)、[Launcher](module_api_p0_p1_20261115/Launcher.patch)、[PowerControl](module_api_p0_p1_20261115/PowerControl.patch)补丁（含新建迁移说明）。这些补丁保留实施前基线，供审阅与未升级工作区迁移；模块本身现已分别提交推送，主框架锁文件也已按正常XRobot流程升级。

### 消减检查与界限

- control/device子字段承载已有纯算法/采集配置；没有引入继承、公共骨架库、兼容构造或新线程。
- YAML迁移工具服务当前框架和真实整车配置，默认预览；没有改生成器来容忍残缺映射。
- 共享消息guard服务同一主配置中同时实例化EKF与ESKF的实际编译需求，保留定义一致性检查，不用guard掩盖布局差异。
- ESKF只把既有方程中的固定步长替换为本拍步长；初始化/恢复/静止窗口仍按样本计数，未新增静止偏置学习或修改噪声默认值。
- 完成源码差异自查；没有独立评审、设备收发、烧录、物理机构或姿态外部真值验收，不将离线误差推广为实机精度。


## 提交与推送

用户授权范围为七个模块与本框架dev分支，不创建PR、不合并发布分支，也不修改兄弟工作区。模块远端提交号已逐个通过ls-remote核对；[发布记录](module_api_p0_p1_20261115/publication.json)保存完整提交号。

| 模块 | dev提交 |
| --- | --- |
| AttitudeEkf | [6f94a01fa6a0](https://github.com/WDR-Xrobot-org/AttitudeEkf/commit/6f94a01fa6a00b709e71a3327cab1dad40b4da51) |
| ESKF | [84c183018310](https://github.com/WDR-Xrobot-org/ESKF/commit/84c1830183102991d447359d88462908145d4b44) |
| DmImu | [386d33325691](https://github.com/WDR-Xrobot-org/DmImu/commit/386d333256916ba095d6dd3ecfdd4efac0a06d87) |
| Chassis | [4fcc2eb9e28f](https://github.com/WDR-Xrobot-org/Chassis/commit/4fcc2eb9e28f4c77ec066e32af6fd3ae0fbbd06e) |
| Gimbal | [5b23f7438fea](https://github.com/WDR-Xrobot-org/Gimbal/commit/5b23f7438feaf50f4a356960e9438fbee409570e) |
| Launcher | [1dcbba2299c3](https://github.com/WDR-Xrobot-org/Launcher/commit/1dcbba2299c3237e566b7ff9b60239aafd747f0c) |
| PowerControl | [dc892dbcbaea](https://github.com/WDR-Xrobot-org/PowerControl/commit/dc892dbcbaeafdbf5826dd772d6d1ac8e8234e1e) |

主框架本次提交包含规范、配置迁移、验证记录、可审阅补丁与新版锁文件；框架推送结果和CI最终状态在交付时单独报告，不将本地构建等同于远端CI完成。
