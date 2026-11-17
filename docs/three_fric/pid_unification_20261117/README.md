# 公共 PID 统一与审查（2026-11-17）

## 行为

ControlAlgorithms 的 wdr_control::PID 统一适配 LibXR::PID，i_limit 限制 Iout，内部换算累计误差限幅。运行优先使用实测 dt，超出 min_dt/max_dt（含非有限周期）仅本次使用 nominal_dt；边界值保留实测周期，下一拍正常立即恢复。零输出上限关闭输出并清状态，积分回算不会给关闭的积分/输出重新注入状态。

PidTiming 默认用于1ms环：nominal_dt=0.001、min_dt=0.0005、max_dt=0.004（单位s）。BMI088使用0.05/0.025/0.2，并在50ms定时回调中测量实际微秒间隔，不再固定传入0.05。

ThreeFricLauncher与Launcher统一通过公共PID。旧1ms离散增益只在入口换算，Iout上限不在模块中重复转换。三摩擦拨弹滤波与PID共用本拍选择后的周期；Launcher热量/卡弹计时仍使用实际dt。

GimbalPID本来已限制积分输出，保留其独立算法（积分分区、角速度D、前馈、回算），只统一周期策略。其integral_limit与通用PID的i_limit均表示Iout上限，不强行改变原有控制行为。LibXR本身未修改。

## 验证

- 主机测试4/4通过，包括三摩擦行为、公共滤波器、RMMotor以及新增公共PID测试。
- 公共PID测试覆盖实测dt、上下边界、超范围、下一拍恢复、不同Ki下相同Iout上限、复位、积分回算、积分/输出关闭、抗饱和、D周期、50ms温控周期及云台角度/速度环周期。
- Gimbal/HostData/DR16调用链以及BMI088头文件兼容编译通过。
- User/xrobot.yaml与User/three_fric.yaml均实际生成；板级生成配置（含新BMI088聚合参数）单独编译通过，当前生成头已恢复三摩擦配置。
- 当前三摩擦Debug/Release均成功构建；未烧录、无实体电机或加热运行验证。

## 审查与迁移边界

当前工程生产代码直接LibXR PID使用仅留在公共适配内；未发现本次实现遗留的阻断问题。三摩擦与Launcher默认D为0，现有行为测试通过；三摩擦改用公共微分先行策略，非零D用户需重新确认动态响应。BMI088配置类型由LibXR Param改为公共PidParam：去掉k，p/i/d改为kp/ki/kd，并增加d_alpha/deadband/timing，i_limit统一为Iout上限。当前YAML已迁移。

范围为当前工程解析模块以及已缓存的云台兼容调用链，不代表其他未检出的WDR仓库已自动迁移。模块均为各自仓库中的未提交修改，本目录保存可恢复补丁。未提交、推送或更新xrobot.lock；发布前须先提交各模块，再重新解析并锁定依赖。不要以旧锁文件的全新恢复结果替代本次工作区源码。

后续归档说明：实现阶段补丁已移到忽略目录 build/publication_backups/pid_review_patches；最终模块源码以 pid_unification_20261117/versions.md 列出的提交为准。
