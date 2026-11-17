> 本文记录本地开发阶段。后续已发布到WDR远程仓库并移除LocalModules，当前恢复方式见publication_20261117.md。

# 发射指令统一及LibXR复用

## 变更

本次只修改本BSP下的LocalModules源仓库；未修改其他工作树，未发布远端。
CMD、I6x、Launcher、Gimbal、HostData、DR16、Referee的兼容修改均保留在各自本地源仓库。
活动产品仅实例化CMD、I6x、四个RMMotor及ThreeFricLauncher；不添加底盘、云台或裁判实例。

- CMD::LauncherCMD为open_fric/is_fire。OP使用遥控开轮/拨弹意图，CMD持有事件切换状态。
- AUTO以auto_open_fric（默认true）决定开轮；AI只提供拨弹意图，超时清除拨弹。
- 所选遥控输入input_online为false时关闭两者，含AUTO。CMD不自行判定原始遥控帧状态。
- I6x沿用原online策略（连续异常帧阈值及有效帧超时）；不采用旧Task额外的逐异常帧停机策略。
- CMD切换模式立即重发指令，并清除旧OP事件锁存；遥控离线清除锁存，恢复后不自动开轮。
- I6x默认SW1_DOWN边沿绑定CMD::FRIC_TOGGLE，fric_toggle_event可在YAML配置；不要再额外绑定一次。
- 其他输入的事件绑定迁移至CMD，旧Launcher::Event/GetEvent接口删除。
- 两种Launcher从Topic消费两个字段，command_timeout_ms默认/配置30ms；电机故障及恢复在机构内处理。
- 电机掉线或Topic超时停机后，需先收到健康关闭指令再开启，避免旧true在恢复时重启。
- ThreeFricLauncher内置1ms线程，保持四电机同拍0x200组帧。独立Task已从源码装配、请求、索引、YAML及锁中移除。
- WDR Launcher保留云台同拍调度、机构permit、OP/AUTO到速判据、热量/弹速和堵转算法；不再按AIM开轮。
- Gimbal移除切换AUTO时的Launcher::Reset以及Request.aim；机构permit撤销仍停机，恢复后按当前CMD意图执行。

## PI和PID

三摩擦删除自定义PI/TWO_PI及DiscretePid，使用LibXR::PI/TWO_PI和LibXR::PID<float>。
YAML仍使用原1ms离散增益，构造时P不变、I除以0.001、D乘以0.001，积分输出限幅除以连续I增益。
PID和公共二阶滤波器均使用实际dt。删除不再使用的d_history_weight配置，现有各回路D均为0。
LibXR采用输出限幅感知的抗饱和和反馈微分，不能宣称与旧PID逐拍等价：
P4000、旧I50，目标21，反馈先0再20.9，旧输出10000/900，新输出约10000/405。
WDR Launcher已有公共wdr_control::PID，其内部复用LibXR::PID，本次保留该公共封装。
未调整几何、目标速度、方向、P/I增益或电流上限；实机需要重新验证PID瞬态。

## 验证范围

主机测试覆盖CMD的OP/AUTO、配置关闭、遥控离线、AI超时、I6x实际事件绑定，
两种Launcher的Topic超时、电机故障恢复，三摩擦组帧/独立制动/堵转/PID及滤波器，
WDR热量停拨和裁判弹速样本调节。

默认入口：python3 tests/three_fric/run_tests.py。
附加集成编译可使用--integration-modules指定完整WDR模块目录；本次使用只读参考工作树的
Modules/WDR-Xrobot-org，实际编译当前LocalModules/Gimbal及其组件、HostData、DR16和Referee。
这项检查验证接口可编译，不构造或运行云台机构，也不代表完整WDR产品已完成链接或实机验证。

当前产品完成Debug/Release编译及冻结锁离线恢复。详细结果和ELF摘要见validation_cmd_20261117/evidence.json。
未烧录、未使能、未驱动机构；板上最后一次已知镜像仍是先前仅做启动验证的版本。
旧源码和下载的Task备份位于build/dependency_backups/cmd-migration，仅作回溯，不参与构建。
