# 蜂鸣器统一占空比

目标与范围：BuzzerAlarm 所有音效由构造参数 duty 统一控制；同步 BMI088 调用、模块说明和 BSP 配置及锁。保留用户已选 C5/E5/G5/C6 上电旋律。无控制算法改动，不烧录、不执行机构动作。

行为：原先 Play/StartTone 可逐次指定占空比，报警写死 0.5%；现在两接口移除 duty 参数，所有输出通过唯一 HardwareTone 使用实例 duty_，默认及 BSP 配置均为 50%。BMI088 校准原 4% 改为统一配置。频率、时长及调度优先级保持。

决定：按用户要求统一配置并提交推送两个模块，随后使用 xrobot setup 更新 BSP lock。移除独立校准/普通提示 duty 状态，避免重复配置。无需兼容旧的逐次占空比接口。

验证范围：代码审查唯一 SetDutyCycle 路径；Debug/Release 构建独立三摩擦产品，并以默认配置 Debug 编译验证 BMI088 调用，最后恢复三摩擦配置。通过后停止软件验证；实板音量未验证。

验证结果：默认配置 Debug 构建通过；恢复 User/three_fric.yaml 后 Debug、Release 均通过。xrobot setup --frozen --offline 通过。lock 仅更新 BuzzerAlarm 至 e300574 和 BMI088 至 01193bc。源码审查确认所有音效通过唯一 SetDutyCycle(duty_) 输出；无独立占空比状态。未烧录，未验证实际声音和机构闭环。

模块已提交并推送各自 origin/dev；BSP 保留当前 feat/three-fric-launcher 分支。此前 HardFault 现场日志不属于本次提交范围。
