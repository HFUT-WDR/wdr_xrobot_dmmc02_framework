# 底盘具名故障快照

`wdr_chassis::State::faults` 从整数位掩码改为 `wdr_chassis::Faults`，包含七个独立布尔字段：
`input_lost`、`command_stale`、`yaw_lost`、`motor_lost`、`gimbal_stop`、`invalid_target`、`invalid_feedback`。

调试器可展开 `chassis.control_.state_.faults`，监视其中任一字段；实际对象路径以当前生成实例为准。
`Any()` 汇总是否存在故障；`Merge()` 将发送前新发现的故障合入本拍快照，不清除此前故障。
公开故障位枚举移除，不保留旧整数接口，不维护第二份诊断状态。
云台 Topic 回调内部仍以一次原子写入传递三个健康判据，避免把同一帧拆成多次非一致读取；该私有传递编码不作为发布状态。

只改变故障表示方式：原失控、STOP、反馈和命令时效、清历史及零电流保护保持不变。
`State` 是进程内 Topic 类型，本次布局发生变化，消费者随锁定模块同步构建；未新增通信协议编码。
终端 `chassis` 命令按名称打印七项 0/1 状态。

验证：底盘主机 ASan/UBSan 回归（含七字段独立判据及合并保留测试）、板级支持回归、Debug/Release 构建及默认 ELF 模块隔离检查均通过。
未烧录、未运行电机，未宣称真实硬件闭环验证。
