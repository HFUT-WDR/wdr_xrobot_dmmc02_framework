# RMMotor 型号配置与反向反馈修正

新增 M2006Param、M3508Param、GM6020Param 和不同参数名的构造重载，委托原 Param 入口；旧入口仍排第一，保持 XRobot 自动补参选择。四台 M2006 YAML 和 ThreeFricLauncher 示例已迁移。

反馈力矩改用已按 reverse 转换的反馈电流计算，与角度、速度及控制输出使用相同方向。正向配置不改变数值；反向配置的力矩反馈符号得到修正。

验证：五种入口（M2006、M3508、GM6020 CURRENT/VOLTAGE、旧 Param）经过实际 xrobot gen 与 C++ 编译。主机测试 3/3 通过；驱动测试覆盖正反方向、正负零电流、反馈/控制分组边界、新旧输出逐字节比较。Debug/Release 构建均通过。

审查：未发现本次修改遗留的阻断问题。保留旧 Param 和同组协调机制，构造不发送。C++ 无类型花括号可能因重载而歧义，调用者应显式写对应参数类型；旧 RMMotor(can, {}) 应改为 RMMotor(can, RMMotor::Param{})。YAML 使用不同参数名，无此歧义。

当前模块修改为各自 Git 仓库中的未提交工作；本目录保存补丁供审查和恢复。尚未提交、推送或更新 xrobot.lock；当前验证针对工作区源码，不代表从旧锁文件全新恢复即可得到新接口。后续发布需要先提交两个模块，再更新工程锁定。未烧录或执行实体电机测试。

后续归档说明：实现阶段补丁已移到忽略目录 build/publication_backups/pid_review_patches；最终模块源码以 pid_unification_20261117/versions.md 列出的提交为准。
