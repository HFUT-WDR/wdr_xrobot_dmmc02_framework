# WDR模块冻结快照

当前仅维护HFUT-WDR命名空间。HFUT-WDR目录保存18个自维护模块的Git bundle和本地索引；正式模块仓库、ModuleIndex和CI均由组织维护。
Modules/sources.yaml优先读取组织索引，并保留HFUT-WDR本地快照源用于冻结恢复。xrobot.lock记录实际提交。

运行python3 tools/check_module_snapshots.py可在临时目录检查18个快照与22个模块的冻结离线恢复。
维护模块时同步更新组织仓库、对应bundle和锁文件。构建、烧录及机构闭环验证范围分别记录。

旧Modules/YunMo-yanyu目录及个人快照已移出工程，不再参与构建或维护；个人GitHub仓库保留历史。
