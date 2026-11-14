# 私有组件仓库迁移（2026-11-04）

用户确认：迁到GitHub账号YunMo-yanyu名下的私有仓库，保留上游历史和许可证。
本轮包含MC02基座、WDR维护的基础组件和新增I6x；LibXR与xrobot-org官方模块继续使用上游。

| 仓库 | 用途 |
| --- | --- |
| YunMo-yanyu/wdr_xrobot_dmmc02_framework | 当前MC02主工程 |
| YunMo-yanyu/xrobot-modules | 私有组件索引，作为ModuleIndex子模块 |
| YunMo-yanyu/CMD | 命令中枢，通用分机构来源选择事件 |
| YunMo-yanyu/Motor | 电机抽象接口 |
| YunMo-yanyu/RMMotor | DJI电机驱动 |
| YunMo-yanyu/DMMotor | 达妙电机驱动 |
| YunMo-yanyu/DR16 | DR16遥控输入 |
| YunMo-yanyu/Referee | 裁判系统 |
| YunMo-yanyu/HostData | 上位机输入汇总 |
| YunMo-yanyu/I6x | i6x的正式XRobot模块 |
| YunMo-yanyu/EventBinder | YAML事件绑定，来源与模式事件解耦 |

原组件Git提交作为新仓库的祖先保留，原LICENSE未改写，UPSTREAM.md记录来源与迁移基线。
I6x额外记录26_sentry原映射来源及MIT声明。组件提供main/dev分支，提交由主工程xrobot.lock锁定。
这些是独立私有仓库，未改变原第三方远端，也没有将公开fork误设为私有。

生产依赖标识和模块manifest已迁到YunMo-yanyu，配置、生成器、测试路径一起更新。
第三方来源集中保留在声明文件及Git历史中。原本构建时应用CMD补丁的步骤已移除，补丁归档至
`docs/archive/cmd_i6x_pre_module`。I6x不再以User/remote里的本地组件或Git包方式维护。

## 私有索引与鉴权

XRobot的索引HTTP读取不自动带GitHub登录，因此不使用私有仓库的raw网页地址。
主工程用Git子模块检出私有索引，Modules/sources.yaml读取本地ModuleIndex/index.yaml；
模块仓库仍由标准XRobot工具克隆、校验并锁定。

在具有权限的新电脑上登录gh并执行`gh auth setup-git`，再递归克隆主工程即可读取私有仓库。
本轮发布与拉取使用单次命令的gh凭据辅助，没有将令牌写进仓库、索引或URL。

当前OAuth登录缺少workflow权限，继承的Actions工作流已从活动目录归档到docs/upstream_ci
（主工程为docs/archive/ci），Git历史中保留原版本。没有自动复制本机令牌到CI Secret。
跨私有仓库CI尚未配置，当前验收为本机生成、编译与回归，不声称云端CI已通过。

## 默认行为和验证

默认产品仍为User/xrobot.yaml，只运行已验收的板级功能。User/xrobot_i6x.yaml是可显式选择的
独立配置，通过YAML创建CMD和I6x；默认不选择、不启动外接设备。本轮未烧录。

验收包括：冻结／离线setup、标准I6x模块接口发现、模块构造生成、完整i6x YAML生成并编译、独立检出中i6x固件完整链接、
Debug/Release构建、原始解码对照与失控/STOP回归、默认ELF隔离检查、板级数据库回归。
主机I6x冻结源对照依赖本机26_sentry参考目录，若仅构建固件不需要该参考目录。
精确组件提交、仓库私有状态和检查结果另记于docs/validation/private_migration_20261104.json。

2026-11-05 新增私有模块 ImuFusion、BoardAttitude、DmImu（2026-11-14 分别改为 ImuEkfFusion、AttitudeFusion），当前版本与验收见 [IMU接入记录](imu_modules_20261105.md)。

> 2026-11-06 清理更新：本文中的多份独立 YAML 为当时验证入口，现已删除。当前机器人入口统一为 `User/xrobot.yaml`；底盘、DM IMU 增量参数位于 `config/templates/chassis.yaml`、`config/templates/dm_imu.yaml`，由 `tools/generate_external_checks.py` 合并并生成回归组合，不直接运行旧生成命令。历史验证结果保留原样。
