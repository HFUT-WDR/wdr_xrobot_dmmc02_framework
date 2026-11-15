# WDR 模块 README 对齐记录

日期：2026-11-15。范围：ModuleIndex 登记的全部 21 个 WDR-Xrobot-org 模块。参照 [MadgwickAHRS README](https://github.com/xrobot-org/MadgwickAHRS/blob/master/README.md)，读取时 README 文件 blob 为 `d582b0b08abd8ead815820dddd5a92ca306c3348`。遵循已加载的 xrobot / xrobot-contrib 技能；未修改技能。

## 交付

全部 README 使用七个编号双语标题：模块作用、运行约定、构造接口、Topic、配置示例、依赖与硬件、来源与许可。运行约定集中容纳参考文档中的时间戳和命令等可选章节，保留其五项核心内容。

19 个可实例化模块直接包含 `module add → setup → instance add → 完整 YAML → gen` 教学；Motor、ControlAlgorithms 明确无需实例化，直接提供添加模块、include 与调用示例。参数、硬件依赖、时序和限制保留在各自 README，未新增 example/examples 目录。HostLink 的详细线格式表移至其 `docs/protocol.md`，README 保留完整接入示例和协议概要。

同步更新[模块文档约定](../module_documentation.md)与[模块开发规范](../module_development_standard.md)。模块实现、参数定义、算法、整车 YAML、锁文件与技能均未修改。21 个模块的文档已分别提交并推送至各自的 dev 分支，远端提交均已核对；提交列表见 [publication.json](module_readme_alignment_20261115/publication.json)。主工程的文档规范与本记录随本次提交交付。

## 校验

- XRobot 1.0.1 读取当前模块构造接口和类型索引，19/19 个 README YAML 完成完整字段、声明顺序、类型及入口生成检查。
- 生成的 19 份 C++ 入口和纯库的 3 个 C++ 调用片段，共 22/22 通过 `g++ -std=c++20 -fsyntax-only`。使用工程当前 LibXR 公共头、Linux 平台头及 Eigen；为示例前置资源提供与构造类型匹配的注册项，EventBinder 提供已声明的 I6x/CMD 前置对象。EKF 相关头使用其现有主机 CMSIS 适配头。Linux 的 64 位 long 格式启用 LibXR 64 位整数格式支持。这是文档接口/类型检查，不是固件链接、程序执行或硬件验收。
- 21/21 README 章节编号连续；仓库内 Markdown 链接可解析；各模块 `git diff --check` 通过。检查工作树仅 README 变更，HostLink 额外新增协议文档。
- 校验结果、源码基线及 README 内容 SHA256 见 [results.json](module_readme_alignment_20261115/results.json)。验证过程中未执行 README 中的 setup/gen 来覆盖主工程配置。

## 随文档纠正的事实

- HostLink 与 HostData 当前协议版本不同：前者为 `auto_aim_cmd/plan_mode/nav_cmd` 与新版反馈结构；后者为 `host_aim/host_nav` 与旧版反馈结构。两份 README 明确不能直接配对，未修改实现或提供未经验证的协议适配。
- 修正 ESKF 旧文案中的默认输出名称为 `eskf_output`；保留按实际采样时间戳推进的当前约定。
- I6x 的 Topic 名改为双引号字符串；Referee 的 chassis 聚合配置展开完整字段；BuzzerAlarm 使用 WDR 模块 ID，删除与当前 Gimbal 行为不符的固定蜂鸣叙述。
- BMI088 构造接口补齐实际默认参数；HostData、HostLink 补齐本地/线上 Topic、构造接口、完整配置及前置资源说明。

## 逐模块索引

| 模块 README | 校验源码提交 | 示例校验 |
| --- | --- | --- |
| [BMI088](../../Modules/WDR-Xrobot-org/BMI088/README.md) | `1bfbc124cefb` | 完整 YAML 与生成 C++ |
| [BuzzerAlarm](../../Modules/WDR-Xrobot-org/BuzzerAlarm/README.md) | `a6f70132d657` | 完整 YAML 与生成 C++ |
| [CMD](../../Modules/WDR-Xrobot-org/CMD/README.md) | `73f2bf4c97b3` | 完整 YAML 与生成 C++ |
| [Chassis](../../Modules/WDR-Xrobot-org/Chassis/README.md) | `4fcc2eb9e28f` | 完整 YAML 与生成 C++ |
| [ControlAlgorithms](../../Modules/WDR-Xrobot-org/ControlAlgorithms/README.md) | `06eae1291ec5` | 纯库 C++ 示例 |
| [DMMotor](../../Modules/WDR-Xrobot-org/DMMotor/README.md) | `3a93aa587d7a` | 完整 YAML 与生成 C++ |
| [DR16](../../Modules/WDR-Xrobot-org/DR16/README.md) | `8b91341c29b0` | 完整 YAML 与生成 C++ |
| [EventBinder](../../Modules/WDR-Xrobot-org/EventBinder/README.md) | `6698fd4d02f5` | 完整 YAML 与生成 C++ |
| [Gimbal](../../Modules/WDR-Xrobot-org/Gimbal/README.md) | `5b23f7438fea` | 完整 YAML 与生成 C++ |
| [HostData](../../Modules/WDR-Xrobot-org/HostData/README.md) | `3bea9d36130a` | 完整 YAML 与生成 C++ |
| [I6x](../../Modules/WDR-Xrobot-org/I6x/README.md) | `ddeaba83d432` | 完整 YAML 与生成 C++ |
| [Launcher](../../Modules/WDR-Xrobot-org/Launcher/README.md) | `1dcbba2299c3` | 完整 YAML 与生成 C++ |
| [Motor](../../Modules/WDR-Xrobot-org/Motor/README.md) | `cf91a88d2414` | 纯库 C++ 示例 |
| [PowerControl](../../Modules/WDR-Xrobot-org/PowerControl/README.md) | `dc892dbcbaea` | 完整 YAML 与生成 C++ |
| [RMMotor](../../Modules/WDR-Xrobot-org/RMMotor/README.md) | `b8570cb05f55` | 完整 YAML 与生成 C++ |
| [Referee](../../Modules/WDR-Xrobot-org/Referee/README.md) | `91ed2ad8f758` | 完整 YAML 与生成 C++ |
| [DmImu](../../Modules/WDR-Xrobot-org/DmImu/README.md) | `386d33325691` | 完整 YAML 与生成 C++ |
| [AttitudeEkf](../../Modules/WDR-Xrobot-org/AttitudeEkf/README.md) | `6f94a01fa6a0` | 完整 YAML 与生成 C++ |
| [ESKF](../../Modules/WDR-Xrobot-org/ESKF/README.md) | `84c183018310` | 完整 YAML 与生成 C++ |
| [HostLink](../../Modules/WDR-Xrobot-org/HostLink/README.md) | `01a559fa6dc7` | 完整 YAML 与生成 C++ |
| [rm_decision](../../Modules/WDR-Xrobot-org/rm_decision/README.md) | `9528f6e32689` | 完整 YAML 与生成 C++ |

提交前将 rm_decision 文档移至现有 dev 基线，保留该分支的 ROS Nav2Pose 节点及依赖说明；其公开构造接口未变化。未将 main 的 CI 变更合入 dev。
