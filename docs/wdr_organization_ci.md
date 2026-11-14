# WDR组织仓库与CI

## 仓库与可见性

2026-11-13 起，19 个自维护模块与共享 CI 仓库迁到 WDR-Xrobot-org，均为私有仓库。
框架仓库（wdr_xrobot_dmmc02_framework、27_sentry）仍在 HFUT-WDR，但调用新组织的共享 CI。
例外：SharedTopic 与 SharedTopicClient 继续使用官方 xrobot-org 源，源码未改动。
依赖与索引统一使用 WDR-Xrobot-org 命名空间；ModuleIndex 子模块指向 WDR-Xrobot-org/xrobot-ci。
本地 Git bundle 快照已移除，离线恢复依赖已检出的 Modules/ 与 xrobot.lock，
可用 tools/check_module_snapshots.py 在临时目录验证。

## 自动运行的位置和流程

提交到dev/master、向这些分支发起PR或手动触发Actions时，GitHub提供Ubuntu临时runner。
任务在XRobot官方Docker镜像内运行，结束后销毁环境。结果在各仓库Actions页、提交状态及PR检查中可见。
不依赖维护者电脑开机，不接触板子。

模块仓库调用xrobot-ci的module-ci.yml：检出被测模块，读取组织与官方索引，解析依赖，生成只编译的构造调用，编译模块及依赖。
构造调用不执行，因此不初始化实际外设，不输出电机控制。
BSP仓库调用bsp-stm32-ci.yml：校验LibXR生成入口、YAML布局和锁文件，按Debug/Release预设进行ARM交叉编译，并上传固件与构建清单。
默认机器人配置仅启用现有板级实例；模块CI覆盖其他模块的编译接口。
只有有意推送master/main、发布标签或Release才进入上游定义的发布流程；首次迁移仅推送dev。

## 私有依赖凭据

GITHUB_TOKEN通常只能读取当前仓库，无法拉取另一个私有模块。现在改用组织 GitHub App：
安装范围为组织全部仓库，权限只有 Contents 与 Metadata 的只读。

每个 job 由 actions/create-github-app-token@v2 铸造一小时有效的安装令牌，
经 credential.helper 从环境变量提供给 git，不写进 git 配置、命令行参数或日志。
令牌自动过期，不需要手工轮换，也不再需要分发工作流。

调用方只需要一处配置：仓库级 Secret WDR_APP_PRIVATE_KEY（App 私钥 PEM）。
免费组织的组织级 Secret 不能被私有仓库读取，所以私钥按仓库配置。
App ID 不是密钥，已经写成共享工作流的 app-id 输入默认值，不再需要组织变量。

新增模块仓库时运行一次：

    tools/provision_module.py --key <私钥 PEM> --repo <仓库名>

脚本写入标准 build.yml、配置 Secret，并在 ModuleIndex/index.yaml 补一行；幂等，可重复运行。
`--all` 遍历已有检出做巡检，`--check` 只报告缺失项。App 私钥长期有效，
仅在泄露时重新生成并更新各仓库 Secret；私钥不要写入文件、提交或聊天。

xrobot-ci 是公开仓库。GitHub 免费版只允许同一组织内的私有可复用工作流互相调用，
公开仓库则不受组织限制，因此位于 HFUT-WDR 的 27_sentry 与 wdr_xrobot_dmmc02_framework
可以直接调用它。索引里的内容只有模块名与仓库地址，模块代码仍在私有仓库；
拉取时用调用方 job 铸造的安装令牌，所以私有模块不会因此泄露。

## 2026-11-13归档

HFUT-WDR 的 19 个模块仓库与 DualYaw 已归档（只读，历史保留），旧 dev 均为新 org dev 的祖先。
HFUT-WDR 剩余的 XRobot 仓库只有 27_sentry 与 wdr_xrobot_dmmc02_framework 两个框架，
以及已归档的 xrobot-ci 副本。

module_sources/HFUT-WDR 的 Git bundle 快照已删除；离线恢复改由已检出的 Modules/ 完成。
20 个模块仓库的 Module CI 全部通过（含 DmAttitude；2026-11-14 该模块被 AttitudeFusion 取代），两个框架仓库的 BSP CI 也通过。
框架跨组织调用公开的 WDR-Xrobot-org/xrobot-ci 已验证可用。

## 2026-11-13补充迁移

27_Sentry_Host、rm_decision、HostLink 迁入 WDR-Xrobot-org，连同其依赖一并收归本组织。
三者均保留完整历史，Module CI 通过；27_Sentry_Host 新增 Host CI，在 Linux 镜像内
执行 setup、Debug 构建与 ctest，本地 4 项测试与云端构建均通过。
HostLink 与 rm_decision 已登记进 ModuleIndex，总数 22。

HFUT-WDR 侧的 HostLink 已归档；27_Sentry_Host 与 rm_decision 的旧仓库需要仓库管理员
归档，当前账号没有 admin 权限。

## 验证边界

Actions绿色表示对应静态检查与编译通过；构建、烧录校验、实际收发、机构闭环为不同验证层次。
CI沿用xrobot-org/XRobot v1共享工作流，WDR维护私有索引、认证和组织调用配置；源许可证保留在xrobot-ci。

## 2026-11-07验收

20个仓库全部私有且读取成员集合一致；个人仓库保留。
18个模块CI、共享CI自检、BSP生成/配置/锁文件检查及Debug/Release云端构建均已通过。
[BSP运行及固件产物](https://github.com/HFUT-WDR/wdr_xrobot_dmmc02_framework/actions/runs/42346203050)包含default-Debug-firmware和default-Release-firmware。
各模块运行链接、提交及验证边界见[完成记录](validation/wdr_organization_ci_20261107.json)。
2026-11-06记录保留最初凭据失败的历史状态，本记录为最终验收结果；本轮未烧录或运行机构。

## 本地维护入口

仅维护Modules/HFUT-WDR与module_sources/HFUT-WDR；旧个人模块目录及旧索引不再参与构建。
Modules/xrobot-org 仅保留 SharedTopic 与 SharedTopicClient 两个跟随官方源的模块，不由本组织维护。
个人GitHub仓库保留历史，不再同步维护。已有个人检出的默认origin现指向组织框架；个人远端保留为personal-archive，仅用于追溯。
