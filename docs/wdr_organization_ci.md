# WDR组织仓库与CI

## 仓库与可见性

HFUT-WDR下新建框架、18个自维护模块和xrobot-ci，共20个私有仓库；个人仓库继续保留。
组织成员通过组织默认Read权限读取，维护者保留管理权限。依赖和冻结快照统一使用HFUT-WDR命名空间。
ModuleIndex子模块指向HFUT-WDR/xrobot-ci；其中index.yaml提供模块HTTPS地址。
module_sources/HFUT-WDR保存本次锁定版本的Git bundle，可用tools/check_module_snapshots.py离线验证。

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

GITHUB_TOKEN通常只能读取当前仓库，无法拉取另一个私有模块。专用Fine-grained Token为此提供Contents Read-only。
资源所有者为HFUT-WDR，仅选本次20个新仓库；Metadata保持默认Read-only。
Token作为WDR_MODULES_READ_TOKEN存储到18个模块和框架的Actions Secrets，通过workflow_call显式传给共享工作流。
共享工作流仓库需允许组织内私有仓库调用。ci/credential_helper.py仅对github.com/HFUT-WDR的HTTPS Git读取返回凭据，不把Token写进Git配置或日志。

更新凭据：在工程终端执行python3 tools/configure_wdr_ci.py，在隐藏输入中粘贴Token。
检查名称：python3 tools/configure_wdr_ci.py --check。此检查无法验证Token权限或有效期，需实际CI拉取成功才能确认。
令牌到期、撤销或新增模块时需要更新选择范围及各调用仓库Secret。不要将Token写入文件、提交或聊天。

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
个人GitHub仓库保留历史，不再同步维护。已有个人检出的默认origin现指向组织框架；个人远端保留为personal-archive，仅用于追溯。
