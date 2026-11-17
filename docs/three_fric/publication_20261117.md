# 三摩擦发射模块发布记录（2026-11-17）

模块发布至 WDR-Xrobot-org 各自仓库的 dev 分支，ThreeFricLauncher README 按现有模块的七节形式整理，含参数表、完整 YAML 示例、Topic、依赖与来源说明。

| 模块 | 发布提交 |
| --- | --- |
| CMD | `8f6533e472283f6c1e9bbed3fcf7beac9bb5df22` |
| ControlAlgorithms | `453e9356c3116449025641c519f954e398180b5f` |
| DR16 | `55fad6dac55bcab6ee03e7064146bc57ee257a28` |
| Gimbal | `871d79760b13c457c046651a3243bfcaaebcf2b6` |
| HostData | `00dff54c07366eef6cdbbe53ea7a30711bec02ee` |
| I6x | `4622d91898c51918a740a80d7f722d019fbb34fd` |
| Launcher | `643b50461ed7ae899f7309126ad717bcb8555f88` |
| Referee | `49d2839752791f3169321b39ddbc4873cde462ac` |
| ThreeFricLauncher | `250121756c74a0ef495341e061f8eea43fafbd09` |

模块索引更新为 `47cdbba`。工程在 `feat/three-fric-launcher` 分支固定这些远程依赖；LocalModules 和本地索引已移出源码树，备份位于忽略的 build/publication_backups 下。使用 xrobot setup 生成锁文件，未手工修改锁文件或生成头文件。

移除本地目录后，`xrobot setup --frozen --offline` 通过（13 个模块、2 个配置），行为测试与公共滤波器测试 2/2 通过，Debug 与 Release 均构建成功。合并上游后的 Gimbal、HostData、DR16 兼容编译通过。

烧录与启动证据见 hardware_20261117；发布阶段未再次烧录，也未验证电机转动或发射。远程 CI 结果尚未作为本次本地验收依据。
