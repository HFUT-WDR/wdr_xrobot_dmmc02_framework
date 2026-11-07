# xrobot-org 模块迁移（2026-11-07）

## 范围

- `xrobot-org/BMI088`（基线 `e4e64d705f5b61685e629a91e123ad2febe75fc7`）迁移为
  `HFUT-WDR/BMI088`（私有仓库），dev 提交
  `a65f9d094c8cf0064adc1b344935624f20abc005` 已推送。
- `xrobot-org/BuzzerAlarm`（基线 `a7c17cf7b1d10409848c32bd420cb393515e7f27`）迁移为
  `HFUT-WDR/BuzzerAlarm`（私有仓库），dev 提交
  `0068ba8a216f8a31d55d210828d8cf4fe50f74bb` 已推送。
- 两个仓库保留上游 Git 历史、标签和 LICENSE，新增 `UPSTREAM.md` 记录来源与迁移基线；
  默认分支为 `dev`。
- `HFUT-WDR/Gimbal` 的校准蜂鸣器改提交在 dev
  `087b3130fd32acaf6521459354644bcc0bfed8e8`，已推送。

## 原因

校准提示需要给 BMI088 和 Gimbal 增加可选 `BuzzerAlarm` 依赖，并扩展
`BuzzerAlarm` 的非阻塞发音接口。直接修改 `Modules/` 中的上游模块只在本地
工作区生效；CI 和其他机器执行 `xrobot setup --frozen` 会回到 lock 中的旧提交，
主工程 YAML 依赖新构造参数时会编译失败。迁移到 HFUT-WDR 后才能由本项目维护，
并在推送前用本地 bundle 保证可复现。

## 本地来源与锁

- `Modules/modules.yaml`：`xrobot-org/BMI088`、`xrobot-org/BuzzerAlarm` 改为
  `HFUT-WDR/BMI088`、`HFUT-WDR/BuzzerAlarm`，均为 `same-or-dev`。
- `module_sources/HFUT-WDR/index.yaml` 新增两个模块条目；
  `BMI088.bundle`、`BuzzerAlarm.bundle` 含 `dev`、`master` 和上游标签。
- `xrobot.lock`：三个条目的 `repo` 已切回 GitHub，`source` 为
  `ModuleIndex/index.yaml`，`commit` 为上述已推送的 dev 提交。
- `ModuleIndex/index.yaml` 已登记 `HFUT-WDR/BMI088`、`HFUT-WDR/BuzzerAlarm`
  的 GitHub 地址，ModuleIndex dev 提交 `5800ab7` 已推送。
- 旧 `Modules/xrobot-org/BMI088`、`Modules/xrobot-org/BuzzerAlarm` 已移到
  `.cache/retired_modules/`，不进入主工程版本库。
## 推送状态

1. `HFUT-WDR/BMI088`、`HFUT-WDR/BuzzerAlarm` 已创建为私有仓库，推送了
   `master`、`dev` 和上游标签，默认分支为 `dev`。
2. `HFUT-WDR/Gimbal` 的 dev 提交 `087b313...` 已推送。
3. `ModuleIndex/index.yaml` 已登记两个新模块并推送（dev `5800ab7`）；
   `xrobot setup --update HFUT-WDR/BMI088 HFUT-WDR/BuzzerAlarm HFUT-WDR/Gimbal`
   已将 lock 的 `repo`/`source` 切回 GitHub/ModuleIndex。
4. 主工程的 `ModuleIndex` 子模块指针、`xrobot.lock` 和文档随本次提交更新。

本地 bundle 仍保留在 `module_sources/HFUT-WDR/`，供 `--frozen` 离线恢复和
`tools/check_module_snapshots.py` 使用；它们与已推送的 dev 提交一致。

## 验证

- `xrobot setup --offline`：21 个模块全部检出。
- `python3 tools/check_module_snapshots.py`：19 份快照、21 个模块冻结离线恢复通过。
- `default` 与 `sentry_remote` 的 Debug/Release 构建通过；9 组主机回归通过。
- 默认板级 Debug 已烧录；BMI088 校准、Flash 持久化和复位加载见
  [校准蜂鸣提示](calibration_buzzer_20261107.md)。
