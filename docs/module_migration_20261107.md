# xrobot-org 模块迁移（2026-11-07）

## 范围

- `xrobot-org/BMI088`（基线 `e4e64d705f5b61685e629a91e123ad2febe75fc7`）迁移为
  `HFUT-WDR/BMI088`，本地 dev 提交 `a65f9d094c8cf0064adc1b344935624f20abc005`。
- `xrobot-org/BuzzerAlarm`（基线 `a7c17cf7b1d10409848c32bd420cb393515e7f27`）迁移为
  `HFUT-WDR/BuzzerAlarm`，本地 dev 提交 `0068ba8a216f8a31d55d210828d8cf4fe50f74bb`。
- 两个仓库保留上游 Git 历史和 LICENSE，新增 `UPSTREAM.md` 记录来源与迁移基线。
- `HFUT-WDR/Gimbal` 已在 HFUT-WDR 名下；本次校准蜂鸣器改动提交在本地 dev
  `087b3130fd32acaf6521459354644bcc0bfed8e8`，尚未推送。

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
- `xrobot.lock`：BMI088、BuzzerAlarm、Gimbal 三个条目的 `repo` 和 `source`
  暂指向 `module_sources/HFUT-WDR/*.bundle` 与本地索引，`commit` 为上述 dev 提交。
- 旧 `Modules/xrobot-org/BMI088`、`Modules/xrobot-org/BuzzerAlarm` 已移到
  `.cache/retired_modules/`，不进入主工程版本库。
- `ModuleIndex/index.yaml` 尚未加入新仓库；在 GitHub 仓库创建并推送前，
  不要把它指向不存在的地址。

## 待完成（需要仓库推送权限）

1. 在 `HFUT-WDR` 下创建 `BMI088`、`BuzzerAlarm` 仓库，推送 `master` 与 `dev`
   （保留上游历史和 `UPSTREAM.md`）。
2. 推送 `HFUT-WDR/Gimbal` 的本地 dev 提交 `087b313...`。
3. 在两个仓库可访问后，把 `ModuleIndex/index.yaml` 的条目改为 GitHub 地址，
   执行 `xrobot setup --update HFUT-WDR/BMI088 HFUT-WDR/BuzzerAlarm HFUT-WDR/Gimbal`，
   把 lock 的 `repo`/`source` 切回 GitHub/ModuleIndex。
4. 推送 ModuleIndex 与主工程更新后的 `xrobot.lock`。

在此之前，本地 bundle 是 CI `--frozen` 与离线恢复的来源。

## 验证

- `xrobot setup --offline`：21 个模块全部检出。
- `python3 tools/check_module_snapshots.py`：19 份快照、21 个模块冻结离线恢复通过。
- `default` 与 `sentry_remote` 的 Debug/Release 构建通过；9 组主机回归通过。
- 默认板级 Debug 已烧录；BMI088 校准、Flash 持久化和复位加载见
  [校准蜂鸣提示](calibration_buzzer_20261107.md)。
