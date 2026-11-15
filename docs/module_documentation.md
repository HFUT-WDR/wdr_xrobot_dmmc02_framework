# 模块文档约定

模块 README 面向第一次使用该模块的开发者，说明能力、接入方法与当前接口。迁移过程、整车接线和验证记录分别维护，避免读者依赖开发聊天或另一个工程才能理解模块。

模块 API 的组织方式和改造范围见[模块开发规范](module_development_standard.md)与[逐模块 API 改造方案](module_api_migration_plan.md)。P0/P1 已在本地实施，具体范围及证据见[实施记录](validation/module_api_p0_p1_20261115.md)；P2 仍为后续方案。

## 统一版式

README 参照 [xrobot-org/MadgwickAHRS](https://github.com/xrobot-org/MadgwickAHRS/blob/master/README.md) 的“先行为、后接口、再接入”顺序。2026-11-15 对齐索引中全部 21 个 WDR 模块；改动与校验见[对齐记录](validation/module_readme_alignment_20261115.md)。章节统一使用以下编号和双语标题，正文以中文为主，已有准确英文说明可以保留：

1. `模块作用 / Purpose`：能力、输入输出、支持范围，构造和运行行为。
2. `运行约定 / Runtime Conventions`：按模块需要分设时间戳、单位/坐标系、命令、事件、模式、线程及失效处理子节。
3. `构造接口 / Constructor`：当前 C++ 签名，依赖对象与完整参数说明；区分源码默认值、必填配置和 BSP 覆盖值。
4. `Topic`：默认名称、方向、C++ 类型、载荷含义；无 Topic 时明确说明实际输入输出接口。
5. `配置示例 / Configuration Example`：添加模块到生成入口的完整教学，以及直接可读的完整 YAML；纯库改为 include 与最小调用示例。
6. `依赖与硬件 / Dependencies and Hardware`：模块依赖、外部库、支持平台、外设与前置资源。
7. `来源与许可 / Provenance and License`：来源摘要及许可证/第三方声明链接。

Madgwick 的独立时间戳和 RamFS 章节在 WDR 版式中归入运行约定，保留五项核心内容并集中放置来源。不得仅为凑齐标题声称模块拥有不存在的线程、命令或 Topic。

## README 内容

1. 开头说明用途、输入输出和适用范围。库模块明确无需 YAML 实例化，驱动和机构模块明确支持的设备或拓扑。
2. README 必须直接包含模块使用教学：添加模块、获取模块、添加实例、填写依赖与参数、重新生成入口。写明依赖对象、BSP 资源与实例顺序；YAML 增量示例必须说明前置对象，参数示例不等于通用实机参数。
3. 参数注明单位、默认值、坐标系和量程。较长参数表可链接本仓库内的接口定义，但关键硬件参数应直接说明。
4. 输入输出写明 Topic 名称、类型和方向；事件写明编号类型、边沿/保持语义及回调上下文。
5. 说明构造是否创建线程或发送、周期由谁负责、缓冲和对象生命周期、超时与输出许可。
6. 文末集中说明来源与许可，并链接 THIRD_PARTY_NOTICES.md。来源于26_sentry的模块在此说明一次，正文直接描述当前算法和行为；其他来源按事实分别注明。

当前生成器要求 YAML 显式列出构造参数；参数对象采用 YAML 映射时应列全成员。示例中的 `'{}'` 是 C++ 默认聚合初始化表达式，不能把它与省略映射字段混为一谈。

无 Topic 的模块只说明无 Topic 和实际调用入口，不填写空表。双语标题统一；无需为了版式逐段复制英文。

## README 中的使用教学

使用教学直接写在模块 README 中，不能只链接 `example/`、`examples/` 或其他示例文件来代替。附加示例工程可以保留，但读者只阅读 README 就应能完成首次接入。

每个可实例化模块的 README 必须依次包含：

1. 前置条件：支持的平台、工具版本、模块源与私有仓库访问要求、需要注册的资源及前置实例。
2. 在 BSP 根目录执行的真实命令：`xrobot module add` 添加请求，`xrobot setup` 获取模块，再用 `xrobot instance add` 添加实例。实例命令拼写为 `instance`。
3. 对应 `User/xrobot.yaml` 的完整实例参数示例，包含 module、id、args；参数映射按声明顺序列全字段，不用省略号隐藏必需配置。
4. 紧邻示例的参数表：名称、类型、默认值或必填、单位、作用和需要按硬件修改的内容。无默认值写“必填”，示例值与源码默认值分开说明。
5. 配置后重新生成的命令，以及该模块可观察的接入结果，例如命令、Topic 或状态。构建、烧录步骤可引用 BSP 文档；不得把构造会驱动机构的模块描述成无副作用的试运行。

以下以 BuzzerAlarm 展示教学形式。前提是 BSP 已配置 WDR 模块源并注册 `pwm_tim12_ch2`；其他模块的 README 必须替换为自己的真实资源和完整参数，不能原样复制这个例子。

```sh
xrobot module add WDR-Xrobot-org/BuzzerAlarm@same-or-dev
xrobot setup
xrobot instance add WDR-Xrobot-org/BuzzerAlarm --id buzzer
```

添加实例后，在 `User/xrobot.yaml` 中将 PWM 依赖填为已注册的对象：

```yaml
modules:
  - module: WDR-Xrobot-org/BuzzerAlarm
    id: buzzer
    args:
      - pwm: pwm_tim12_ch2
      - alarm_freq: 1500
      - alarm_duration: 300
      - alarm_delay: 300
```

已有 modules 列表时只追加实例，不创建第二个 modules 根键。

| 参数 | 类型 | 默认值 / 必填 | 单位 | 作用与填写要求 |
| --- | --- | --- | --- | --- |
| pwm | LibXR::PWM& | 必填 | — | BSP 注册的蜂鸣器 PWM 对象；示例资源名不是模块默认值 |
| alarm_freq | uint32_t | 1500 | Hz | 致命错误报警频率 |
| alarm_duration | uint32_t | 300 | ms | 每次报警的鸣叫时长 |
| alarm_delay | uint32_t | 300 | ms | 每次报警鸣叫后的等待时间 |

填写后运行：

```sh
xrobot gen -c User/xrobot.yaml
```

再按 BSP 文档构建与烧录。BuzzerAlarm 构造时会播放开机提示音并注册致命错误回调，约阻塞 800 ms；提示音只能作为该模块已执行构造的观察，不能证明其他模块或机构已通过验收。

`standalone: false` 的纯库模块不执行 `instance add`。其 README 仍需直接给出添加/获取模块的方法、include 与最小调用示例，以及对应参数或类型说明，明确无需 YAML 实例化。

## 独立文档

| 内容 | 位置 |
| --- | --- |
| 上游、作者、原文件、提交及许可边界 | THIRD_PARTY_NOTICES.md |
| 旧接口、移植差异、行为调整与兼容要求 | docs/migration.md |
| 详细协议、结构偏移、状态机 | docs/protocol.md 或对应设计文档 |
| 具体车体总线、ID、方向、PID、零位和拨杆绑定 | 整车 YAML 与整车使用说明 |
| 构建、离线测试、烧录和机构验证结果 | 对应验证记录 |

“沿用原车”“同原工程”“删除旧模式”等说明不能代替当前行为定义。保留影响使用的限制，例如CAN组帧协调、角度单位、PID参数基准和实际支持的轮型；不因删除来源叙述而扩大能力声明。

## 更新检查

- 接口以当前源码为准，核对构造参数、枚举、默认值、消息布局和线程行为。
- 本仓库内的链接应可解析，示例不引用已删除的配置或旧事件枚举。
- YAML 示例应通过生成和编译检查；库示例按实际工程选项检查 C++ 语法。
- 可实例化模块的 README 包含添加模块、获取模块、添加实例、完整 YAML、参数表和生成步骤；不能仅以示例目录链接通过检查。
- 不在 README 中声称未完成的实机收发、功率合规或机构闭环验证。
- 文档改动单独提交，不夹带控制行为修改。
