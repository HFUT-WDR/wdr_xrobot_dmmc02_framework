> 本文为初版实现记录；当前架构以cmd_migration_20261117.md为准，独立Task已移除。

# 三摩擦发射实施记录

## 当前任务契约

- 目标：三台无减速箱 M2006 摩擦轮和一台 36:1 M2006 拨弹，接 I6x/CMD，独立 1 ms 运行。
- 范围：本工程 LocalModules 下的发射机构模块和薄调度适配模块、实例 YAML、模块请求/锁定、必要的测试和使用文档。
- 非目标：修改原 MC02_Framework_CPP、现有双轮 Launcher、底盘/云台、热量/裁判/弹速调节、发布/推送/烧录。
- 允许：初始化依赖、生成入口、Debug/Release 构建、离线回归。
- 行为：三轮到速锁存；各电机独立零速刹停并锁存释放；掉线电机零电流且不读旧反馈制动；任一掉线撤销整个机构开启，恢复需重新操作。
- 输入：CMD 提供 fire；I6x 边沿提供摩擦轮切换；独立调度模块提供运行许可，不依赖 Gimbal。此独立产品仅配置 OP，切入 AUTO 时停止。
- 时序：构造发射模块不创建线程、不发 CAN；薄调度模块最后构造，绑定事件后启动线程；每拍 Update 后 Send，组内四路同拍提交。
- 验证预算：机构主机回归（到速、停机、掉线/恢复、卡弹、滤波和 PID 基线）、真实 CMD/驱动集成测试、生成配置、Debug/Release 构建；无烧录和物理机构结论。
- 停止条件：上述软件检查通过；外部依赖不可用则保留可复现错误和未验证边界。
- 执行面：当前指定目录本地工作树；不修改其他工作树。

## 已接受决策与讨论记录

1. 独立 ThreeFricLauncher，同步 Update/Send，不扩大既有双轮 Launcher 的硬件约束。
2. 三摩擦停止语义优先；借鉴 WDR 对掉线电机零电流处理。
3. WDR 现有 OP/AUTO 权限无问题，本任务不修改它。
4. CMD 不依赖底盘/云台存在；独立发射产品另有薄调度适配模块，避免塞入虚构云台或改生成器。
5. 原 ShootTask 的 PID 按离散逐拍积分，仅移植实际使用的运算；不直接替换为 WDR PID。
6. 本地新模块以独立 Git 源仓库保存于 LocalModules，XRobot 从本地索引检出至 Modules；未授权发布。

## 基线和变化

源：/home/yunmo/.codex/worktrees/three-fric-test/MC02_Framework_CPP 当前 shoot_task.cpp / shoot_config.hpp（有未提交修改，仅读取）。
参考：WDR-Xrobot-org/Launcher 23c4762 的模块接口与掉线处理。

保持：CAN1 ID1拨弹、ID2左、ID3右、ID4上；40mm、三轮21m/s；摩擦PID 4000/50/0、I限500；拨弹2000/5/0、I限1000；输出±10000；1ms；卡弹1000拍、反转500拍与13rad/s。
变化：掉线电机由使用旧速度刹车改成零电流；反馈恢复前后保持零电流至重新开启。健康状态不再由 TOE 全局编号提供，改用驱动真实收帧时效。
输入映射沿用 WDR I6x 当前 raw 值：SW1=-1边沿切换；SW0=-1且SW1=+1允许拨弹。旧项目 UP/DOWN 宏对同一 raw 值的称呼相反，文档以 WDR 的物理标注和 raw 值说明。

## 进度

完成：机构模块与调度适配、本地源索引、产品配置、锁定、主机回归、最终Debug/Release构建均完成。当前阶段为本地交付；待用户安排板级和机构验收。


## 最终验证与交付

- 机构源提交：9c7acac；调度源提交：a28f868。下载检出、xrobot.lock与源提交一致。
- xrobot setup --frozen --offline：11个模块提交、2份配置通过，当前选择User/three_fric.yaml，生成头无变化。
- 主机回归通过：真实CMD/I6x事件/RMMotor/LibXR；测试替代CAN与时钟，拦截线程启动后调用真实Step。原PID冻结副本SHA-256检查通过，三轮与拨弹PID逐拍对照通过。
- 最终Debug/Release均重新编译链接通过，ELF含机构Update与独立任务ThreadFunc。DTCM占用均94720B/128KiB；Flash分别150964B、151128B。
- 格式检查与git diff --check通过。证据与最终ELF哈希见validation/evidence.json及同目录日志。
- 工具更新本地源时仍会fetch远端依赖，曾遇SSH断连；最终仅对该次进程设置Git本地镜像映射，使用已检出的依赖完成更新，未修改全局Git设置、依赖版本或生成器。最终离线冻结恢复通过。
- 保留1 LSB归一化截断差；其他已声明变化为过期反馈零电流、按真实反馈时效门控及独立OP产品调度。空0x1FF帧不再发送。
- 未改原三摩擦工程、原双轮Launcher和其他工作树；未推送、发布或烧录。未进行独立代码审查或硬件验收。

## 精简检查

- ThreeFricLauncher：机构算法消费者，不可删除。
- ThreeFricLauncherTask：无Gimbal产品的唯一周期调度和输入许可消费者，不可删除；没有机构算法副本。
- LocalModules与本地索引：未发布模块的维护源与可复现下载入口，不可只保留被忽略的Modules目录。
- tests/reference：当前原PID的冻结数值参照，只链接主机测试；没有额外算法库或固件兼容层。
- 参数没有引入热量、裁判、上位机、数据库或通用控制框架；方向只由RMMotor转换。

## 后续授权：烧录测试

用户追加授权烧录测试。已识别UID、备份整片Flash、烧录Debug并通过校验，运行只读采样正常；没有注入机构控制命令。遥控和四电机未收到有效数据，等待用户确认供电/连接；随后调试器USB断开，服务已关闭。最后已知固件为新Debug；当前未完成机构验收。证据在hardware_20261116，备份留在build/hardware_20261116。

## 2026-11-17 公共滤波器适配契约

用户授权将当前发射模块适配ControlAlgorithms已有SecondOrderLowPass；本轮只改拨弹反馈滤波及依赖、参数、测试、文档，不合并调度模块或迁移PID，不烧录。固定采样率1000Hz，截止频率取旧递推的-3dB频率，作为YAML显式参数。改为标准二阶Butterworth，保留零历史启动与停止复位，但不宣称与旧递推逐拍等价。软件验收为公共滤波器实际调用、解析/生成、机构回归、Debug/Release；已有板上旧固件证据仅归属于旧版本。

公共滤波器适配已完成：ThreeFricLauncher提交e3d8e82，ControlAlgorithms锁定f9ec82e。12个模块/2份配置生成与离线冻结恢复通过。独立直接一型数值参照、零历史复位和机构回归通过；最终Debug/Release通过。单位阶跃首拍增益由0.03023791改为0.006845234，带宽匹配但瞬态不同。本轮未烧录，板上仍是上次Debug版本；证据位于validation_filter_20261117。

## 2026-11-17 公共滤波dt接口契约

用户授权修改公共二阶低通：保留Update(input)按构造/YAML采样率；新增Update(input, dt_seconds)按当前dt设计系数。单拍dt不覆盖配置默认值；切换系数从已保存输入/输出历史重建延迟状态，不复位。无效dt或违反Nyquist时沿用公共滤波器错误语义：Reset并返回零，不静默使用默认值。发射配置显式提供默认采样率，任务用微秒时钟测实际周期；首拍无前次时刻，按名义1ms调度周期。原PID和卡弹计拍语义保持，本轮不烧录。ControlAlgorithms在本工程LocalModules独立维护，原工作树不修改。验收：旧固定接口逐拍兼容、变dt独立递推对照、默认恢复、复位与异常周期、机构回归、配置生成及Debug/Release。

dt适配验收完成：ControlAlgorithms 453e935、ThreeFricLauncher da20d13、调度模块09a9fc2，源/下载检出与锁定一致。旧固定接口2000拍逐值相等；变dt与混用配置默认值通过独立直接一型对照；Reset和非法dt处理通过。两项CTest、Debug/Release、生成与离线冻结恢复通过。本次未烧录。证据在validation_dt_20261117。公共算法源码维护位置为本工程LocalModules/ControlAlgorithms，未改其他工作树或远程仓库。
