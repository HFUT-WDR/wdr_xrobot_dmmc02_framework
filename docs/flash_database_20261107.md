# Flash 数据库与标定持久化

日期：2026-11-07。本文记录内部 Flash 数据库的接入范围、存储布局和未验证项。

## 变更

1. 保留数据区：STM32H723 的 1 MB Flash 末尾两个 128 KB 扇区
   （0x080C0000-0x080FFFFF）固定给 LibXR `DatabaseRaw` 的主块和备份块，
   `STM32H723xG_flash.ld` 的 FLASH 长度由 1024K 改为 768K，固件不再占用这两个扇区。
2. `User/libxr_config.yaml` 设置 `database.enable: true`、`block_size: auto`，
   重新执行 `libxr stm32 setup -d . --xrobot`。生成的 `User/app_main.cpp` 加入
   `STM32Flash` 和 `DatabaseRaw<STM32Flash::MIN_WRITE_SIZE>`，并以 `database`
   名称注册为 `LibXR::Database`。
3. `User/xrobot.yaml` 和 `User/RobotConfig/sentry_remote.yaml` 中 BMI088 的
   `database` 参数由 `imu_ram_database` 改为 `database`；整车配置的
   `gimbal.calibration_database` 由 `nullptr` 改为 `&database`。云台和 IMU 的
   模块实现未改动，仍通过 `LibXR::Database::Key<T>` 读写。
4. `User/imu_ram_database.hpp` 及其主机回归保留，作为 RAM 后端参考；
   当前两份机器人配置不再注入它。

## 存储布局

| 项目 | 值 |
| --- | --- |
| Flash 扇区表 | 0x08000000 起，8 个 128 KB 扇区（整片 1 MB，见 `User/flash_map.hpp`） |
| 数据库主块 | 0x080C0000-0x080DFFFF |
| 数据库备份块 | 0x080E0000-0x080FFFFF |
| 最小写入单位 | 32 B（STM32H7 FLASHWORD，`STM32Flash::MIN_WRITE_SIZE`） |
| 固件链接上限 | 0x080C0000（768 KB） |

当前镜像大小约为 Debug 248 KB、Release 240 KB，距数据区起点仍有充足余量。
固件增长到 768 KB 时链接会直接失败，不会静默覆盖数据库扇区。

## 行为

- BMI088：`bmi088 cali` 结束后通过 `gyro_data_key_.Set(...)` 写入 Flash；
  上电时 `Database::Key` 构造从数据库加载零偏。空数据库首次构造会先写入
  零初值，这是 LibXR `Key<T>` 的约定行为。
- 云台：CALI 完成后 `ApplyCalibration` 调用 `calibration_key_->Set(c)`，
  `gimbal_status.calibration_saved` 只在写入成功时为 true。数据库内容长度不符
  或标定非法时仍回退 YAML，不覆盖原值。首次上电时键不存在，
  `Database::Key` 会用 YAML 初值建键。
- 掉电：`DatabaseRaw` 使用主备块和写完标记，新增、更新、回收任一时刻掉电后，
  重新上电时每个键保持写入前或写入后的值。
- 首次上电或整片擦除后，空数据库需要初始化主块并写入初值，启动过程会增加
  一次扇区擦写的耗时；整片擦除同样会清空已保存的标定。
- 写入时机：Flash 擦写期间驱动关闭相关 Cache 并阻塞取指。云台 CALI 在停止
  输出后保存；BMI088 `cali` 由终端命令随时触发，接入 Flash 后必须在机构停止
  输出时执行。常规键更新是 32 B 对齐追加，空间回收时才会擦除 128 KB 扇区。
- 已知限制：BMI088 模块不检查 `Set` 的返回值，写入返回 `FULL` 时仍会打印
  `Calibration data saved.`。当前两个小键距空间耗尽很远；云台侧已通过
  `calibration_saved` 如实反映写入结果。

## 验证范围

- 通过：`default` 和 `sentry_remote` 配置的 Debug/Release 构建；
  `board_support`、`imu`、`gimbal`、`chassis`、`control_algorithms`、`i6x`、
  `launcher`、`power` 主机回归。
- 未验证：未烧录、未上板；上电首次建键、真实标定写入、掉电恢复、
  Flash 擦写阻塞时间、云台 `calibration_saved` 的实机表现均未测试。
- 未验证：CubeMX 重新生成后 `.ld` 是否保留 768K 限制。重新生成后必须核对
  `STM32H723xG_flash.ld` 的 FLASH 长度；若被改回 1024K，须手工恢复并重新构建。

## 维护

- 修改 Flash 容量或扇区表时，同时更新生成器产出的 `User/flash_map.hpp` 和
  `.ld` 中的保留区，保持两者一致。
- 不要只提高 `.ld` 的 FLASH 长度而不缩小数据库区；`DatabaseRaw` 初始化或回收
  会擦除末尾两个扇区。
- 需要临时恢复 RAM 存储时，把 YAML 的 `database` 指回 `imu_ram_database`，
  并把 `calibration_database` 改回 `nullptr`；两条路径共用同一套模块代码。
