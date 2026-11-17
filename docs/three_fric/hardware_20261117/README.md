# 2026-11-17 烧录与启动验证

Debug固件备份校验、烧录校验、复位启动通过，当前保留新固件。
运行中SWD采样约2.55秒，控制序号增长2550、系统tick增长2552ms；550MHz、空闲堆59544字节，CFSR/HFSR均为0。
发射STOP、四路电流零、遥控及电机反馈均离线。未发送任何使能/开火或RAM覆盖指令。
本次没有验证遥控、CAN电机通信、方向、PID瞬态或实际机构运行。完整备份位于build/hardware_20261117/pre_flash.bin（1MiB）。
详见result.json、flash.log与runtime_samples.json。
