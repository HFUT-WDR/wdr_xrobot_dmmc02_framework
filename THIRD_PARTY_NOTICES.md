# 第三方来源与许可声明

本工程由 https://github.com/QDU-Robomaster/bsp-dev-mc02 的 `8c9ee21ace9f7c454a30b81e4edcada857ebd7cd` 演进，保留Git历史和Apache-2.0 LICENSE。
当前独立维护仓库：https://github.com/YunMo-yanyu/wdr_xrobot_dmmc02_framework 。
当前基础组件均由WDR独立维护。官方LibXR/XRobot模块保留原上游。
详见 docs/private_repositories_20261104.md 。

## EventBinder

私有组件 `YunMo-yanyu/EventBinder` 基于 [原始 EventBinder](https://github.com/QDU-Robomaster/EventBinder)，
迁移基线为 `3d4adde3a7c10152a6a0b6c67738b88c972bd675`。原许可证与 Git 历史保留在组件仓库。

## IMU移植

新增 YunMo-yanyu/ImuFusion、BoardAttitude、DmImu。原始行为参考26_sentry的板载与DM专用EKF、DM协议；
本次复用MC02_Framework_CPP修正后的移植，保留原作者及来源，未把旧HAL入口搬入XRobot模块。
代码基线、修复和文件散列见各组件THIRD_PARTY_NOTICES.md与tests/imu/reference。
引用的26_sentry快照未附工程根许可证，本工程不替其推定新的许可；组件LICENSE只覆盖WDR新增部分。
CMSIS-DSP使用基座已有Drivers/CMSIS版本及原许可证，未复制或修改第三方源码。
