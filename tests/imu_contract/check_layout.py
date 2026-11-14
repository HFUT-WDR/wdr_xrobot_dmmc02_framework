#!/usr/bin/env python3
"""校验 wdr_imu::Attitude 的多份定义布局一致且姿态角为 roll,pitch,yaw。

组织内 AttitudeEkf、ImuEkfFusion、ESKF 各有一份 imu_types.hpp，同名同命名空间。
它们靠 Topic 裸结构发布，任何一份顺序不同都会造成 ODR 冲突或载荷错位，而编译器
不会报错。本脚本分别编译三份头文件，比对 sizeof 与各成员 offsetof。

只依赖 C++ 编译器与模块源码；不链接 LibXR，也不访问外设。
"""
from pathlib import Path
import subprocess
import sys
import tempfile

ROOT = Path(__file__).resolve().parents[2]
MODULES = ROOT / "Modules" / "WDR-Xrobot-org"

# 各 Attitude 定义的所在仓与额外包含路径。AttitudeEkf 的 imu_types.hpp 会拉入
# C 核 kalman_filter.h，主机侧需要它的 arm_math 替身（模块自带的主机回归桩）。
AEKF = MODULES / "AttitudeEkf"
TARGETS = [
    ("AttitudeEkf", AEKF,
     [AEKF / "vendor" / "CMSIS" / "Core" / "Include", AEKF / "tests" / "stubs"]),
    ("ESKF", MODULES / "ESKF", []),
]

PROBE = """#include <cstddef>
#include <cstdio>
#include "imu_types.hpp"
int main()
{
  std::printf("%zu %zu %zu %zu %zu %zu\\n", sizeof(wdr_imu::Attitude),
              offsetof(wdr_imu::Attitude, roll), offsetof(wdr_imu::Attitude, pitch),
              offsetof(wdr_imu::Attitude, yaw), offsetof(wdr_imu::Attitude, gyro),
              offsetof(wdr_imu::Attitude, accel));
  return 0;
}
"""


def layouts():
    """返回 {仓名: (size, roll, pitch, yaw, gyro, accel)}，编译失败时抛出异常。"""
    out = {}
    with tempfile.TemporaryDirectory(prefix="imu-contract-") as directory:
        tmp = Path(directory)
        probe = tmp / "probe.cpp"
        probe.write_text(PROBE)
        for name, repo, extra in TARGETS:
            if not (repo / "imu_types.hpp").is_file():
                raise SystemExit(f"缺少 {repo / 'imu_types.hpp'}")
            exe = tmp / name
            subprocess.run(["c++", "-std=c++20", "-Wall", "-Wextra", "-Werror",
                            f"-I{repo}", *(f"-I{d}" for d in extra),
                            str(probe), "-o", str(exe)], check=True)
            text = subprocess.run([str(exe)], capture_output=True, text=True,
                                  check=True).stdout
            out[name] = tuple(int(v) for v in text.split())
    return out


def main():
    # ImuEkfFusion 是独立仓库，可能未检出；检出时一并比对。
    fusion = MODULES / "ImuEkfFusion"
    if (fusion / "imu_types.hpp").is_file():
        TARGETS.append(("ImuEkfFusion", fusion, []))
    result = layouts()
    for name, values in result.items():
        size, roll, pitch, yaw, gyro, accel = values
        # 姿态角必须按 roll,pitch,yaw 递增排列且三者紧邻。
        assert roll < pitch < yaw, f"{name}: 姿态角不是 roll,pitch,yaw 顺序"
        assert pitch - roll == 4 and yaw - pitch == 4, f"{name}: 姿态角不连续"
        assert gyro > yaw and accel > gyro, f"{name}: 三分量数组位置异常"
        print(f"  {name}: size={size} roll@{roll} pitch@{pitch} yaw@{yaw} "
              f"gyro@{gyro} accel@{accel}")
    reference = result["AttitudeEkf"]
    for name, values in result.items():
        assert values == reference, (
            f"{name} 的 Attitude 布局与 AttitudeEkf 不一致：{values} != {reference}")
    print(f"PASS: {len(result)} 份 Attitude 布局一致，姿态角为 roll,pitch,yaw")


if __name__ == "__main__":
    sys.exit(main())
