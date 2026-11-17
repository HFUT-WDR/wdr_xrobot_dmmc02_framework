#!/usr/bin/env python3
"""验证三摩擦发射软件行为；无硬件副作用。

用法：python3 tests/three_fric/run_tests.py
输入：锁定模块中的机构源码、冻结PID参照、已恢复的LibXR/CMD/I6x/RMMotor。
输出：build/three_fric_host构建与CTest结果；可重复执行，不改产品配置。
依赖：CMake、Ninja、C++20主机编译器；不烧录、不发送实体CAN。
"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess

parser = argparse.ArgumentParser()
parser.add_argument('--integration-modules', default='')
args = parser.parse_args()

root = Path(__file__).resolve().parents[2]
source = root / 'Modules/WDR-Xrobot-org/ThreeFricLauncher/tests'
reference = source / 'reference'
for name, digest in json.loads((reference / 'hashes.json').read_text()).items():
    if hashlib.sha256((reference / name).read_bytes()).hexdigest() != digest:
        raise SystemExit(f'Frozen PID reference changed: {name}')
build = root / 'build/three_fric_host'
for command in (
    ['cmake', '-S', str(source), '-B', str(build), '-G', 'Ninja', '-DCMAKE_BUILD_TYPE=Debug', f'-DBSP_ROOT={root}', f'-DINTEGRATION_MODULES={args.integration_modules}'],
    ['cmake', '--build', str(build), '-j4'],
    ['ctest', '--test-dir', str(build), '--output-on-failure'],
):
    subprocess.run(command, check=True)
