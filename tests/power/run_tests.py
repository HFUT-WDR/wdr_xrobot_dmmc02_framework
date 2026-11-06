#!/usr/bin/env python3
"""功率算法与超电协议离线回归，不访问板卡。"""
import argparse
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--modules-dir', type=Path, default=root/'Modules/YunMo-yanyu')
args = parser.parse_args()
module = args.modules_dir.resolve()/'Chassis'
with tempfile.TemporaryDirectory(prefix='wdr-power-') as temp:
    exe = Path(temp)/'power'
    subprocess.run(['c++', '-std=c++20', '-DLIBXR_DEFAULT_SCALAR=float', '-O1', '-g', '-fsanitize=address,undefined',
                    '-static-libasan', '-fno-omit-frame-pointer',
                    '-I'+str(root/'tests/chassis/stubs'), '-I'+str(module),
                    '-I'+str(root/'Middlewares/Third_Party/LibXR/src/structure'),
                    str(root/'tests/power/test_power.cpp'), str(module/'chassis_power.cpp'),
                    '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
