#!/usr/bin/env python3
"""功率算法与超电协议离线回归，不访问板卡。"""
import argparse
from pathlib import Path
import subprocess
import tempfile
root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('--modules-dir', type=Path, default=root/'Modules/HFUT-WDR')
args = parser.parse_args()
module = args.modules_dir.resolve()/'PowerControl'
with tempfile.TemporaryDirectory(prefix='wdr-power-') as temp:
    reference = root/'tests/power/reference'
    import hashlib, json
    frozen = (reference/'chassis_power_control.c').read_text()
    assert hashlib.sha256(frozen.encode()).hexdigest() == json.loads((reference/'source.json').read_text())['sha256']
    tail = frozen[frozen.index('    allocatablePower = chassis_max_power;'):]
    assert tail in (reference/'allocation.c').read_text()
    rls = frozen[frozen.index('void Power_RLS_update('):frozen.index('Power_RLS_t *get_Power_RLS_point')]
    assert rls in (reference/'allocation.c').read_text()
    obj = Path(temp)/'reference.o'
    subprocess.run(['cc', '-std=c11', '-O1', '-c', str(reference/'allocation.c'), '-o', str(obj)], check=True)
    exe = Path(temp)/'power'
    subprocess.run(['c++', '-std=c++20', '-DLIBXR_DEFAULT_SCALAR=float', '-O1', '-g', '-fsanitize=address,undefined',
                    '-static-libasan', '-fno-omit-frame-pointer',
                    '-I'+str(root/'tests/chassis/stubs'), '-I'+str(module),
                    '-I'+str(root/'Middlewares/Third_Party/LibXR/src/structure'),
                    str(root/'tests/power/test_power.cpp'), str(module/'chassis_power.cpp'),
                    str(obj), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
