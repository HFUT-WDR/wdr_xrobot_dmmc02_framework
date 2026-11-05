#!/usr/bin/env python3
"""底盘原算法、真实模块接线与失控保护回归；不访问硬件。"""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
ROOT=Path(__file__).resolve().parents[2]
p=argparse.ArgumentParser();p.add_argument('--modules-dir',type=Path,default=ROOT/'Modules/YunMo-yanyu');args=p.parse_args()
modules=args.modules_dir.resolve();test=Path(__file__).resolve().parent;chassis=modules/'Chassis'
for name, expected in json.loads((test/'reference/algorithm_hashes.json').read_text()).items():
    assert hashlib.sha256((chassis/name).read_bytes()).hexdigest()==expected, f'Frozen algorithm changed: {name}'
with tempfile.TemporaryDirectory(prefix='wdr-chassis-') as temp:
    temp=Path(temp);objects=[]
    flags=['-O1','-g','-fsanitize=address,undefined','-static-libasan','-fno-omit-frame-pointer']
    for i,source in enumerate([chassis/'algorithm/PID.c',chassis/'algorithm/LPF.c',chassis/'algorithm/translation_filter.c',test/'reference/wheel.c']):
        obj=temp/f'{i}.o';subprocess.run(['cc','-std=gnu11',*flags,'-c',str(source),'-o',str(obj)],check=True);objects.append(str(obj))
    inc=[test/'stubs',chassis,modules/'CMD',modules/'RMMotor',modules/'Motor',ROOT/'Middlewares/Third_Party/LibXR/src/structure']
    exe=temp/'chassis'
    subprocess.run(['c++','-std=c++20',*flags,*['-I'+str(i) for i in inc],str(test/'test_chassis.cpp'),str(chassis/'chassis_control.cpp'),*objects,'-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
