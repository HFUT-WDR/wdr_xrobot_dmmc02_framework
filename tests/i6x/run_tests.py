#!/usr/bin/env python3
"""对照参考 SBUS 解码，并验证真实 CMD、HostData 与事件路由。"""
from pathlib import Path
import subprocess
import argparse
import tempfile

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--modules-dir', type=Path, default=root / 'Modules/YunMo-yanyu')
modules = parser.parse_args().modules_dir.resolve()
reference = Path('/home/yunmo/WorkingSpace/STM32_Project/rm_26_sentry/26-sentry_-dm/User/components/device/control/I6x')
with tempfile.TemporaryDirectory(prefix='wdr-i6x-test-') as temp:
    obj = Path(temp) / 'reference.o'
    exe = Path(temp) / 'i6x_test'
    subprocess.run(['gcc', '-std=c11', '-c', str(reference / 'i6x.c'), '-o', str(obj)], check=True)
    subprocess.run(['g++', '-std=c++20', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-static-libasan', '-g',
                    '-I' + str(root / 'tests/i6x/stubs'),
                    '-I' + str(modules / 'I6x'),
                    '-I' + str(modules / 'CMD'), '-I' + str(modules / 'HostData'),
                    '-I' + str(modules / 'EventBinder'), '-I' + str(reference), str(root / 'tests/i6x/i6x_test.cpp'),
                    str(obj), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('PASS: 1568 frozen-reference vectors; framing, failsafe threshold, CMD routing, HostData zero targets, independent PC expiry and STOP')
