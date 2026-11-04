#!/usr/bin/env python3
"""对照参考 SBUS 解码，并验证真实 CMD 的停机和输入来源路由。"""
from pathlib import Path
import subprocess
import tempfile

root = Path(__file__).resolve().parents[2]
reference = Path('/home/yunmo/WorkingSpace/STM32_Project/rm_26_sentry/26-sentry_-dm/User/components/device/control/I6x')
with tempfile.TemporaryDirectory(prefix='wdr-i6x-test-') as temp:
    obj = Path(temp) / 'reference.o'
    exe = Path(temp) / 'i6x_test'
    subprocess.run(['gcc', '-std=c11', '-c', str(reference / 'i6x.c'), '-o', str(obj)], check=True)
    subprocess.run(['g++', '-std=c++20', '-Wall', '-Wextra', '-Werror',
                    '-fsanitize=address,undefined', '-static-libasan', '-g',
                    '-I' + str(root / 'tests/i6x/stubs'),
                    '-I' + str(root / 'Modules/YunMo-yanyu/I6x'),
                    '-I' + str(root / 'Modules/YunMo-yanyu/CMD'),
                    '-I' + str(reference), str(root / 'tests/i6x/i6x_test.cpp'),
                    str(obj), '-o', str(exe)], check=True)
    subprocess.run([str(exe)], check=True)
print('PASS: 1568 frozen-reference vectors; framing, failsafe, timeout, independent CMD routing and STOP')
