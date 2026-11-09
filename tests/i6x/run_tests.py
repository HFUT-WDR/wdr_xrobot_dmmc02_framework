#!/usr/bin/env python3
"""对照参考 SBUS 解码，并验证真实 CMD、HostData 与事件路由。"""
from pathlib import Path
import subprocess
import argparse
import tempfile
import re
import yaml

root = Path(__file__).resolve().parents[2]
parser = argparse.ArgumentParser()
parser.add_argument('--modules-dir', type=Path, default=root / 'Modules/HFUT-WDR')
modules = parser.parse_args().modules_dir.resolve()
reference = Path('/home/yunmo/WorkingSpace/STM32_Project/rm_26_sentry/26-sentry_-dm/User/components/device/control/I6x')
# 检查产品实际绑定和增量模板，防止测试中的手写绑定与烧录配置分离。
for name in ['User/RobotConfig/sentry_remote.yaml', 'config/templates/gimbal.yaml',
             'config/templates/chassis.yaml']:
    instances = yaml.safe_load((root / name).read_text())['modules']
    cmd = next(m for m in instances if m['id'] == 'cmd')
    args = {k: v for item in cmd['args'] for k, v in item.items()}
    assert args['auto_chassis_wz_source'] == 'CMD::ChassisWzSource::RC', name
    rows = set()
    for instance in instances:
        for argument in instance.get('args', []):
            if 'event_binding_groups' in argument:
                rows.update(tuple(x.strip() for x in row) for row in re.findall(
                    r'\{"i6x",\s*([^,]+),\s*"([^\"]+)",\s*([^{}]+)\}',
                    argument['event_binding_groups']))
    if any(m['id'] == 'gimbal' for m in instances):
        for source, target in [('SW2_UP', 'RELAX'), ('SW2_MID', 'ABSOLUTE_ANGLE'),
                               ('SW2_DOWN', 'AUTO')]:
            assert (f'I6x::SwitchHold::{source}', 'gimbal', f'Gimbal::Mode::{target}') in rows, name
        assert ('I6x::Gesture::CALIBRATION_CONFIRMED', 'gimbal',
                'Gimbal::Event::CALIBRATE') in rows, name
        for phase in ['CALIBRATION_SELECTING', 'CALIBRATION_CANCELLED']:
            assert (f'I6x::Gesture::{phase}', 'gimbal', f'Gimbal::Event::{phase}') in rows, name
        assert all(target != 'Gimbal::Mode::CALI' for _, _, target in rows), name
    if any(m['id'] == 'chassis' for m in instances):
        for combination in ['SW2_MID_SW3_DOWN', 'SW2_DOWN_SW3_DOWN']:
            assert (f'I6x::ChordHold::{combination}', 'chassis',
                    'Chassis<Omni>::Mode::NO_FOLLOW_YAW') in rows, name
            assert (f'I6x::Chord::{combination}_EXIT', 'chassis',
                    'Chassis<Omni>::Mode::RELAX') in rows, name
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
print('PASS: actual robot/template bindings; 1568 reference vectors; framing, failsafe, OP fire switches, local AUTO spin, PC expiry, two-stage calibration gesture and recovery')
