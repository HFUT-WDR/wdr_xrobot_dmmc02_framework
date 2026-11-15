#!/usr/bin/env python3
"""迁移2026-11-15的七个WDR模块YAML接口，默认只显示差异。

用法：python tools/migrate_module_api.py User/xrobot.yaml [--apply]
输入：旧版XRobot YAML文件（可传多个）；输出：差异与修改的实例数。
依赖：Python 3、ruamel.yaml。副作用：仅--apply覆盖显式给出的文件；不生成、不构建、不访问设备。
幂等：新接口实例不重复处理；未知参数或非映射Param报错，不猜测C++表达式。
示例：先不带--apply审阅差异，再加--apply落盘。
"""
import argparse
import copy
import difflib
import io
from pathlib import Path

from ruamel.yaml import YAML
from ruamel.yaml.comments import CommentedMap
from ruamel.yaml.scalarstring import DoubleQuotedScalarString as String


FUSION_NAMES = ('gyro_topic_name', 'accel_topic_name', 'output_topic_name',
                'diagnostics_topic_name', 'command_name')
ESKF_DEFAULTS = {
    'dt_source': 'wdr_imu::DtSource::MEASURED', 'nominal_period_s': '0.001f',
    'task_stack_depth': 4096, 'accel_filter_enabled': True,
    'accel_filter_tau_s': '0.05f', 'gyro_noise_density': '0.003f',
    'bias_random_walk': '0.0001f', 'accel_direction_std': '0.03f',
    'accel_norm_soft_mps2': '0.35f', 'accel_norm_reject_mps2': '1.5f',
    'innovation_soft': '5.991f', 'innovation_reject': '25.0f',
    'max_direction_error_rad': '1.04719755f', 'gravity_update_interval': 1,
    'initialization_samples': 200, 'recovery_samples': 50,
    'rest_bias_enabled': False, 'rest_gyro_threshold': '0.015f',
    'rest_accel_std': '0.08f', 'rest_samples': 500, 'rest_bias_std': '0.003f',
}
EXTRAS = {
    'DmImu': dict(installation='wdr_imu::IDENTITY', gyro_topic_name=String('dm_gyro'),
                  accel_topic_name=String('dm_accl'), raw_topic_name=String('dm_measurement'),
                  diagnostics_topic_name=String('dm_diagnostics'), command_name=String('dm_imu')),
    'Chassis': dict(command_name=String('chassis'), command_topic_name=String('chassis_cmd'),
                   attitude_topic_name=String('board_attitude'),
                   gimbal_topic_name=String('gimbal_chassis_feedback'),
                   output_topic_name=String('chassis_status'), task_stack_depth=4096,
                   thread_priority='LibXR::Thread::Priority::MEDIUM',
                   referee_topic_name=String('robot_game_ref')),
    'Gimbal': dict(calibration_key=String('gimbal_yaw_calibration'),
                  command_topic_name=String('gimbal_cmd'), board_topic_name=String('board_attitude'),
                  dm_topic_name=String('dm_attitude'), context_topic_name=String('gimbal_context'),
                  referee_topic_name=String('robot_game_ref'),
                  chassis_topic_name=String('gimbal_chassis_feedback'),
                  status_topic_name=String('gimbal_status'), task_stack_depth=4096,
                  thread_priority='LibXR::Thread::Priority::MEDIUM'),
    'Launcher': dict(fire_topic_name=String('launcher_cmd'),
                    referee_topic_name=String('robot_game_ref'), status_topic_name=String('launcher_status')),
}
DEPS = {
    'AttitudeEkf': ('ramfs',), 'ESKF': ('ramfs',),
    'DmImu': ('database', 'ramfs', 'can', 'uart'),
    'Chassis': ('right_front', 'left_front', 'left_rear', 'right_rear', 'cmd', 'power_control', 'ramfs'),
    'Gimbal': ('big_yaw', 'small_yaw', 'pitch_motor', 'cmd', 'calibration_database', 'launcher', 'calibration_buzzer'),
    'Launcher': ('fric_right', 'fric_left', 'trigger'), 'PowerControl': ('cap_can',),
}


def migrate_instance(instance):
    """按明确的旧字段搬迁，保留每个旧配置值和字符串引号。"""
    identity = instance.get('module', '')
    if not identity.startswith('WDR-Xrobot-org/'):
        return False
    name = identity.split('/')[-1]
    if name not in DEPS:
        return False
    args = CommentedMap()
    for item in instance.get('args', []):
        if not isinstance(item, dict) or len(item) != 1:
            raise ValueError(f'{instance.get("id")}: args必须为单键映射列表')
        key, value = next(iter(item.items()))
        if key in args:
            raise ValueError(f'{instance.get("id")}: 重复参数 {key}')
        args[key] = value
    if list(args) == [*DEPS[name], 'param']:
        return False
    param = args.pop('param', None)
    if not isinstance(param, dict):
        raise ValueError(f'{instance.get("id")}: 旧param必须为完整映射，C++表达式请手工迁移')
    if name in ('AttitudeEkf', 'ESKF'):
        if name == 'ESKF':
            unknown = set(param) - set(ESKF_DEFAULTS)
            if unknown:
                raise ValueError(f'ESKF未知字段: {sorted(unknown)}')
            filled = CommentedMap(copy.deepcopy(ESKF_DEFAULTS))
            filled.update(param)
            param = filled
        fallback = dict(zip(FUSION_NAMES,
                            [String('bmi088_gyro'), String('bmi088_accl'),
                             String('board_attitude' if name == 'AttitudeEkf' else 'eskf_output'),
                             String('board_fusion_diagnostics' if name == 'AttitudeEkf' else 'eskf_diagnostics'),
                             String('attitude' if name == 'AttitudeEkf' else 'eskf')]))
        for key in FUSION_NAMES:
            param[key] = args.pop(key, fallback[key])
    elif name in EXTRAS:
        if name == 'DmImu':
            param = CommentedMap(device=param)
        elif name == 'Chassis':
            param = CommentedMap(control=param)
        for key, default in EXTRAS[name].items():
            param[key] = args.pop(key, copy.deepcopy(default))
    else:
        param = CommentedMap(control=param, super_cap=args.pop('cap_param', CommentedMap(
            feedback_id='0x329', command_id='0x330', normal_state=0)),
            referee_power_topic_name=args.pop('referee_power_topic_name', String('chassis_power_ref')))
    for key in DEPS[name]:
        if key not in args:
            if name == 'Gimbal' and key in ('calibration_database', 'launcher', 'calibration_buzzer'):
                args[key] = 'nullptr'
            elif name == 'PowerControl' and key == 'cap_can':
                args[key] = 'nullptr'
            else:
                raise ValueError(f'{instance.get("id")}: 缺少依赖 {key}')
    unknown = set(args) - set(DEPS[name])
    if unknown:
        raise ValueError(f'{instance.get("id")}: 无法迁移未知参数 {sorted(unknown)}')
    instance['args'] = [CommentedMap([(key, args[key])]) for key in DEPS[name]]
    instance['args'].append(CommentedMap(param=param))
    return True


def migrate_file(path, apply=False):
    yaml = YAML()
    yaml.preserve_quotes = True
    yaml.indent(mapping=2, sequence=4, offset=2)
    yaml.width = 1000
    before = path.read_text()
    data = yaml.load(before)
    count = sum(migrate_instance(instance) for instance in data.get('modules', []))
    if not count:
        return 0
    output = io.StringIO()
    yaml.dump(data, output)
    after = output.getvalue()
    print(''.join(difflib.unified_diff(before.splitlines(True), after.splitlines(True),
                                     fromfile=str(path), tofile=str(path))), end='')
    if apply:
        path.write_text(after)
    print(f'{path}: {count} 个实例，' + ('已写入' if apply else '仅预览'))
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('configs', nargs='+', type=Path)
    parser.add_argument('--apply', action='store_true')
    args = parser.parse_args()
    for path in args.configs:
        migrate_file(path, args.apply)


if __name__ == '__main__':
    main()
