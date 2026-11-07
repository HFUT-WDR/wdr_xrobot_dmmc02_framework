#!/usr/bin/env python3
"""将专用只读令牌加密写入WDR仓库的CI Secret，不保存到本地文件。"""
import argparse
import concurrent.futures
import getpass
import json
import subprocess
import urllib.error
import urllib.request

ORG = 'HFUT-WDR'
SECRET = 'WDR_MODULES_READ_TOKEN'
REPOSITORIES = [
    'BoardAttitude', 'CMD', 'Chassis', 'ControlAlgorithms', 'DMMotor', 'DR16',
    'DmImu', 'EventBinder', 'Gimbal', 'HostData', 'I6x', 'ImuFusion',
    'Launcher', 'Motor', 'PowerControl', 'RMMotor', 'Referee',
    'wdr_xrobot_dmmc02_framework',
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='只检查Secret是否已配置，不读取其值')
    args = parser.parse_args()
    if args.check:
        missing = []
        for name in REPOSITORIES:
            data = subprocess.check_output([
                'gh', 'secret', 'list', '--repo', ORG + '/' + name, '--json', 'name'
            ], text=True)
            present = any(item['name'] == SECRET for item in json.loads(data))
            print(('OK ' if present else 'MISSING ') + name)
            if not present:
                missing.append(name)
        return int(bool(missing))
    print('请使用HFUT-WDR专用Fine-grained Token：新建20个仓库的Contents只读权限。')
    print('包含xrobot-ci、18个模块及框架；不要使用个人账户的广泛读写令牌。')
    token = getpass.getpass('输入只读Token（不会显示或写入文件）：').strip()
    if not token:
        raise SystemExit('未输入Token，未修改任何Secret。')

    def validate(name):
        # 读取分支引用需要Contents权限，避免仅Metadata可读却误判配置成功。
        request = urllib.request.Request(
            'https://api.github.com/repos/' + ORG + '/' + name + '/git/ref/heads/dev',
            headers={'Authorization': 'Bearer ' + token,
                     'Accept': 'application/vnd.github+json'},
        )
        try:
            with urllib.request.urlopen(request, timeout=20) as response:
                if response.status != 200:
                    raise RuntimeError(name + ': unexpected HTTP ' + str(response.status))
        except urllib.error.HTTPError as error:
            raise RuntimeError(name + ': HTTP ' + str(error.code) +
                               '，请检查资源所有者、仓库选择、Contents只读权限、组织审批及有效期。') from None
        return name

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        try:
            for name in pool.map(validate, ['xrobot-ci', *REPOSITORIES]):
                print('读取权限已确认 ' + ORG + '/' + name)
        except (RuntimeError, urllib.error.URLError) as error:
            raise SystemExit(str(error) + '\n尚未修改任何Secret。') from None

    def configure(name):
        # 令牌经标准输入交给gh加密，不进入命令行参数或日志。
        result = subprocess.run([
            'gh', 'secret', 'set', SECRET, '--repo', ORG + '/' + name
        ], input=token, text=True, capture_output=True)
        if result.returncode:
            raise RuntimeError(name + ': ' + result.stderr.strip())
        return name

    with concurrent.futures.ThreadPoolExecutor(max_workers=4) as pool:
        for name in pool.map(configure, REPOSITORIES):
            print('已配置 ' + ORG + '/' + name)
    print('完成；令牌只保存在GitHub加密Secret中。')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
