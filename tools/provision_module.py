#!/usr/bin/env python3
"""为新模块仓库补齐 CI 与凭据，替代逐仓库手工配置 Secret。

私有模块的 CI 必须能读取同组织的其他私有模块，而这需要一个 GitHub App
安装令牌；reusable workflow 只能收到调用方传入的 Secret，不能读取自己仓库
里的 Secret，所以每个模块仓库都要有一份 WDR_APP_PRIVATE_KEY。
App ID 不是密钥，已经写成共享工作流的输入默认值，不需要按仓库配置。

用法：
  tools/provision_module.py --key ~/下载/wdr-xrobot-ci.private-key.pem --all
  tools/provision_module.py --key ... --repo NewModule [--template-args '["Omni"]']

--all 遍历 Modules/WDR-Xrobot-org 下已有的检出；--repo 用于新仓库，
仓库必须已存在并且有 dev 分支。脚本幂等，可重复运行。
"""
import argparse
import os
import shutil
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MODULES = ROOT / 'Modules' / 'WDR-Xrobot-org'
INDEX = ROOT / 'ModuleIndex' / 'index.yaml'
ORG = 'WDR-Xrobot-org'
SECRET = 'WDR_APP_PRIVATE_KEY'

TEMPLATE = """name: Module CI
on:
  push:
    branches: [dev, master]
  pull_request:
    branches: [dev, master]
  workflow_dispatch:
permissions:
  contents: read
jobs:
  build:
    uses: {org}/xrobot-ci/.github/workflows/module-ci.yml@dev
{extra}    secrets:
      {secret}: ${{{{ secrets.{secret} }}}}
"""


def run(args, **kw):
    return subprocess.run(args, check=True, text=True, capture_output=True, **kw).stdout


def gh_json(args):
    return run(['gh', *args])


def secret_present(name):
    """仓库里是否已有同名 Secret。只列出名字，不读取值。"""
    out = run(['gh', 'secret', 'list', '--repo', f'{ORG}/{name}', '--json', 'name'])
    return SECRET in out


def set_secret(name, key):
    # 通过标准输入传入私钥，避免出现在命令行参数与进程列表里。
    with open(key, 'rb') as handle:
        subprocess.run(['gh', 'secret', 'set', SECRET, '--repo', f'{ORG}/{name}'],
                       stdin=handle, check=True, capture_output=True)


def existing_with_block(path):
    """取出已有工作流里 `with:` 到 `secrets:` 之间的内容，避免重写时丢掉已提交的输入。"""
    if not path.is_file():
        return ''
    lines = path.read_text().splitlines(keepends=True)
    try:
        start = next(i for i, line in enumerate(lines) if line.strip() == 'with:')
        end = next(i for i, line in enumerate(lines[start:], start) if line.strip() == 'secrets:')
    except StopIteration:
        return ''
    return ''.join(lines[start:end])


def write_workflow(path, template_args):
    """重写 CI 工作流；显式给出的模板实参优先，否则沿用已有的 `with:` 块。"""
    extra = ''
    if template_args:
        extra = f"    with:\n      template-args: '{template_args}'\n"
    else:
        extra = existing_with_block(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(TEMPLATE.format(org=ORG, extra=extra, secret=SECRET))


def ensure_index(name):
    """在索引中补一行；已存在时不改动，保持原有顺序。"""
    entry = f'- id: {ORG}/{name}\n  repo: https://github.com/{ORG}/{name}.git\n  status: community\n'
    text = INDEX.read_text()
    if f'- id: {ORG}/{name}\n' in text:
        return False
    anchor = 'modules:\n'
    at = text.index(anchor) + len(anchor)
    INDEX.write_text(text[:at] + entry + text[at:])
    return True


def main():
    parser = argparse.ArgumentParser(description='为新模块仓库写入 CI 并配置 App 凭据')
    parser.add_argument('--key', required=True, type=Path, help='App 私钥 PEM 路径')
    parser.add_argument('--repo', action='append', default=[], help='仓库名，可重复')
    parser.add_argument('--all', action='store_true', help='处理 Modules/WDR-Xrobot-org 下的全部检出')
    parser.add_argument('--template-args', default='', help="类模板实参的 JSON 列表，如 '[\"Omni\"]'")
    parser.add_argument('--check', action='store_true', help='只报告缺什么，不写入')
    args = parser.parse_args()

    if not args.key.is_file():
        sys.exit(f'找不到 App 私钥：{args.key}')
    names = list(args.repo)
    if args.all:
        names += sorted(p.name for p in MODULES.iterdir() if (p / '.git').exists())
    names = sorted(dict.fromkeys(names))
    if not names:
        sys.exit('没有要处理的仓库：用 --repo 指定，或加 --all')

    changed = []
    for name in names:
        local = MODULES / name
        workflow = local / '.github' / 'workflows' / 'build.yml'
        todo = []
        if not local.is_dir():
            todo.append('本地没有检出')
        if local.is_dir() and not secret_present(name):
            todo.append('缺 Secret')
        if todo:
            changed.append(name)
        print(f'{name:22} {"、".join(todo) if todo else "已就绪"}')
        if args.check or not todo:
            continue
        if not local.is_dir():
            continue
        write_workflow(workflow, args.template_args)
        set_secret(name, args.key)
        if ensure_index(name):
            print(f'{"":22} 已加入 ModuleIndex/index.yaml')

    if args.check:
        return
    print(f'\n{len(changed)} 个仓库需要处理')


if __name__ == '__main__':
    main()
