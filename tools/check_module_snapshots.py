#!/usr/bin/env python3
"""在临时检出验证锁文件能从现有模块检出恢复，保持当前工作区不变。

逐模块从本工程的 Modules/<id> 共享克隆，切到 xrobot.lock 记录的提交，
再执行冻结离线 setup。这样验证的是「锁文件 + 现有检出」能否一致复现，
不依赖已移除的 module_sources bundle 快照。
"""
from pathlib import Path
import tempfile,subprocess,shutil,yaml
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='wdr-snapshot-restore-') as temp:
 stage=Path(temp);(stage/'Modules').mkdir()
 for name in ['modules.yaml','sources.yaml']:shutil.copyfile(root/'Modules'/name,stage/'Modules'/name)
 shutil.copyfile(root/'xrobot.lock',stage/'xrobot.lock')
 (stage/'ModuleIndex').symlink_to(root/'ModuleIndex',target_is_directory=True)
 shutil.copytree(root/'User',stage/'User',ignore=shutil.ignore_patterns('xrobot_main.hpp'))
 lock=yaml.safe_load((root/'xrobot.lock').read_text())
 for name in lock['modules']:
  target=stage/'Modules'/name;target.parent.mkdir(exist_ok=True)
  subprocess.run(['git','clone','-q','--shared','--no-checkout',str(root/'Modules'/name),str(target)],check=True)
  subprocess.run(['git','-C',str(target),'checkout','-q','--detach',lock['modules'][name]['commit']],check=True)
 subprocess.run(['xrobot','setup','--frozen','--offline','--no-line-directives'],cwd=stage,check=True)
 print(f'PASS: {len(lock["modules"])}-module frozen offline setup from existing checkouts')
