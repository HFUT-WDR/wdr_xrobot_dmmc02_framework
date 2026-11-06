#!/usr/bin/env python3
"""在临时检出验证本地模块快照和锁文件，保持当前工作区不变。"""
from pathlib import Path
import tempfile,subprocess,shutil,yaml
root=Path(__file__).resolve().parents[1]
with tempfile.TemporaryDirectory(prefix='wdr-snapshot-restore-') as temp:
 stage=Path(temp);(stage/'Modules').mkdir()
 for name in ['modules.yaml','sources.yaml']:shutil.copyfile(root/'Modules'/name,stage/'Modules'/name)
 shutil.copyfile(root/'xrobot.lock',stage/'xrobot.lock')
 for name in ['module_sources','ModuleIndex']:(stage/name).symlink_to(root/name,target_is_directory=True)
 shutil.copytree(root/'User',stage/'User',ignore=shutil.ignore_patterns('xrobot_main.hpp'))
 lock=yaml.safe_load((root/'xrobot.lock').read_text())
 snapshots={m['id']:root/'module_sources/HFUT-WDR'/Path(m['repo']).name for m in yaml.safe_load((root/'module_sources/HFUT-WDR/index.yaml').read_text())['modules']}
 for name in lock['modules']:
  target=stage/'Modules'/name;target.parent.mkdir(exist_ok=True)
  if name in snapshots:
   subprocess.run(['git','clone','-q','-b','dev',str(snapshots[name]),str(target)],check=True)
  else:subprocess.run(['git','clone','-q','--shared','--no-checkout',str(root/'Modules'/name),str(target)],check=True)
  subprocess.run(['git','-C',str(target),'checkout','-q','--detach',lock['modules'][name]['commit']],check=True)
 subprocess.run(['xrobot','setup','--frozen','--offline','--no-line-directives'],cwd=stage,check=True)
 print(f'PASS: {len(snapshots)} snapshots restored; {len(lock["modules"])}-module frozen offline setup')
