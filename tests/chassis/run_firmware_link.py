#!/usr/bin/env python3
"""临时工程完整链接底盘组合，不修改默认配置、不烧录。"""
from pathlib import Path
import tempfile,shutil,subprocess,yaml
root=Path(__file__).resolve().parents[2]
log=root/"build/validation/chassis_full_link.log"
log.parent.mkdir(parents=True,exist_ok=True)
with tempfile.TemporaryDirectory(prefix='wdr-full-chassis-') as temp:
 stage=Path(temp)
 for item in root.iterdir():
  if item.name in ['.git','User','build','.cache']:continue
  (stage/item.name).symlink_to(item,target_is_directory=item.is_dir())
 shutil.copytree(root/'User',stage/'User',ignore=shutil.ignore_patterns('xrobot_main.hpp'))
 board=yaml.safe_load((root/'User/xrobot.yaml').read_text())
 board['modules'].extend(yaml.safe_load((root/'config/templates/chassis.yaml').read_text())['modules'])
 def quote(x):
  if isinstance(x,dict):
   for k,v in x.items():
    if isinstance(v,str) and (k.endswith(('_topic_name','_tp_name')) or k=='topic_name'):x[k]='"'+v.strip('"')+'"'
    else:quote(v)
  elif isinstance(x,list):
   for v in x:quote(v)
 quote(board);(stage/'User/full.yaml').write_text(yaml.safe_dump(board,sort_keys=False,allow_unicode=True))
 # lock与配置生成仅在临时工程中运行，不改变默认机器人。
 (stage/'xrobot.lock').unlink();shutil.copyfile(root/'xrobot.lock',stage/'xrobot.lock')
 with log.open('w') as out:
  for cmd in [['xrobot','gen','-c','User/full.yaml','--no-line-directives'],['cmake','--preset','Debug'],['cmake','--build','--preset','Debug','-j4','--target','wdr_xrobot_dmmc02_framework']]:
   subprocess.run(cmd,cwd=stage,stdout=out,stderr=subprocess.STDOUT,check=True)
  symbols=subprocess.check_output(['arm-none-eabi-nm','-C',str(stage/'build/Debug/wdr_xrobot_dmmc02_framework.elf')],text=True)
  for name in ['Chassis<wdr_chassis::Control>','PowerControl::Calculate','wdr_chassis::PowerControl::Calculate']:
   assert name in symbols,name
 print('PASS: full chassis/power/RC/referee/cap firmware linked in temporary project')
