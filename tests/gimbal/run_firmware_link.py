#!/usr/bin/env python3
"""临时工程完整链接云台组合，默认固件配置不变、不烧录。"""
from pathlib import Path
import tempfile,shutil,subprocess,yaml,argparse
parser=argparse.ArgumentParser();parser.add_argument("--combined",action="store_true");args=parser.parse_args()
root=Path(__file__).resolve().parents[2]
log=root/('build/validation/gimbal_chassis_full_link.log' if args.combined else 'build/validation/gimbal_full_link.log');log.parent.mkdir(parents=True,exist_ok=True)
with tempfile.TemporaryDirectory(prefix='wdr-full-gimbal-') as temp:
 stage=Path(temp)
 for item in root.iterdir():
  if item.name in ['.git','User','build','.cache','xrobot.lock']:continue
  (stage/item.name).symlink_to(item,target_is_directory=item.is_dir())
 shutil.copyfile(root/'xrobot.lock',stage/'xrobot.lock')
 shutil.copytree(root/'User',stage/'User',ignore=shutil.ignore_patterns('xrobot_main.hpp'))
 board=yaml.safe_load((root/'User/xrobot.yaml').read_text())
 extras=[]
 if args.combined:extras.extend(yaml.safe_load((root/'config/templates/chassis.yaml').read_text())['modules'])
 extras.extend(yaml.safe_load((root/'config/templates/gimbal.yaml').read_text())['modules'])
 known={m['id'] for m in board['modules']}
 for instance in extras:
  if instance['id'] not in known:board['modules'].append(instance);known.add(instance['id'])
 # 底盘停止联锁随gimbal_chassis_feedback电平发布，组合工程无需额外事件绑定。
 def quote(value):
  if isinstance(value,dict):
   for key,item in value.items():
    if isinstance(item,str) and (key.endswith(('_topic_name','_tp_name')) or key=='topic_name'):value[key]='"'+item.strip('"')+'"'
    else:quote(item)
  elif isinstance(value,list):
   for item in value:quote(item)
 quote(board);(stage/'User/full.yaml').write_text(yaml.safe_dump(board,sort_keys=False,allow_unicode=True))
 with log.open('w') as out:
  for command in [['xrobot','gen','-c','User/full.yaml','--no-line-directives'],['cmake','--preset','Debug'],['cmake','--build','--preset','Debug','-j4','--target','wdr_xrobot_dmmc02_framework']]:
   subprocess.run(command,cwd=stage,stdout=out,stderr=subprocess.STDOUT,check=True)
  symbols=subprocess.check_output(['arm-none-eabi-nm','-C',str(stage/'build/Debug/wdr_xrobot_dmmc02_framework.elf')],text=True)
  for name in ['Gimbal::ThreadFunc','DualYaw::ControlLoop','Pitch::ControlLoop','Launcher::Update']+(['Chassis<wdr_chassis::Control>','PowerControl::Calculate'] if args.combined else []):assert name in symbols,name
 print('PASS: full gimbal'+(' + chassis/power/interlock' if args.combined else '')+' firmware linked; no flash')
