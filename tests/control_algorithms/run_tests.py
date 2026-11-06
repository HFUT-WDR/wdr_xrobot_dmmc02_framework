#!/usr/bin/env python3
"""公共控制算法、冻结旧实现缺陷与sp_middleware行为对照；不访问硬件。"""
from pathlib import Path
import argparse,subprocess,tempfile,hashlib,json
root=Path(__file__).resolve().parents[2];test=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--modules-dir',type=Path,default=root/'Modules/YunMo-yanyu');args=parser.parse_args()
module=args.modules_dir.resolve()/'ControlAlgorithms';frozen=root/'tests/chassis/reference/control_algorithms/algorithm'
sp=Path('/home/yunmo/WorkingSpace/sp_middleware/sp_middleware')
for name,digest in json.loads((root/'tests/chassis/reference/algorithm_hashes.json').read_text()).items():
 assert hashlib.sha256((frozen.parent/name).read_bytes()).hexdigest()==digest
with tempfile.TemporaryDirectory(prefix='wdr-control-algorithms-') as temp:
 temp=Path(temp);obj=[];flags=['-O1','-g','-fsanitize=address,undefined','-static-libasan','-fno-omit-frame-pointer']
 for i,name in enumerate(['PID.c','LPF.c']):
  out=temp/f'{i}.o';subprocess.run(['cc','-std=gnu11',*flags,'-c',str(frozen/name),'-o',str(out)],check=True);obj.append(str(out))
 includes=[root/'tests/chassis/stubs',module,module/'algorithm',frozen,root/'Middlewares/Third_Party/LibXR/src/utils',sp]
 exe=temp/'test'
 subprocess.run(['c++','-std=c++20','-DLIBXR_DEFAULT_SCALAR=float',*flags,*['-I'+str(x) for x in includes],str(test/'test_algorithms.cpp'),str(sp/'tools/pid/pid.cpp'),str(sp/'tools/math_tools/math_tools.cpp'),str(sp/'tools/low_pass_filter/low_pass_filter.cpp'),*obj,'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)

 try:
  subprocess.run([str(exe),'--frozen-angle-inf'],check=True,stdout=subprocess.PIPE,stderr=subprocess.PIPE,timeout=.5)
  raise AssertionError('Frozen angle Inf probe unexpectedly terminated')
 except subprocess.TimeoutExpired as result:
  assert b'entered frozen angle probe' in result.stdout
  print('PASS: frozen angle Inf nontermination reproduced in isolated child; new PID rejects Inf')
