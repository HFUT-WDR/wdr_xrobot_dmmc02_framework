#!/usr/bin/env python3
"""冻结算法对照、流组帧、安装矩阵与独立实例回归。"""
from pathlib import Path
import argparse, subprocess, tempfile, re, json, hashlib
root=Path(__file__).resolve().parents[2]; test=Path(__file__).resolve().parent
parser=argparse.ArgumentParser();parser.add_argument('--modules-dir',type=Path,default=root/'Modules/YunMo-yanyu');args=parser.parse_args()
core=args.modules_dir.resolve()/'ImuFusion'
for name,digest in json.loads((test/'reference/sha256.json').read_text()).items():
    assert hashlib.sha256((test/'reference'/name).read_bytes()).hexdigest()==digest
with tempfile.TemporaryDirectory(prefix='wdr-imu-') as directory:
    tmp=Path(directory)
    flags=['-DUSER_MALLOC=tracked_malloc','-include',str(test/'stubs/test_allocator.h'),'-O1','-g','-fsanitize=address,undefined','-static-libasan','-fno-omit-frame-pointer']
    inc=['-I'+str(test/'stubs'),'-I'+str(core)]
    for board in [True,False]:
        name='Quaternion.c' if board else 'dm_imu_l1_attitude.c'
        oracle=(test/'reference'/name).read_text().replace('Quaternion.h','quaternion.h').replace('Kalman_Filter.h','kalman_filter.h')
        oracle=oracle.replace('long i = *(long *)&y;','uint32_t i; memcpy(&i, &y, sizeof(i));').replace('y = *(float *)&i;','memcpy(&y, &i, sizeof(y));')
        oracle=oracle.replace('Matrix_Transpose(&KF->Mat.Cache_Matrix[1], &KF->Mat.Cache_Vector[0])','Matrix_Transpose(&KF->Mat.Cache_Vector[1], &KF->Mat.Cache_Vector[0])')
        oracle=oracle.replace('Matrix_Transpose(&kf->Mat.Cache_Matrix[1], &kf->Mat.Cache_Vector[0])','Matrix_Transpose(&kf->Mat.Cache_Vector[1], &kf->Mat.Cache_Vector[0])')
        mapping=json.loads((test/'reference'/('board_names.json' if board else 'dm_names.json')).read_text())
        if board: mapping.update(quaternion_info='reference_info',Quaternion_Info='reference_info',QuaternionEKF_Init='reference_init',QuaternionEKF_Update='reference_update')
        else:
            # 原文件仍使用双槽位，类型取生产输出结构，避免修改冻结源。
            oracle=oracle.replace('#include "dm_imu_l1_attitude.h"','#include "dm_imu_l1_attitude.h"\n#define DM_IMU_L1_ATTITUDE_IMU_COUNT 2U')
            for suffix in ['init_all','reset','update','get']:
                mapping['dm_imu_l1_attitude_'+suffix]='reference_'+suffix
            # 原内部上下文与公开的实例上下文分别命名。
            mapping['dm_imu_l1_attitude_ctx_t']='reference_ctx_t'
        oracle=re.sub(r'\b[A-Za-z_]\w*\b',lambda m:mapping.get(m[0],m[0]),oracle)
        source=tmp/'reference.c';source.write_text(oracle)
        exe=tmp/('board' if board else 'dm')
        subprocess.run(['cc','-std=c11',*flags,'-DUSER_MALLOC=tracked_malloc','-include',str(test/'stubs/test_allocator.h'),*inc,str(test/('test_board.c' if board else 'test_dm.c')),str(source),str(core/'kalman_filter.c'),'-lm','-o',str(exe)],check=True)
        subprocess.run([str(exe)],check=True)
    objects=[]
    for i,name in enumerate(['kalman_filter.c','quaternion.c','dm_imu_l1_attitude.c','board_attitude.cpp','dm_imu_device.cpp','dm_imu_rs485_stream_assembler.cpp']):
        cpp=name.endswith('.cpp');obj=tmp/(str(i)+'.o')
        subprocess.run(['c++' if cpp else 'cc','-std=c++20' if cpp else '-std=c11',*flags,*inc,'-c',str(core/name),'-o',str(obj)],check=True);objects.append(str(obj))
    exe=tmp/'pipeline'
    subprocess.run(['c++','-std=c++20',*flags,*inc,'-I'+str(test),str(test/'test_pipeline.cpp'),*objects,'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)

    exe=tmp/'module'
    subprocess.run(['c++','-std=c++20',*flags,'-I'+str(test/'module_stubs'),*inc,'-I'+str(args.modules_dir.resolve()/'DmImu'),'-I'+str(test),str(test/'test_module.cpp'),*objects,'-lm','-o',str(exe)],check=True)
    subprocess.run([str(exe)],check=True)
