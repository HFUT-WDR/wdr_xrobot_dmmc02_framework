#!/usr/bin/env python3
"""云台真实模块离线集成、协议与原车冻结算法回归；不发送硬件。"""
from pathlib import Path
import subprocess,tempfile,hashlib,json,shutil
root=Path(__file__).resolve().parents[2];test=Path(__file__).resolve().parent
modules=root/'Modules/HFUT-WDR'
for name,digest in json.loads((test/'reference/hashes.json').read_text()).items():
 assert hashlib.sha256((test/'reference'/name).read_bytes()).hexdigest()==digest,name
inc=[test/'stubs',root/'tests/chassis/stubs']+[modules/n for n in ['ImuFusion','Gimbal','Gimbal/components','Launcher','CMD','Motor','RMMotor','DMMotor','Chassis','ControlAlgorithms','ControlAlgorithms/include','HostData','Referee']]+[root/'Middlewares/Third_Party/LibXR/src'/n for n in ['structure','utils']]
with tempfile.TemporaryDirectory(prefix='wdr-gimbal-') as temp:

 temp=Path(temp)
 # 只在离线时钟桩中注入回调，复现控制拍内到达的停止事件；生产代码无测试钩子。
 stubs=temp/'stubs';shutil.copytree(root/'tests/chassis/stubs',stubs)
 fake=stubs/'fake_libxr.hpp';text=fake.read_text()
 text=text.replace('  static inline uint64_t now = 1000;', '  static inline uint64_t now = 1000;\n  static inline std::function<void()> time_hook;')
 text=text.replace('  static MillisecondTimestamp GetMilliseconds() { return now / 1000; }', '  static MillisecondTimestamp GetMilliseconds() { if(time_hook){auto callback=time_hook;time_hook={};callback();} return now / 1000; }')
 fake.write_text(text);inc[1]=stubs
 pid=(test/'reference/PID.c').read_text()
 pid=pid[pid.index('static float angle_wrap('):pid.index('void PID_ClearExtended(')]
 cali=(test/'reference/gimbal_cali.c').read_text()
 cali=cali[cali.index('static fp32 motor_ecd_to_angle_change('):cali.index('void calc_gimbal_cali(')]
 plan=(test/'reference/dual_yaw_ctrl.c').read_text()
 plan=plan[plan.index('void Two_Yaw_small_yaw_first_final_err_calculate('):plan.index('void TwoYAW_gimbal_absolute_angle_control_small_yaw_first(')]
 prefix="""#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include "reference_api.h"
typedef float fp32;
#define ECD_RANGE 8191
#define MOTOR_ECD_TO_RAD 0.000766990394f
#define INNER_YAW 0
#define OUTER_YAW 1
static float rad_format(float x){while(x>M_PI)x-=2*M_PI;while(x< -M_PI)x+=2*M_PI;return x;}
static float fp32_constrain(float x,float lo,float hi){return x<lo?lo:x>hi?hi:x;}
typedef struct {
 float relative_angle,absolute_angle,absolute_angle_set,max_relative_angle,min_relative_angle;
 float tor,motor_gyro,motor_gyro_set,current_set,last_gyro;
 int id;int16_t given_current;
 pid_type_def gimbal_motor_absolute_angle_pid,gimbal_motor_gyro_pid,gimbal_motor_soft_gyro_pid;
} ReferenceAxis;
typedef ReferenceAxis gimbal_motor_t;
typedef struct {ReferenceAxis gimbal_yaw_motor[2];float big_yaw_drag_target;ReferenceAxis gimbal_pitch_motor;} gimbal_control_t;
"""
 suffix="""
void reference_plan(float rel,float small,float big,float target,float minimum,float maximum,float*drag,float*big_error,float*small_error)
{
 gimbal_control_t c={0};
 c.gimbal_yaw_motor[0].absolute_angle=big;
 c.gimbal_yaw_motor[1]=(ReferenceAxis){rel,small,target,maximum,minimum};
 c.big_yaw_drag_target=*drag;
 Two_Yaw_small_yaw_first_final_err_calculate(&c,big_error,small_error);
 *drag=c.big_yaw_drag_target;
}
"""
 pitch=(test/'reference/pitch_ctrl.c').read_text()
 pitch_i=pitch[pitch.index('static inline void pitch_i_update('):pitch.index('// Pitch 独立 ESO')]
 coulomb=pitch[pitch.index('static float PitchCoulombComp('):pitch.index('void NormalControlStrategy(')]
 strategy=pitch[pitch.index('void NormalControlStrategy('):pitch.index('void Pitch_Scan_Mode(')]
 pitch_prefix="""
#include "gimbal_params.h"
#define PITCH_DATA_COLLECT 0
#define PITCH_USE_ESO 0
#define YAW_MOTOR 0
#define GIMBAL_AUTO 1
#define GIMBAL_AUTO_TEST 2
static int gimbal_behaviour=1,last_gimbal_behaviour=1;
static gimbal_control_t gimbal_control;
typedef struct {float pitch,pitch_vel,pitch_acc;} VisionToGimbal;
static VisionToGimbal vision;
static const VisionToGimbal* get_latest_vision_data(void){return &vision;}
static int get_auto_mode(void){return 0;}
static int get_auto_last_mode(void){return 0;}
static float get_pitch_angle(void){return vision.pitch;}
static float get_pitch_vel(void){return vision.pitch_vel;}
static float gimbal_PID_calc(pid_type_def*p,float a,float b,float c){return PID_CalculateByError(p,b-a,c);}
static float fb_gimbal_PID_calc(pid_type_def*p,float a,float b){return PID_CalculateVelocity(p,a,b);}
static int16_t current_limit_i16(float x,float l){return (int16_t)fp32_constrain(x,-l,l);}
"""
 pitch_suffix="""
float reference_pitch(float angle,float gyro,float target,float velocity,float acceleration,float*integral)
{
 ReferenceAxis* p=&gimbal_control.gimbal_pitch_motor;
 p->id=1;p->absolute_angle=angle;p->motor_gyro=gyro;
 p->gimbal_motor_absolute_angle_pid.Integral=*integral;
 vision=(VisionToGimbal){target,velocity,acceleration};
 NormalControlStrategy(p);
 *integral=p->gimbal_motor_absolute_angle_pid.Integral;
 return p->tor;
}
"""
 source=temp/'reference.c';source.write_text(prefix+pid+cali+plan+suffix+pitch_prefix+pitch_i+coulomb+strategy+pitch_suffix)
 obj=temp/'reference.o'
 subprocess.run(['cc','-std=gnu11','-O1','-I'+str(test),'-I'+str(test/'reference'),'-c',str(source),'-o',str(obj)],check=True)
 exe=Path(temp)/'gimbal'
 subprocess.run(['c++','-std=c++20','-DLIBXR_DEFAULT_SCALAR=float','-O1','-g','-fsanitize=address,undefined','-static-libasan','-fno-omit-frame-pointer',*['-I'+str(p) for p in inc],str(test/'test_gimbal.cpp'),str(test/'stubs/host_telemetry.cpp'),str(obj),*[str(modules/n/f) for n,f in [('Gimbal','components/dual_yaw.cpp'),('Gimbal','components/pitch.cpp'),('Launcher','launcher.cpp'),('Gimbal','gimbal.cpp')]],'-o',str(exe)],check=True)
 subprocess.run([str(exe)],check=True)

 boundary=temp/'boundaries'
 subprocess.run(['c++','-std=c++20','-DLIBXR_DEFAULT_SCALAR=float','-O1','-g','-fsanitize=address,undefined','-static-libasan','-fno-omit-frame-pointer',*['-I'+str(p) for p in inc],str(test/'test_boundaries.cpp'),str(test/'stubs/host_telemetry.cpp'),*[str(modules/n/f) for n,f in [('Gimbal','components/dual_yaw.cpp'),('Gimbal','components/pitch.cpp'),('Launcher','launcher.cpp'),('Gimbal','gimbal.cpp')]],'-o',str(boundary)],check=True)
 for scenario in ['late_stop','mode_loss','revive','init','startup','scan_zero','scan_limits','aim_limits','boundary_feedforward','navigation','launcher','source_switch','power_on','buzzer','buzzer_complete']:
  subprocess.run([str(boundary),scenario],check=True)
