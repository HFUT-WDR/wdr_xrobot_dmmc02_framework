#!/usr/bin/env python3
"""发射机构离线回归：行为场景，以及虚拟热量/弹速管理/卡弹/滤波与原车冻结代码逐拍对照。"""
from pathlib import Path
import hashlib, json, subprocess, tempfile

root = Path(__file__).resolve().parents[2]
test = Path(__file__).resolve().parent
modules = root / 'Modules/HFUT-WDR'
reference = test / 'reference'
for name, digest in json.loads((reference / 'hashes.json').read_text()).items():
    assert hashlib.sha256((reference / name).read_bytes()).hexdigest() == digest, name


def cut(text, start, end):
    begin = text.index(start)
    return text[begin:text.index(end, begin)]


shoot = (reference / 'shoot_task.c').read_text()
header = (reference / 'shoot_task.h').read_text()
heat = (reference / 'heat_control.c').read_text()
lpf = (reference / 'LPF.c').read_text()
prefix = r'''
#include <math.h>
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "reference_api.h"
typedef float fp32;
typedef uint8_t bool_t;
#define RC_TYPE 0
#define GIMBAL_CONTROL_TIME 1
#define KEY_PRESSED_OFFSET_Q 0
#define KEY_PRESSED_OFFSET_E 0
#define KEY_PRESSED_OFFSET_R 0
#define GIMBAL_AUTO 1
#define GIMBAL_AUTO_TEST 2
#define ENHANCED_ATTACK_STATE 99
#define switch_is_down(s) ((s) == 2)
static uint32_t ref_tick;
static uint32_t HAL_GetTick(void) { return ref_tick; }
static int gimbal_behaviour = GIMBAL_AUTO;
static int get_sentry_state(void) { return 0; }
static float ref_cooling, ref_limit, ref_bullet_speed;
static uint16_t ref_heat1;
static float get_shoot_cooling_value(void) { return ref_cooling; }
static float get_shoot_heat_limit(void) { return ref_limit; }
static uint16_t get_shoot_heat1(void) { return ref_heat1; }
static void record_fire_detect_L(void) {}
static void set_interrupt_flag_end_L(void) {}
static void get_shooter_now_speed(fp32* speed, uint8_t id) { (void)id; *speed = ref_bullet_speed; }
typedef struct { int8_t s[4]; } rc_t;
typedef struct { rc_t rc; } RC_ctrl_t;
typedef enum { SHOOT_STOP = 0, SHOOT_READY_FRIC, SHOOT_READY_BULLET, SHOOT_READY, SHOOT_BULLET,
               SHOOT_CONTINUE_BULLET } shoot_mode_e;
typedef struct {
  shoot_mode_e shoot_mode; const RC_ctrl_t* shoot_rc; bool_t press_l;
  fp32 Trigger_Targetspeed, Trigger_Actualspeed, Trigger_FBspeed_filtered;
  uint16_t block_time, reverse_time, shoot_speed_limit;
} shoot_control_t;
typedef struct { float Input; float Output[3]; float Alpha[3]; bool Initialized; } LowPassFilter2p_Info_TypeDef;
'''
source = (prefix
          + '#define USE_I6X_RECEIVER 1\n'
          + cut(header, '#define SHOOT_RC_MODE_CHANNEL', '/**\n * @brief 发射状态机模式。')
          + cut(shoot, '#define LARGE_ARMOR_WIDTH', '/*******下位机火控开关')
          + cut(shoot, 'static fp32 basic_tarspeed', 'static void shoot_set_mode(')
          + cut(shoot, 'static void trigger_motor_turn_back(shoot_control_t *trigger_back)\n{',
                'static void shoot_bullet_control(shoot_control_t *bullet_control)\n{')
          + cut(shoot, 'static void shoot_speed_control(shoot_control_t *shoot_control)\n{',
                'void set_interrupt_flag_end_L(void)')
          + cut(heat, '#define SIM_HEAT_MAX', 'static float Feedforward_Freq_Calc(float q_res, float a, float d, uint16_t n, float max_freq);')
          + cut(heat, 'fp32 shoot_heat_watch', 'float Get_Virtual_Heat(void)')
          + cut(lpf, 'float LowPassFilter2p_Update(', '//------')
          + r'''
static shoot_control_t ref_ctrl = {.shoot_mode = SHOOT_CONTINUE_BULLET};
static RC_ctrl_t ref_rc;
static LowPassFilter2p_Info_TypeDef ref_lpf = {.Alpha = {1.725709860247969f, -0.75594777109163436f, 0.030237910843665373f}};
void ref_heat_set(float cooling, float limit, uint16_t heat1, uint32_t tick)
{ ref_cooling = cooling; ref_limit = limit; ref_heat1 = heat1; ref_tick = tick; }
float ref_heat_update(float a, float b, float speed, float target)
{ Shoot_Virtual_Heat_Update(a, b, speed, target); return g_virtual_heat.current_heat; }
float ref_speed_update(float bullet, uint32_t tick)
{ ref_bullet_speed = bullet; ref_tick = tick; ref_ctrl.shoot_speed_limit = 0; shoot_speed_control(&ref_ctrl); return basic_tarspeed; }
float ref_speed_ema(void) { return spd_ema_speed; }
float ref_jam(float target, float feedback)
{ ref_ctrl.shoot_rc = &ref_rc; ref_ctrl.Trigger_Targetspeed = target; ref_ctrl.Trigger_FBspeed_filtered = feedback;
  trigger_motor_turn_back(&ref_ctrl); return ref_ctrl.Trigger_Actualspeed; }
void ref_filter(float x, float* latest, float* delayed)
{ LowPassFilter2p_Update(&ref_lpf, x); *latest = ref_lpf.Output[2]; *delayed = ref_lpf.Output[0]; }
''')
stubs = [root / 'tests/chassis/stubs', root / 'tests/gimbal/stubs']
inc = stubs + [modules / n for n in ['Launcher', 'CMD', 'Motor', 'RMMotor', 'ControlAlgorithms', 'ControlAlgorithms/include',
                                     'Referee']] + \
    [root / 'Middlewares/Third_Party/LibXR/src' / n for n in ['structure', 'utils']] + [test]
with tempfile.TemporaryDirectory(prefix='wdr-launcher-') as temp:
    temp = Path(temp)
    (temp / 'reference.c').write_text(source)
    obj = temp / 'reference.o'
    subprocess.run(['cc', '-std=gnu11', '-O1', '-w', '-I' + str(test), '-c', str(temp / 'reference.c'),
                    '-o', str(obj)], check=True)
    exe = temp / 'launcher'
    subprocess.run(['c++', '-std=c++20', '-DLIBXR_DEFAULT_SCALAR=float', '-O1', '-g', '-Wall',
                    '-Wextra', '-fsanitize=address,undefined', '-static-libasan',
                    '-fno-omit-frame-pointer', *['-I' + str(p) for p in inc],
                    str(test / 'test_launcher.cpp'), str(modules / 'Launcher/launcher.cpp'),
                    str(obj), '-o', str(exe)], check=True)
    for scenario in ['behavior', 'heat', 'speed', 'jam', 'filter']:
        subprocess.run([str(exe), scenario], check=True)
