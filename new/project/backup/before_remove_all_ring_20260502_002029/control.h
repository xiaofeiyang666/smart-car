#ifndef _CONTROL_H
#define _CONTROL_H
#include "zf_common_headfile.h"

extern int target_speed_base;


void control_init(void);
void control_loop(void);

extern volatile uint8 print_flag;
extern volatile int16 control_debug_steer_out;
extern volatile int16 control_debug_curve_x100;
extern volatile int16 control_debug_gyro_z;
extern volatile int16 control_ring_yaw_abs;
extern volatile uint8 control_debug_straight_lock;
extern volatile uint8 control_debug_s_transition;
#endif
