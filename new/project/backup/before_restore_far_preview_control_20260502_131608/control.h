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
extern volatile int16 control_debug_speed_x100;
#endif
