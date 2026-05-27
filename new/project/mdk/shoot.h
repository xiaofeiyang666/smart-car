#ifndef SHOOT_H
#define SHOOT_H

#include "zf_common_headfile.h"

extern volatile uint8 shoot_enable;
extern volatile uint8 shoot_state;
extern volatile uint8 shoot_laser_on_flag;
extern volatile uint8 shoot_done_flag;

extern volatile int16 shoot_servo2_angle_x10;
extern volatile int16 shoot_target_error;

void shoot_init(void);
void shoot_task_5ms(void);
void shoot_laser_on(void);
void shoot_laser_off(void);

#endif