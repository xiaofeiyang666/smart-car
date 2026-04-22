#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

void motor_init(void);
void set_motor_speed(int left_duty, int right_duty);

#endif