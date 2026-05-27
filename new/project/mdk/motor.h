#ifndef __MOTOR_H_
#define __MOTOR_H_

#include "zf_common_headfile.h"

// 初始化左右电机方向 GPIO 和 PWM 通道
void motor_init(void);
// 设置左右电机占空比控制量，参数范围 -90~90，正值前进、负值后退
void set_motor_speed(int left_duty, int right_duty);

#endif
