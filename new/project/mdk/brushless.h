#ifndef __BRUSHLESS_H
#define __BRUSHLESS_H

#include "zf_common_headfile.h"

// 初始化无刷电调 PWM 通道
void fan_init(void);
// 设置左右无刷电调占空比计数值，50Hz 下 500~1000 约对应 0%~100%
void set_fan_speed(int left, int right);

#endif
