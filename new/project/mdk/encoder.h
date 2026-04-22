#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "zf_common_headfile.h"

// 使用 extern 声明全局变量，这样 main.c 和未来的 pid.c 都可以直接读取这两个速度值
extern int16 left_speed;
extern int16 right_speed;
extern int16 left_speed_raw;
extern int16 right_speed_raw;

// 函数声明
void encoder_init(void);
void encoder_update(void);
int16 Distance_Measure(void);

#endif