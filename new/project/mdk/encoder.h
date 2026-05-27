#ifndef __ENCODER_H_
#define __ENCODER_H_

#include "zf_common_headfile.h"

// 左右轮速度反馈，单位为本控制周期内的编码器脉冲数
extern int16 left_speed;
extern int16 right_speed;
// 左右轮原始脉冲值，已按当前硬件接线做左右映射/方向修正
extern int16 left_speed_raw;
extern int16 right_speed_raw;

// 初始化左右轮正交编码器
void encoder_init(void);
// 读取本周期编码器脉冲、更新速度反馈并清空硬件计数
void encoder_update(void);
// 读取累计行驶距离脉冲并清零，返回两次调用之间的平均轮速脉冲累计
int16 Distance_Measure(void);

#endif
