#ifndef _SERVO_H
#define _SERVO_H

#include "zf_common_headfile.h"

// 舵机硬件参数宏定义 (请根据你的主板实际引脚修改)
#define SERVO_PWM_PIN           (PWME_CH1P_PA0)   // 舵机连接的引脚
#define SERVO_FREQ              (50)              // 舵机频率 (通常为50Hz)

// 舵机机械限位 (需要上车实际调试)
#define SERVO_CENTER            (90.0f)           // 中值角度
#define SERVO_L_MAX             (75.0f)           // 左打死最大角度
#define SERVO_R_MAX             (100.0f)          // 右打死最大角度；若拉杆顶死或舵机发热，先降回100~102

// 占空比计算公式宏
#define SERVO_DUTY(angle)       ((float)PWM_DUTY_MAX / (1000.0f / (float)SERVO_FREQ) * (0.5f + (float)(angle) / 90.0f))

void servo_init(void);
void servo_set_angle(float angle);

#endif