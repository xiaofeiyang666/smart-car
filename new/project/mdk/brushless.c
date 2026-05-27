#include "brushless.h"

// 无刷电调 PWM 控制频率，单位 Hz；常见电调支持 50~300Hz
#define FREQ               (50)                                                // 控制频率为50HZ，最高支持300HZ
// 左右无刷电调 PWM 通道
#define PWM_1              (PWMF_CH3_PA5)
#define PWM_2              (PWMF_CH4_PA7)

uint16 duty = 0;                    // 预留占空比变量，当前主流程未使用

// 无刷电调控制量说明：
// 在 50Hz 下，1ms~2ms 脉宽对应 0%~100% 输出，换算到 PWM_DUTY_MAX=10000 时约为 500~1000。
// 50Hz： 0%~100% 对应  500~1000
// 100Hz：0%~100% 对应 1000~2000
// 200Hz：0%~100% 对应 2000~4000
// 300Hz：0%~100% 对应 3000~6000

// 初始化两个无刷电调 PWM 通道，占空比先置 0，避免上电误转
void fan_init(void)
{
	pwm_init(PWM_1, FREQ, 0);                   // PWM 通道1 初始化频率 50Hz  占空比初始为 0
  pwm_init(PWM_2, FREQ, 0);                   // PWM 通道2 初始化频率 50Hz  占空比初始为 0
}

/* 设置左右无刷电调控制量。
 * 参数 left：PWM_1 占空比计数值，50Hz 下 500~1000 约对应 0%~100%。
 * 参数 right：PWM_2 占空比计数值，含义同 left。
 */
void set_fan_speed(int left, int right)
{
	pwm_set_duty(PWM_1, left);
	pwm_set_duty(PWM_2, right);
}

