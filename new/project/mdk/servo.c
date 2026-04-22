#include "servo.h"

// 舵机初始化
void servo_init(void)
{
    // 初始化 PWM 引脚，初始占空比给 0，防止上电乱抽
    pwm_init(SERVO_PWM_PIN, SERVO_FREQ, 0); 
    
    // 初始化完成后，先让舵机回到中位
    servo_set_angle(SERVO_CENTER);
}

// 舵机角度控制函数 (带限幅保护)
void servo_set_angle(float angle)
{
    // 软件限幅，防止打角过大损坏机械拉杆或烧毁舵机
    if(angle > SERVO_R_MAX) angle = SERVO_R_MAX;
    if(angle < SERVO_L_MAX) angle = SERVO_L_MAX;
    
    // 计算并输出对应角度的 PWM 占空比
    pwm_set_duty(SERVO_PWM_PIN, (uint32)SERVO_DUTY(angle));
}