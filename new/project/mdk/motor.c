#include "motor.h"

//#define DIR_1               ( IO_P50 ) 
//#define PWM_1               ( PWMD_CH2_P51 )
//#define DIR_2               ( IO_P52 ) 
//#define PWM_2               ( PWMD_CH4_P53 )

#define DIR_2               ( IO_P50 ) 
#define PWM_2               ( PWMD_CH2_P51 )
#define DIR_1               ( IO_P52 ) 
#define PWM_1               ( PWMD_CH4_P53 )

// 电机初始化函数
void motor_init(void)
{
    gpio_init(DIR_1, GPO, GPIO_HIGH, GPO_PUSH_PULL);   
    pwm_init(PWM_1, 17000, 0);                         
    gpio_init(DIR_2, GPO, GPIO_HIGH, GPO_PUSH_PULL);   
    pwm_init(PWM_2, 17000, 0);                         
}

// 电机控制函数
void set_motor_speed(int left_duty, int right_duty)
{
    //right_duty = -right_duty; // 修正硬件镜像安装

    if(left_duty > 90) left_duty = 90;
    if(left_duty < -90) left_duty = -90;
    if(right_duty > 90) right_duty = 90;
    if(right_duty < -90) right_duty = -90;

    // 左后轮控制
    if(left_duty >= 0) {
        gpio_set_level(DIR_1, GPIO_HIGH);                                   
        pwm_set_duty(PWM_1, left_duty * (PWM_DUTY_MAX / 100));              
    } else {
        gpio_set_level(DIR_1, GPIO_LOW);                                    
        pwm_set_duty(PWM_1, (-left_duty) * (PWM_DUTY_MAX / 100));           
    }

    // 右后轮控制
    if(right_duty >= 0) {
        gpio_set_level(DIR_2, GPIO_HIGH);                                   
        pwm_set_duty(PWM_2, right_duty * (PWM_DUTY_MAX / 100));             
    } else {
        gpio_set_level(DIR_2, GPIO_LOW);                                    
        pwm_set_duty(PWM_2, (-right_duty) * (PWM_DUTY_MAX / 100));          
    }
}