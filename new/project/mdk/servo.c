#include "servo.h"

volatile uint32 servo_pwm_output = 0;

static float servo_clamp_duty(float duty)
{
    float min_duty;
    float max_duty;

    if (SERVO_DUTY_L_MAX < SERVO_DUTY_R_MAX)
    {
        min_duty = SERVO_DUTY_L_MAX;
        max_duty = SERVO_DUTY_R_MAX;
    }
    else
    {
        min_duty = SERVO_DUTY_R_MAX;
        max_duty = SERVO_DUTY_L_MAX;
    }

    if (duty < min_duty) duty = min_duty;
    if (duty > max_duty) duty = max_duty;
    return duty;
}

void servo_init(void)
{
    pwm_init(SERVO_PWM_PIN, SERVO_FREQ, 0);
    servo_set_angle(SERVO_CENTER);
}

void servo_set_angle(float angle)
{
    if (angle > SERVO_R_MAX) angle = SERVO_R_MAX;
    if (angle < SERVO_L_MAX) angle = SERVO_L_MAX;
    servo_set_duty(SERVO_DUTY(angle));
}

void servo_set_duty(float duty)
{
    duty = servo_clamp_duty(duty);
    servo_pwm_output = (uint32)duty;
    pwm_set_duty(SERVO_PWM_PIN, servo_pwm_output);
}
