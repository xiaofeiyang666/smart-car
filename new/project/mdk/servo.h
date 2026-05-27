#ifndef _SERVO_H
#define _SERVO_H

#include "zf_common_headfile.h"

#define SERVO_PWM_PIN           (PWME_CH1P_PA0)
#define SERVO_FREQ              (50)

/* ===================== 舵机机械限幅 =====================
 * 这里限制的是舵机最终 PWM 对应的物理角度，防止轮胎打死卡底盘。
 * SERVO_CENTER：舵机中位。车静止时轮子不正，先调这里。
 * SERVO_L_MAX / SERVO_R_MAX：左右最大角。
 *   某一侧打死卡住：把对应侧限幅往 CENTER 收。
 *   过弯打不够：在不蹭底盘的前提下，适当放开对应侧限幅。
 * 注意：这里是最终物理保护，不等于舵机 PID 增益；缩小限幅会让车更不容易打死，
 * 但如果缩得太多，会出现弯道转不过去。
 */
#define SERVO_CENTER            (97.0f)
#define SERVO_L_MAX             (84.0f)
#define SERVO_R_MAX             (110.0f)

#define SERVO_DUTY(angle)       ((float)PWM_DUTY_MAX / (1000.0f / (float)SERVO_FREQ) * (0.5f + (float)(angle) / 90.0f))
#define SERVO_DUTY_CENTER       (SERVO_DUTY(SERVO_CENTER))
#define SERVO_DUTY_L_MAX        (SERVO_DUTY(SERVO_L_MAX))
#define SERVO_DUTY_R_MAX        (SERVO_DUTY(SERVO_R_MAX))
#define SERVO_DUTY_PER_DEGREE   ((float)PWM_DUTY_MAX / (1000.0f / (float)SERVO_FREQ) / 90.0f)

extern volatile uint32 servo_pwm_output;

void servo_init(void);
void servo_set_angle(float angle);
void servo_set_duty(float duty);

#endif
