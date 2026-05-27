#include "Shoot.h"
#include "Target_Ring.h"
#include "vision.h"

#define AIM_SERVO2_PWM_PIN          (PWME_CH2P_PA2)
#define AIM_SERVO2_FREQ             (50)

#define SHOOT_LASER_PIN             (IO_P67)

#define AIM_SERVO2_CENTER_DEG       (85.0f)
#define AIM_SERVO2_MIN_DEG          (60.0f)
#define AIM_SERVO2_MAX_DEG          (120.0f)
#define AIM_SERVO2_DIR              (-1.0f)

#define AIM_SERVO2_KP_DEG_PER_PIX   (0.35f)

#define AIM_FIRE_Y_MIN              (0)
#define AIM_FIRE_Y_MAX              (119)
#define AIM_FIRE_X_LIMIT_PIX        (90)

#define AIM_STABLE_NEED_TICKS       (6)
#define AIM_LASER_ON_TICKS          (100)
#define AIM_LOCK_TICKS              (100)

#define SHOOT_STATE_IDLE            (0)
#define SHOOT_STATE_AIM             (1)
#define SHOOT_STATE_FIRE            (2)
#define SHOOT_STATE_LOCK            (3)

volatile uint8 shoot_enable = 0;
volatile uint8 shoot_state = SHOOT_STATE_IDLE;
volatile uint8 shoot_laser_on_flag = 0;
volatile uint8 shoot_done_flag = 0;

volatile int16 shoot_servo2_angle_x10 = (int16)(AIM_SERVO2_CENTER_DEG * 10.0f);
volatile int16 shoot_target_error = 0;

static uint16 aim_stable_ticks = 0;
static uint16 laser_ticks = 0;
static uint16 lock_ticks = 0;
static float servo2_last_angle = AIM_SERVO2_CENTER_DEG;

static float clamp_f(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static uint16 servo2_duty(float angle)
{
    float duty;

    duty = (float)PWM_DUTY_MAX / (1000.0f / (float)AIM_SERVO2_FREQ) *
           (0.5f + angle / 90.0f);

    if (duty < 0.0f) duty = 0.0f;
    if (duty > (float)PWM_DUTY_MAX) duty = (float)PWM_DUTY_MAX;

    return (uint16)duty;
}

static void servo2_set_angle(float angle)
{
    angle = clamp_f(angle, AIM_SERVO2_MIN_DEG, AIM_SERVO2_MAX_DEG);
    servo2_last_angle = angle;
    shoot_servo2_angle_x10 = (int16)(angle * 10.0f);
    pwm_set_duty(AIM_SERVO2_PWM_PIN, servo2_duty(angle));
}

void shoot_laser_on(void)
{
    gpio_set_level(SHOOT_LASER_PIN, 1);
    shoot_laser_on_flag = 1;
}

void shoot_laser_off(void)
{
    gpio_set_level(SHOOT_LASER_PIN, 0);
    shoot_laser_on_flag = 0;
}

void shoot_init(void)
{
    pwm_init(AIM_SERVO2_PWM_PIN, AIM_SERVO2_FREQ, servo2_duty(AIM_SERVO2_CENTER_DEG));
    gpio_init(SHOOT_LASER_PIN, GPO, 0, GPO_PUSH_PULL);

    servo2_last_angle = AIM_SERVO2_CENTER_DEG;
    servo2_set_angle(AIM_SERVO2_CENTER_DEG);
    shoot_laser_off();

    shoot_state = SHOOT_STATE_IDLE;
    shoot_done_flag = 0;
    aim_stable_ticks = 0;
    laser_ticks = 0;
    lock_ticks = 0;
}

static void target_snapshot(uint8 *flag, uint8 *conf, uint8 *y, int16 *input)
{
    uint8 ea_state;

    ea_state = EA;
    EA = 0;

    *flag = target_flag;
    *conf = target_confidence;
    *y = target_y;
    *input = target_servo2_input;

    EA = ea_state;
}

static uint8 target_in_fire_window(uint8 flag, uint8 conf, uint8 y, int16 input)
{
    if (!flag) return 0;
    if (conf < 100) return 0;

    if (y < AIM_FIRE_Y_MIN || y > AIM_FIRE_Y_MAX) return 0;

    if (input > AIM_FIRE_X_LIMIT_PIX || input < -AIM_FIRE_X_LIMIT_PIX)
    {
        return 0;
    }

    return 1;
}

void shoot_task_5ms(void)
{
    float angle;
    int16 err;
    uint8 flag;
    uint8 conf;
    uint8 ty;
    uint8 fire_ok;

    target_snapshot(&flag, &conf, &ty, &err);
    shoot_target_error = err;

    if (!shoot_enable)
    {
        shoot_laser_off();
        shoot_state = SHOOT_STATE_IDLE;
        aim_stable_ticks = 0;
        laser_ticks = 0;
        lock_ticks = 0;
        servo2_set_angle(AIM_SERVO2_CENTER_DEG);
        return;
    }

    if (flag)
    {
        angle = AIM_SERVO2_CENTER_DEG +
                AIM_SERVO2_DIR * (float)err * AIM_SERVO2_KP_DEG_PER_PIX;
        servo2_set_angle(angle);
    }
    else
    {
        servo2_set_angle(servo2_last_angle);
    }

    fire_ok = target_in_fire_window(flag, conf, ty, err);

    switch (shoot_state)
    {
        case SHOOT_STATE_IDLE:
            shoot_laser_off();
            shoot_done_flag = 0;
            aim_stable_ticks = 0;
            laser_ticks = 0;

            if (fire_ok)
            {
                shoot_state = SHOOT_STATE_AIM;
            }
            break;

        case SHOOT_STATE_AIM:
            shoot_laser_off();

            if (fire_ok)
            {
                if (aim_stable_ticks < 65535) aim_stable_ticks++;

                if (aim_stable_ticks >= AIM_STABLE_NEED_TICKS)
                {
                    laser_ticks = 0;
                    shoot_state = SHOOT_STATE_FIRE;
                }
            }
            else
            {
                aim_stable_ticks = 0;
                shoot_state = SHOOT_STATE_IDLE;
            }
            break;

        case SHOOT_STATE_FIRE:
            shoot_laser_on();

            if (laser_ticks < 65535) laser_ticks++;

            if (laser_ticks >= AIM_LASER_ON_TICKS)
            {
                shoot_laser_off();
                shoot_done_flag = 1;
                lock_ticks = 0;
                shoot_state = SHOOT_STATE_LOCK;
            }
            break;

        case SHOOT_STATE_LOCK:
        default:
            shoot_laser_off();

            if (lock_ticks < 65535) lock_ticks++;

            if (lock_ticks >= AIM_LOCK_TICKS)
            {
                shoot_state = SHOOT_STATE_IDLE;
            }
            break;
    }
}