#include "shoot.h"
#include "Target_Ring.h"
#include "vision.h"

#define AIM_SERVO2_PWM_PIN          (PWME_CH2P_PA2)
#define AIM_SERVO2_FREQ             (50)
#define SHOOT_LASER_PIN             (IO_P67)

#define AIM_SERVO2_CENTER_DEG       (90.0f)
#define AIM_SERVO2_MIN_DEG          (30.0f)
#define AIM_SERVO2_MAX_DEG          (150.0f)

#define SHOOT_STATE_IDLE            (0)
#define SHOOT_STATE_LOCK_TARGET     (1)
#define SHOOT_STATE_AIM_PRESET      (2)
#define SHOOT_STATE_HOLD            (3)
#define SHOOT_STATE_FIRE            (4)
#define SHOOT_STATE_LOCKOUT         (5)

#define AIM_CALIB_MODE              0
#define AIM_CALIB_ANGLE_DEG         90.0f
#define AIM_CALIB_LASER_ON          1

#define SHOOT_MODULE_ENABLE         0

#define AIM_LOCK_CONF_MIN           (100)
#define AIM_LOCK_SCORE_MIN          (55)
#define AIM_LOCK_VOTES_MIN          (2)
#define AIM_LOCK_RELIABLE_MIN       (2)

#define AIM_FIRE_CONF_MIN           (100)
#define AIM_FIRE_Y_MIN              (80)
#define AIM_FIRE_Y_MAX              (98)
#define AIM_FIRE_OUTER_MIN          (27)
#define AIM_FIRE_OUTER_MAX          (65)
#define AIM_FIRE_STABLE_TICKS       (2)

#define AIM_PRESET_TICKS            (12)
#define AIM_HOLD_TIMEOUT_TICKS      (200)
#define AIM_LASER_ON_TICKS          (12)
#define AIM_LOCKOUT_TICKS           (40)
#define TARGET_LOST_ABORT_TICKS     (30)

typedef struct
{
    int16 x;
    float angle;
} laser_aim_point_t;

static const laser_aim_point_t laser_aim_table[] =
{
    { 47,  125.0f },
    { 73,  105.0f },
    { 85,  100.0f },
    { 95,   90.0f },
    { 112,  80.0f },
    { 118,  75.0f },
    { 128,  65.0f },
};

volatile uint8 shoot_enable = 0;
volatile uint8 shoot_state = SHOOT_STATE_IDLE;
volatile uint8 shoot_laser_on_flag = 0;
volatile uint8 shoot_done_flag = 0;
volatile uint8 shoot_slow_request = 0;
volatile uint8 shoot_stop_request = 0;
volatile uint8 shoot_reverse_request = 0;
volatile uint8 shoot_forward_request = 0;

volatile int16 shoot_servo2_angle_x10 = (int16)(AIM_SERVO2_CENTER_DEG * 10.0f);
volatile int16 shoot_target_error = 0;
volatile int16 shoot_aim_error_used = 0;
volatile int16 shoot_aim_error_predict = 0;

static uint16 state_ticks = 0;
static uint16 laser_ticks = 0;
static uint16 lockout_ticks = 0;
static uint16 lost_ticks = 0;
static uint16 fire_stable_ticks = 0;

static float servo2_hold_angle = AIM_SERVO2_CENTER_DEG;

static int16 lock_x = 0;

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

static float lookup_laser_angle(int16 x)
{
    uint8 i;
    float x0;
    float x1;
    float a0;
    float a1;
    uint8 table_count;

    table_count = sizeof(laser_aim_table) / sizeof(laser_aim_table[0]);

    if (x <= laser_aim_table[0].x)
    {
        return laser_aim_table[0].angle;
    }

    for (i = 0; i < table_count - 1; i++)
    {
        if (x <= laser_aim_table[i + 1].x)
        {
            x0 = (float)laser_aim_table[i].x;
            x1 = (float)laser_aim_table[i + 1].x;
            a0 = laser_aim_table[i].angle;
            a1 = laser_aim_table[i + 1].angle;

            return a0 + (a1 - a0) * ((float)x - x0) / (x1 - x0);
        }
    }

    return laser_aim_table[table_count - 1].angle;
}

void shoot_init(void)
{
    pwm_init(AIM_SERVO2_PWM_PIN, AIM_SERVO2_FREQ, servo2_duty(AIM_SERVO2_CENTER_DEG));
    gpio_init(SHOOT_LASER_PIN, GPO, 0, GPO_PUSH_PULL);

    servo2_hold_angle = AIM_SERVO2_CENTER_DEG;
    servo2_set_angle(AIM_SERVO2_CENTER_DEG);
    shoot_laser_off();

    shoot_state = SHOOT_STATE_IDLE;
    shoot_done_flag = 0;
    shoot_slow_request = 0;
    shoot_stop_request = 0;
    shoot_reverse_request = 0;
    shoot_forward_request = 0;
    state_ticks = 0;
    laser_ticks = 0;
    lockout_ticks = 0;
    lost_ticks = 0;
    fire_stable_ticks = 0;
}

static void target_snapshot(uint8 *flag,
                            uint8 *stage,
                            uint8 *conf,
                            uint8 *x,
                            uint8 *y,
                            uint8 *outer_w,
                            uint8 *score,
                            uint8 *votes,
                            uint8 *reliable_votes,
                            int16 *input)
{
    uint8 ea_state;

    ea_state = EA;
    EA = 0;

    *flag = target_flag;
    *stage = target_stage;
    *conf = target_confidence;
    *x = target_x;
    *y = target_y;
    *outer_w = target_outer_w;
    *score = target_score;
    *votes = target_votes;
    *reliable_votes = target_reliable_votes;
    *input = target_servo2_input;

    EA = ea_state;
}

static uint8 target_stage_is_fresh(uint8 stage)
{
    return (stage == 3 || stage == 6 || stage == 9) ? 1 : 0;
}

static uint8 target_lock_ok(uint8 flag,
                            uint8 stage,
                            uint8 conf,
                            uint8 score,
                            uint8 votes,
                            uint8 reliable_votes)
{
    if (!flag) return 0;
    if (!target_stage_is_fresh(stage)) return 0;
    if (conf < AIM_LOCK_CONF_MIN) return 0;
    if (score < AIM_LOCK_SCORE_MIN) return 0;
    if (votes < AIM_LOCK_VOTES_MIN) return 0;
    if (reliable_votes < AIM_LOCK_RELIABLE_MIN) return 0;

    return 1;
}

static uint8 target_fire_window_ok(uint8 flag,
                                   uint8 stage,
                                   uint8 conf,
                                   uint8 y,
                                   uint8 outer_w,
                                   uint8 votes)
{
    if (!flag) return 0;
    if (!target_stage_is_fresh(stage)) return 0;
    if (conf < AIM_FIRE_CONF_MIN) return 0;
    if (votes < AIM_LOCK_VOTES_MIN) return 0;
    if (y < AIM_FIRE_Y_MIN || y > AIM_FIRE_Y_MAX) return 0;
    if (outer_w < AIM_FIRE_OUTER_MIN || outer_w > AIM_FIRE_OUTER_MAX) return 0;

    return 1;
}

static void clear_motion_requests(void)
{
    shoot_slow_request = 0;
    shoot_stop_request = 0;
    shoot_reverse_request = 0;
    shoot_forward_request = 0;
}

static void shoot_goto_state(uint8 state)
{
    shoot_state = state;
    state_ticks = 0;

    if (state == SHOOT_STATE_IDLE)
    {
        laser_ticks = 0;
        lockout_ticks = 0;
        lost_ticks = 0;
        fire_stable_ticks = 0;
    }
    else if (state == SHOOT_STATE_AIM_PRESET)
    {
        fire_stable_ticks = 0;
    }
    else if (state == SHOOT_STATE_HOLD)
    {
        fire_stable_ticks = 0;
    }
    else if (state == SHOOT_STATE_FIRE)
    {
        laser_ticks = 0;
    }
    else if (state == SHOOT_STATE_LOCKOUT)
    {
        lockout_ticks = 0;
    }
}

static void capture_lock(uint8 x)
{
    lock_x = (int16)x;
    servo2_hold_angle = lookup_laser_angle(lock_x);

    shoot_aim_error_used = lock_x;
    shoot_aim_error_predict = (int16)(servo2_hold_angle * 10.0f);
}

void shoot_task_5ms(void)
{
#if !SHOOT_MODULE_ENABLE
    shoot_laser_off();
    clear_motion_requests();
    shoot_done_flag = 0;
    shoot_state = SHOOT_STATE_IDLE;
    servo2_set_angle(AIM_SERVO2_CENTER_DEG);
#else
    uint8 flag;
    uint8 stage;
    uint8 conf;
    uint8 x;
    uint8 y;
    uint8 outer_w;
    uint8 score;
    uint8 votes;
    uint8 reliable_votes;
    uint8 fresh;
    uint8 fire_window;
    int16 input;

    target_snapshot(&flag, &stage, &conf, &x, &y, &outer_w,
                    &score, &votes, &reliable_votes, &input);

#if AIM_CALIB_MODE
    clear_motion_requests();
    servo2_set_angle(AIM_CALIB_ANGLE_DEG);

    if (AIM_CALIB_LASER_ON)
    {
        shoot_laser_on();
    }
    else
    {
        shoot_laser_off();
    }

    shoot_state = 9;
    return;
#endif

    shoot_target_error = input;

    fresh = (flag && target_stage_is_fresh(stage)) ? 1 : 0;
    fire_window = target_fire_window_ok(flag, stage, conf, y, outer_w, votes);

    if (!shoot_enable)
    {
        shoot_laser_off();
        clear_motion_requests();
        shoot_done_flag = 0;
        shoot_goto_state(SHOOT_STATE_IDLE);
        servo2_set_angle(AIM_SERVO2_CENTER_DEG);
        return;
    }

    clear_motion_requests();

    if (state_ticks < 65535) state_ticks++;

    if (fresh)
    {
        lost_ticks = 0;
    }
    else if (lost_ticks < 65535)
    {
        lost_ticks++;
    }

    switch (shoot_state)
    {
        case SHOOT_STATE_IDLE:
            shoot_laser_off();
            shoot_done_flag = 0;
            fire_stable_ticks = 0;

            if (target_lock_ok(flag, stage, conf, score, votes, reliable_votes))
            {
                capture_lock(x);
                shoot_goto_state(SHOOT_STATE_LOCK_TARGET);
            }
            else
            {
                servo2_set_angle(AIM_SERVO2_CENTER_DEG);
            }
            break;

        case SHOOT_STATE_LOCK_TARGET:
            shoot_laser_off();
            shoot_slow_request = 1;
            servo2_set_angle(servo2_hold_angle);
            shoot_goto_state(SHOOT_STATE_AIM_PRESET);
            break;

        case SHOOT_STATE_AIM_PRESET:
            shoot_laser_off();
            shoot_slow_request = 1;
            servo2_set_angle(servo2_hold_angle);

            if (state_ticks >= AIM_PRESET_TICKS)
            {
                shoot_goto_state(SHOOT_STATE_HOLD);
            }
            else if (lost_ticks >= TARGET_LOST_ABORT_TICKS)
            {
                shoot_goto_state(SHOOT_STATE_IDLE);
            }
            break;

        case SHOOT_STATE_HOLD:
            shoot_laser_off();
            shoot_slow_request = 1;
            servo2_set_angle(servo2_hold_angle);

            if (fire_window)
            {
                if (fire_stable_ticks < 65535) fire_stable_ticks++;
            }
            else
            {
                fire_stable_ticks = 0;
            }

            if (fire_stable_ticks >= AIM_FIRE_STABLE_TICKS)
            {
                shoot_goto_state(SHOOT_STATE_FIRE);
            }
            else if (state_ticks >= AIM_HOLD_TIMEOUT_TICKS)
            {
                shoot_goto_state(SHOOT_STATE_LOCKOUT);
            }
            else if (lost_ticks >= TARGET_LOST_ABORT_TICKS)
            {
                shoot_goto_state(SHOOT_STATE_IDLE);
            }
            break;

        case SHOOT_STATE_FIRE:
            shoot_slow_request = 1;
            servo2_set_angle(servo2_hold_angle);
            shoot_laser_on();

            if (laser_ticks < 65535) laser_ticks++;

            if (laser_ticks >= AIM_LASER_ON_TICKS)
            {
                shoot_laser_off();
                shoot_done_flag = 1;
                shoot_goto_state(SHOOT_STATE_LOCKOUT);
            }
            break;

        case SHOOT_STATE_LOCKOUT:
        default:
            shoot_laser_off();
            shoot_slow_request = 1;
            servo2_set_angle(servo2_hold_angle);

            if (lockout_ticks < 65535) lockout_ticks++;

            if (lockout_ticks >= AIM_LOCKOUT_TICKS)
            {
                shoot_goto_state(SHOOT_STATE_IDLE);
            }
            break;
    }
#endif
}
