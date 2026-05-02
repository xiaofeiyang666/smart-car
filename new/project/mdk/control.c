#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"

/* 开源风格控制：
 * 固定行段偏差 PD 控舵，编码器 PI 控速，弯道内轮减速。
 */
#define MAX_SPEED_PULSES          4000
#define BASE_SPEED_MATCH_GAIN     1.05f

#define SERVO_OUT_LIMIT_LEFT      (SERVO_CENTER - SERVO_L_MAX)
#define SERVO_OUT_LIMIT_RIGHT     (SERVO_R_MAX - SERVO_CENTER)
#define SERVO_DEADBAND_PIX        1.0f

#define BIAS_FILTER_ALPHA         0.55f
#define PREVIEW_FILTER_ALPHA      0.35f

#define SPEED_CURVE_START         4.0f
#define SPEED_CURVE_K             0.020f
#define SPEED_SCALE_MIN           0.45f
#define SPEED_SCALE_MAX           1.00f
#define SPEED_ACCEL_UP_ALPHA      0.05f
#define SPEED_ACCEL_DOWN_ALPHA    0.45f
#define LOW_CONF_TH               35
#define LOW_CONF_SPEED_SCALE      0.70f

#define DIFF_STEER_DEADBAND       2.0f
#define DIFF_MAX_RATIO            0.32f

volatile uint8 print_flag = 0;

int target_speed_base = 15;
float servo_kp = 0.38f;
float servo_kd = 0.14f;
float servo_kff = 0.16f;

static float abs_f(float x)
{
    return (x >= 0.0f) ? x : -x;
}

static float clamp_f(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static float clamp_steer_f(float x)
{
    if (x > SERVO_OUT_LIMIT_RIGHT) return SERVO_OUT_LIMIT_RIGHT;
    if (x < -SERVO_OUT_LIMIT_LEFT) return -SERVO_OUT_LIMIT_LEFT;
    return x;
}

static float steer_limit_for_side(float steer)
{
    if (steer >= 0.0f) return SERVO_OUT_LIMIT_RIGHT;
    return SERVO_OUT_LIMIT_LEFT;
}

void control_init(void)
{
    left_motor_speed_pid_init(0.10f, 0.002f, 0.0f, 90, 90);
    right_motor_speed_pid_init(0.10f, 0.002f, 0.0f, 90, 90);
}

void control_loop(void)
{
    static uint8 loop_cnt = 0;
    static uint8 filter_init = 0;
    static uint8 speed_filter_init = 0;
    static float bias_filtered = 0.0f;
    static float preview_filtered = 0.0f;
    static float target_pulses_smooth = 0.0f;
    static float error_last = 0.0f;

    int bias_raw;
    int preview_raw;
    int preview_far_raw;
    int curve_raw;
    float error;
    float error_delta;
    float steer_out;
    float abs_steer;
    float steer_limit;

    int base_pulses;
    float curve_score;
    float speed_scale;
    float speed_alpha;
    int target_pulses_expect;
    int target_pulses;
    int diff_speed;
    int left_target_pulses;
    int right_target_pulses;

    bias_raw = (int)camera_bias_raw;
    if (bias_raw > 90) bias_raw = 90;
    if (bias_raw < -90) bias_raw = -90;

    preview_raw = (int)camera_preview_raw;
    if (preview_raw > 90) preview_raw = 90;
    if (preview_raw < -90) preview_raw = -90;

    preview_far_raw = (int)camera_preview_far_raw;
    if (preview_far_raw > 90) preview_far_raw = 90;
    if (preview_far_raw < -90) preview_far_raw = -90;

    curve_raw = (int)camera_curve_raw;
    if (curve_raw > 90) curve_raw = 90;
    if (curve_raw < -90) curve_raw = -90;

    if (!filter_init)
    {
        bias_filtered = (float)bias_raw;
        preview_filtered = (float)preview_raw;
        error_last = 0.0f;
        filter_init = 1;
    }

    bias_filtered = bias_filtered * (1.0f - BIAS_FILTER_ALPHA) + (float)bias_raw * BIAS_FILTER_ALPHA;
    preview_filtered = preview_filtered * (1.0f - PREVIEW_FILTER_ALPHA) + (float)preview_raw * PREVIEW_FILTER_ALPHA;

    error = bias_filtered;
    if ((error <= SERVO_DEADBAND_PIX) && (error >= -SERVO_DEADBAND_PIX))
    {
        error = 0.0f;
    }
    else if (error > 0.0f)
    {
        error -= SERVO_DEADBAND_PIX;
    }
    else
    {
        error += SERVO_DEADBAND_PIX;
    }

    error_delta = error - error_last;
    error_last = error;

    steer_out = (-servo_kp * error) - (servo_kd * error_delta) + (servo_kff * preview_filtered);
    steer_out = clamp_steer_f(steer_out);
    servo_set_angle(SERVO_CENTER + steer_out);

    base_pulses = (int)((float)(((long)target_speed_base * MAX_SPEED_PULSES) / 90) * BASE_SPEED_MATCH_GAIN);

    curve_score = abs_f(preview_filtered) +
                  0.45f * abs_f((float)preview_far_raw) +
                  0.35f * abs_f((float)curve_raw) +
                  0.70f * abs_f(steer_out);

    if (curve_score <= SPEED_CURVE_START)
    {
        speed_scale = SPEED_SCALE_MAX;
    }
    else
    {
        speed_scale = SPEED_SCALE_MAX - (curve_score - SPEED_CURVE_START) * SPEED_CURVE_K;
    }
    speed_scale = clamp_f(speed_scale, SPEED_SCALE_MIN, SPEED_SCALE_MAX);

    if (camera_confidence < LOW_CONF_TH)
    {
        speed_scale *= LOW_CONF_SPEED_SCALE;
        speed_scale = clamp_f(speed_scale, SPEED_SCALE_MIN, SPEED_SCALE_MAX);
    }

    target_pulses_expect = (int)((float)base_pulses * speed_scale);
    if (target_pulses_expect < 0) target_pulses_expect = 0;

    if (!speed_filter_init)
    {
        target_pulses_smooth = (float)target_pulses_expect;
        speed_filter_init = 1;
    }
    else
    {
        if ((float)target_pulses_expect > target_pulses_smooth)
        {
            speed_alpha = SPEED_ACCEL_UP_ALPHA;
        }
        else
        {
            speed_alpha = SPEED_ACCEL_DOWN_ALPHA;
        }
        target_pulses_smooth += ((float)target_pulses_expect - target_pulses_smooth) * speed_alpha;
    }

    target_pulses = (int)target_pulses_smooth;
    if (target_pulses < 0) target_pulses = 0;

    abs_steer = abs_f(steer_out);
    diff_speed = 0;
    if (abs_steer > DIFF_STEER_DEADBAND)
    {
        steer_limit = steer_limit_for_side(steer_out);
        if (steer_limit < 1.0f) steer_limit = 1.0f;
        diff_speed = (int)((float)target_pulses * DIFF_MAX_RATIO * (abs_steer / steer_limit));
        if (diff_speed > (int)((float)target_pulses * DIFF_MAX_RATIO))
        {
            diff_speed = (int)((float)target_pulses * DIFF_MAX_RATIO);
        }
    }

    if (steer_out > 0.0f)
    {
        left_target_pulses = target_pulses + diff_speed;
        right_target_pulses = target_pulses - diff_speed;
    }
    else if (steer_out < 0.0f)
    {
        left_target_pulses = target_pulses - diff_speed;
        right_target_pulses = target_pulses + diff_speed;
    }
    else
    {
        left_target_pulses = target_pulses;
        right_target_pulses = target_pulses;
    }

    if (left_target_pulses < 0) left_target_pulses = 0;
    if (right_target_pulses < 0) right_target_pulses = 0;

    encoder_update();
    left_motor_speed_pid_calc(left_target_pulses, left_speed);
    right_motor_speed_pid_calc(right_target_pulses, right_speed);
    set_motor_speed((int)left_motor_speedpid.output, (int)right_motor_speedpid.output);

    loop_cnt++;
    if (loop_cnt >= 5)
    {
        loop_cnt = 0;
        print_flag = 1;
    }
}
