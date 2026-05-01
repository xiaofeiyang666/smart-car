#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "brushless.h"

// 目标速度（0~90）到编码器目标脉冲映射
#define MAX_SPEED_PULSES                 4000
#define BASE_SPEED_MATCH_GAIN            1.06f

// ===================== 精简控制参数（3 路况模式） =====================
// 路况模式：直道 / 普通弯 / 急弯
// 用曲率评分自动切换，去掉复杂状态机

// 前瞻行
#define PREVIEW_Y_NEAR                   (MT9V03X_H - 14)
#define PREVIEW_Y_FAR                    30

// 传感滤波
#define OFFSET_LPF_ALPHA_STRAIGHT        0.18f
#define OFFSET_LPF_ALPHA_CURVE           0.54f
#define PREVIEW_LPF_ALPHA                0.36f
#define GYRO_LPF_ALPHA                   0.30f

// 曲率评分（越大越弯）
#define SCORE_PREVIEW_FULL               14.0f
#define SCORE_DPREVIEW_FULL              2.2f
#define SCORE_OFFSET_FULL                28.0f
#define SCORE_DPREVIEW_WEIGHT            0.35f
#define SCORE_OFFSET_WEIGHT              0.20f

// 模式切换阈值（带滞回）
#define MODE_CURVE_ON                    0.46f
#define MODE_CURVE_OFF                   0.30f
#define MODE_SHARP_ON                    0.92f
#define MODE_SHARP_OFF                   0.72f

// 舵机
#define SERVO_SOFT_DEADBAND_PIX          2.8f
#define SERVO_OUT_LIMIT                  20.0f
#define RIGHT_TURN_STEER_MUL             1.12f
#define LEFT_TURN_STEER_MUL              1.04f
#define STEER_KP_ADD                     0.85f
#define STEER_KD_ADD                     0.20f
#define STEER_FF_BASE_MUL                0.28f
#define STEER_FF_ADD                     1.05f
#define STEER_FF_LIMIT                   8.2f
#define STEER_RAW_ALPHA_STRAIGHT         0.14f
#define STEER_RAW_ALPHA_SHARP            0.54f
#define STEER_STEP_STRAIGHT              0.40f
#define STEER_STEP_SHARP                 5.80f
#define STEER_REVERSE_DPREVIEW_TH        1.8f
#define STEER_REVERSE_STEP_MUL           1.35f

// 速度
#define SPEED_DROP_MAX                   0.34f
#define SPEED_CURVE_K                    0.78f
#define SPEED_STEER_K                    0.40f
#define SPEED_CURVE_MIN                  0.46f
#define SPEED_STEER_MIN                  0.58f
#define SPEED_SCALE_MIN                  0.34f
#define SPEED_SCALE_MAX                  1.06f
#define HARD_BRAKE_PREVIEW_TH            6.2f
#define HARD_BRAKE_DPREVIEW_TH           0.42f
#define HARD_BRAKE_KEEP                  0.66f
#define HARD_BRAKE_HOLD_CYCLES           7

// 目标速度斜坡（20ms 周期）
#define TARGET_STEP_UP_STRAIGHT          8.0f
#define TARGET_STEP_UP_SHARP             1.5f
#define TARGET_STEP_DOWN                 220.0f

// 后轮差速
#define DIFF_K_BASE                      0.40f
#define DIFF_K_GAIN                      0.95f
#define DIFF_STEER_DEADBAND              2.0f
#define RIGHT_TURN_DIFF_MUL              1.08f
#define LEFT_TURN_DIFF_MUL               1.00f
#define DIFF_FILTER_ALPHA                0.22f
#define DIFF_STEP_STRAIGHT               8.0f
#define DIFF_STEP_SHARP                  20.0f
#define DIFF_STEP_RIGHT_MUL              0.80f
#define DIFF_CAP_RATIO                   0.22f
#define DIFF_CAP_MIN                     12

// 内轮最低速度保护（带滞回）
#define DIFF_INNER_MIN_RATIO             0.24f
#define DIFF_INNER_MIN_STEER_ON          11.0f
#define DIFF_INNER_MIN_SCORE_ON          0.78f
#define DIFF_INNER_MIN_STEER_OFF         8.0f
#define DIFF_INNER_MIN_SCORE_OFF         0.58f

volatile uint8 print_flag = 0;

// 外部保留接口：基础速度（0~90）
int target_speed_base = 20;

// 三个主控制参数
float servo_kp = 0.36f;
float servo_kd = 0.14f;
float servo_kff = 0.22f;

typedef enum
{
    ROAD_STRAIGHT = 0,
    ROAD_CURVE    = 1,
    ROAD_SHARP    = 2
} road_mode_e;

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

void control_init(void)
{
    left_motor_speed_pid_init(0.10f, 0.002f, 0.0f, 90, 90);
    right_motor_speed_pid_init(0.10f, 0.002f, 0.0f, 90, 90);
}

void control_loop(void)
{
    static uint8 loop_cnt = 0;
    static uint8 lpf_init = 0;
    static uint8 hard_brake_cnt = 0;
    static uint8 inner_min_enable = 0;
    static road_mode_e road_mode = ROAD_STRAIGHT;

    static float offset_filtered = 0.0f;
    static float preview_filtered = 0.0f;
    static float preview_last = 0.0f;
    static float gyro_z_filtered = 0.0f;
    static float steer_raw_filtered = 0.0f;
    static float steer_out_last = 0.0f;
    static float target_pulses_ramped = 0.0f;
    static float diff_filtered = 0.0f;
    static float diff_limited = 0.0f;

    int near_y;
    int far_y;
    int near_mid;
    int far_mid;

    int offset_raw;

    float preview_raw;
    float preview_delta;
    float preview_delta_abs;
    float offset_lpf_alpha;
    float gyro_z_actual;

    float offset_ctrl;
    float abs_preview;
    float abs_offset;

    float curve_score;
    float score_from_preview;
    float score_from_dpreview;
    float score_from_offset;

    float mode_intensity;

    float kp_now;
    float kd_now;
    float kff_now;
    float ff_term;

    float steer_raw;
    float steer_alpha;
    float steer_step_limit;
    float delta_out;
    float steer_out;
    float abs_steer;

    float speed_scale_mode;
    float speed_scale_curve;
    float speed_scale_steer;
    float speed_scale_total;
    float steer_norm;

    int base_pulses;
    int target_pulses_des;
    int target_pulses_min;
    int target_pulses_max;
    int target_pulses;
    float target_step_up;

    float diff_k;
    float steer_for_diff;
    float diff_cmd;
    float diff_step_limit;
    float diff_delta;
    int diff_speed;
    int diff_cap;

    int left_target_pulses;
    int right_target_pulses;
    int inner_min_pulses;

    // 1) 前瞻采样
    near_y = PREVIEW_Y_NEAR;
    if (near_y >= MT9V03X_H) near_y = MT9V03X_H - 1;
    if (near_y <= search_end_line) near_y = search_end_line + 1;

    far_y = PREVIEW_Y_FAR;
    if (far_y <= search_end_line) far_y = search_end_line + 1;
    if (far_y >= MT9V03X_H) far_y = MT9V03X_H - 1;

    near_mid = mid_line_list[near_y];
    far_mid = mid_line_list[far_y];

    preview_raw = (float)(far_mid - near_mid);
    offset_raw = (int)final_mid_line - MID_W;

    if (!lpf_init)
    {
        offset_filtered = (float)offset_raw;
        preview_filtered = preview_raw;
        preview_last = preview_raw;
        gyro_z_filtered = 0.0f;
        steer_raw_filtered = 0.0f;
        steer_out_last = 0.0f;
        target_pulses_ramped = 0.0f;
        diff_filtered = 0.0f;
        diff_limited = 0.0f;
        hard_brake_cnt = 0;
        inner_min_enable = 0;
        road_mode = ROAD_STRAIGHT;
        lpf_init = 1;
    }

    // 2) 滤波
    if (abs_f(preview_filtered) >= 3.5f)
    {
        offset_lpf_alpha = OFFSET_LPF_ALPHA_CURVE;
    }
    else
    {
        offset_lpf_alpha = OFFSET_LPF_ALPHA_STRAIGHT;
    }

    offset_filtered = offset_filtered * (1.0f - offset_lpf_alpha) + (float)offset_raw * offset_lpf_alpha;
    preview_filtered = preview_filtered * (1.0f - PREVIEW_LPF_ALPHA) + preview_raw * PREVIEW_LPF_ALPHA;

    preview_delta = preview_filtered - preview_last;
    preview_last = preview_filtered;
    preview_delta_abs = abs_f(preview_delta);

    imu660ra_get_gyro();
    gyro_z_actual = imu660ra_gyro_transition(imu660ra_gyro_z);
    gyro_z_filtered = gyro_z_filtered * (1.0f - GYRO_LPF_ALPHA) + gyro_z_actual * GYRO_LPF_ALPHA;

    // 3) 曲率评分
    abs_preview = abs_f(preview_filtered);
    abs_offset = abs_f(offset_filtered);

    score_from_preview = abs_preview / SCORE_PREVIEW_FULL;
    score_from_dpreview = preview_delta_abs / SCORE_DPREVIEW_FULL;
    score_from_offset = abs_offset / SCORE_OFFSET_FULL;

    curve_score = score_from_preview + SCORE_DPREVIEW_WEIGHT * score_from_dpreview + SCORE_OFFSET_WEIGHT * score_from_offset;
    curve_score = clamp_f(curve_score, 0.0f, 1.6f);

    // 4) 路况模式切换（滞回）
    if (road_mode == ROAD_STRAIGHT)
    {
        if (curve_score >= MODE_SHARP_ON) road_mode = ROAD_SHARP;
        else if (curve_score >= MODE_CURVE_ON) road_mode = ROAD_CURVE;
    }
    else if (road_mode == ROAD_CURVE)
    {
        if (curve_score >= MODE_SHARP_ON) road_mode = ROAD_SHARP;
        else if (curve_score <= MODE_CURVE_OFF) road_mode = ROAD_STRAIGHT;
    }
    else
    {
        if (curve_score <= MODE_SHARP_OFF)
        {
            if (curve_score <= MODE_CURVE_OFF) road_mode = ROAD_STRAIGHT;
            else road_mode = ROAD_CURVE;
        }
    }

    // 模式强度（0/0.55/1.0）
    if (road_mode == ROAD_STRAIGHT) mode_intensity = 0.0f;
    else if (road_mode == ROAD_CURVE) mode_intensity = 0.55f;
    else mode_intensity = 1.0f;

    // 5) 偏差软死区
    offset_ctrl = offset_filtered;
    if (offset_ctrl <= SERVO_SOFT_DEADBAND_PIX && offset_ctrl >= -SERVO_SOFT_DEADBAND_PIX)
    {
        offset_ctrl = 0.0f;
    }
    else if (offset_ctrl > 0.0f)
    {
        offset_ctrl -= SERVO_SOFT_DEADBAND_PIX;
    }
    else
    {
        offset_ctrl += SERVO_SOFT_DEADBAND_PIX;
    }

    // 6) 舵机控制
    kp_now = servo_kp * (1.0f + STEER_KP_ADD * mode_intensity);
    kd_now = servo_kd * (1.0f + STEER_KD_ADD * mode_intensity);
    kff_now = servo_kff * (STEER_FF_BASE_MUL + STEER_FF_ADD * mode_intensity);

    ff_term = kff_now * preview_filtered;
    ff_term = clamp_f(ff_term, -STEER_FF_LIMIT, STEER_FF_LIMIT);

    steer_raw = (-kp_now * offset_ctrl) - (kd_now * gyro_z_filtered) + ff_term;
    if (steer_raw >= 0.0f) steer_raw *= RIGHT_TURN_STEER_MUL;
    else steer_raw *= LEFT_TURN_STEER_MUL;

    steer_alpha = STEER_RAW_ALPHA_STRAIGHT + (STEER_RAW_ALPHA_SHARP - STEER_RAW_ALPHA_STRAIGHT) * mode_intensity;
    steer_raw_filtered = steer_raw_filtered * (1.0f - steer_alpha) + steer_raw * steer_alpha;
    steer_raw = clamp_f(steer_raw_filtered, -SERVO_OUT_LIMIT, SERVO_OUT_LIMIT);

    steer_step_limit = STEER_STEP_STRAIGHT + (STEER_STEP_SHARP - STEER_STEP_STRAIGHT) * mode_intensity;
    if ((preview_delta_abs >= STEER_REVERSE_DPREVIEW_TH) &&
        ((steer_out_last > 0.0f && steer_raw < 0.0f) ||
         (steer_out_last < 0.0f && steer_raw > 0.0f)))
    {
        steer_step_limit *= STEER_REVERSE_STEP_MUL;
    }

    delta_out = steer_raw - steer_out_last;
    if (delta_out > steer_step_limit) steer_out = steer_out_last + steer_step_limit;
    else if (delta_out < -steer_step_limit) steer_out = steer_out_last - steer_step_limit;
    else steer_out = steer_raw;

    steer_out = clamp_f(steer_out, -SERVO_OUT_LIMIT, SERVO_OUT_LIMIT);
    steer_out_last = steer_out;
    abs_steer = abs_f(steer_out);

    servo_set_angle(SERVO_CENTER + steer_out);

    // 7) 速度控制
    base_pulses = (int)((float)(((long)target_speed_base * MAX_SPEED_PULSES) / 90) * BASE_SPEED_MATCH_GAIN);

    speed_scale_mode = 1.0f - SPEED_DROP_MAX * mode_intensity;

    speed_scale_curve = 1.0f / (1.0f + SPEED_CURVE_K * curve_score * curve_score);
    if (speed_scale_curve < SPEED_CURVE_MIN) speed_scale_curve = SPEED_CURVE_MIN;

    steer_norm = abs_steer / SERVO_OUT_LIMIT;
    if (steer_norm > 1.0f) steer_norm = 1.0f;
    speed_scale_steer = 1.0f - SPEED_STEER_K * steer_norm * steer_norm;
    if (speed_scale_steer < SPEED_STEER_MIN) speed_scale_steer = SPEED_STEER_MIN;

    speed_scale_total = speed_scale_mode * speed_scale_curve * speed_scale_steer;

    if ((abs_preview >= HARD_BRAKE_PREVIEW_TH) && (preview_delta >= HARD_BRAKE_DPREVIEW_TH))
    {
        hard_brake_cnt = HARD_BRAKE_HOLD_CYCLES;
    }
    if (hard_brake_cnt > 0)
    {
        speed_scale_total *= HARD_BRAKE_KEEP;
        hard_brake_cnt--;
    }

    speed_scale_total = clamp_f(speed_scale_total, SPEED_SCALE_MIN, SPEED_SCALE_MAX);

    target_pulses_des = (int)((float)base_pulses * speed_scale_total);
    target_pulses_min = (int)((float)base_pulses * SPEED_SCALE_MIN);
    target_pulses_max = (int)((float)base_pulses * SPEED_SCALE_MAX);
    if (target_pulses_des < target_pulses_min) target_pulses_des = target_pulses_min;
    if (target_pulses_des > target_pulses_max) target_pulses_des = target_pulses_max;

    if (target_pulses_ramped <= 1.0f)
    {
        target_pulses_ramped = (float)target_pulses_des;
    }

    target_step_up = TARGET_STEP_UP_STRAIGHT + (TARGET_STEP_UP_SHARP - TARGET_STEP_UP_STRAIGHT) * mode_intensity;

    if ((float)target_pulses_des > target_pulses_ramped + target_step_up)
    {
        target_pulses_ramped += target_step_up;
    }
    else if ((float)target_pulses_des < target_pulses_ramped - TARGET_STEP_DOWN)
    {
        target_pulses_ramped -= TARGET_STEP_DOWN;
    }
    else
    {
        target_pulses_ramped = (float)target_pulses_des;
    }

    target_pulses = (int)(target_pulses_ramped + 0.5f);
    if (target_pulses < target_pulses_min) target_pulses = target_pulses_min;
    if (target_pulses > target_pulses_max) target_pulses = target_pulses_max;

    // 8) 后轮差速
    diff_k = DIFF_K_BASE + DIFF_K_GAIN * mode_intensity;
    steer_for_diff = steer_out;
    if (abs_steer <= DIFF_STEER_DEADBAND)
    {
        steer_for_diff = 0.0f;
        diff_k *= 0.25f;
    }

    diff_cmd = steer_for_diff * diff_k;
    if (steer_for_diff > 0.0f) diff_cmd *= RIGHT_TURN_DIFF_MUL;
    else if (steer_for_diff < 0.0f) diff_cmd *= LEFT_TURN_DIFF_MUL;

    diff_filtered = diff_filtered * (1.0f - DIFF_FILTER_ALPHA) + diff_cmd * DIFF_FILTER_ALPHA;

    diff_step_limit = DIFF_STEP_STRAIGHT + (DIFF_STEP_SHARP - DIFF_STEP_STRAIGHT) * mode_intensity;
    if (steer_for_diff > 0.0f) diff_step_limit *= DIFF_STEP_RIGHT_MUL;

    diff_delta = diff_filtered - diff_limited;
    if (diff_delta > diff_step_limit) diff_limited += diff_step_limit;
    else if (diff_delta < -diff_step_limit) diff_limited -= diff_step_limit;
    else diff_limited = diff_filtered;

    diff_speed = (int)diff_limited;

    diff_cap = (int)((float)target_pulses * DIFF_CAP_RATIO);
    if (diff_cap < DIFF_CAP_MIN) diff_cap = DIFF_CAP_MIN;
    if (diff_speed > diff_cap) diff_speed = diff_cap;
    else if (diff_speed < -diff_cap) diff_speed = -diff_cap;

    left_target_pulses = target_pulses + diff_speed;
    right_target_pulses = target_pulses - diff_speed;

    // 急弯内轮最低速度保护（滞回）
    if (!inner_min_enable)
    {
        if ((abs_steer >= DIFF_INNER_MIN_STEER_ON) &&
            (curve_score >= DIFF_INNER_MIN_SCORE_ON))
        {
            inner_min_enable = 1;
        }
    }
    else
    {
        if ((abs_steer <= DIFF_INNER_MIN_STEER_OFF) ||
            (curve_score <= DIFF_INNER_MIN_SCORE_OFF))
        {
            inner_min_enable = 0;
        }
    }

    if (inner_min_enable)
    {
        inner_min_pulses = (int)((float)base_pulses * DIFF_INNER_MIN_RATIO);
        if (inner_min_pulses < 1) inner_min_pulses = 1;

        if (left_target_pulses < right_target_pulses)
        {
            if (left_target_pulses < inner_min_pulses) left_target_pulses = inner_min_pulses;
        }
        else
        {
            if (right_target_pulses < inner_min_pulses) right_target_pulses = inner_min_pulses;
        }
    }

    // 9) 电机闭环
    encoder_update();

    left_motor_speed_pid_calc(left_target_pulses, left_speed);
    right_motor_speed_pid_calc(right_target_pulses, right_speed);

    set_motor_speed((int)left_motor_speedpid.output, (int)right_motor_speedpid.output);

    // 保留低频调试触发（不做在线调参）
    loop_cnt++;
    if (loop_cnt >= 5)
    {
        loop_cnt = 0;
        print_flag = 1;
    }
}