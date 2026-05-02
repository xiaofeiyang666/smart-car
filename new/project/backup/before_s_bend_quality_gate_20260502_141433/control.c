#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "brushless.h"

// ===================== 速度映射 =====================
#define MAX_SPEED_PULSES                 4000
#define BASE_SPEED_MATCH_GAIN            1.05f   // 让target_speed_base与实车速度更接近

// ===================== 舵机控制参数 =====================
#define SERVO_OUT_LIMIT_LEFT             (SERVO_CENTER - SERVO_L_MAX)
#define SERVO_OUT_LIMIT_RIGHT            (SERVO_R_MAX - SERVO_CENTER)
#define SERVO_DEADBAND_PIX               1.2f

#define ERR_SMALL_TH                     3.5f
#define ERR_MID_TH                       10.0f

#define KP_MUL_SMALL                     0.58f
#define KP_MUL_MID                       0.92f
#define KP_MUL_LARGE                     1.20f

#define KD_MUL_SMALL                     1.26f
#define KD_MUL_MID                       1.00f
#define KD_MUL_LARGE                     0.86f

#define STEP_SMALL                       1.10f
#define STEP_MID                         3.20f
#define STEP_LARGE                       7.60f
#define STEP_MIN                         0.60f

#define STEP_REVERSE_DPV_TH              1.00f
#define STEP_REVERSE_MUL                 1.70f

#define RIGHT_TURN_STEER_MUL             1.18f   // 右转不足就加，右转过激就减
#define LEFT_TURN_STEER_MUL              1.00f

#define OFFSET_LPF_ALPHA_STRAIGHT        0.30f
#define OFFSET_LPF_ALPHA_CURVE           0.62f
#define PREVIEW_LPF_ALPHA                0.68f   // 前瞻滤波：S弯第二个弯晚可适当增大，直线摆动则减小
#define GYRO_LPF_ALPHA                   0.30f
#define ERRD_LPF_ALPHA                   0.40f
#define STEER_RAW_ALPHA_STRAIGHT         0.22f
#define STEER_RAW_ALPHA_CURVE            0.78f

#define STEER_FF_DPREVIEW_K              0.36f   // 前瞻变化微分：S弯反向切换慢就增大
#define STEER_FF_LIMIT_BASE              3.2f   // 前瞻输出限幅：十字接圆弯入弯晚可增大
#define STEER_FF_LIMIT_CURVE_ADD         2.6f
#define STEER_FF_SUPPRESS_ERR_TH         4.2f
#define STEER_FF_SUPPRESS_PREVIEW_TH     3.2f
#define STEER_FF_SUPPRESS_MUL            0.35f

// 双PD：横向PD负责“贴中线”，前瞻PD负责“提前转向”
#define DUAL_PD_LAT_D_BASE               0.32f  // 横向D增益基数（抑制冲过头）
#define DUAL_PD_LAT_D_CURVE_ADD          0.30f  // 弯道时横向D增强
#define DUAL_PD_HEAD_P_CURVE_ADD         0.62f  // 弯道前瞻P增强，太贴内线/直线晃则减小  // 弯道时前瞻P增强
#define DUAL_PD_HEAD_D_GAIN              1.12f  // S弯换向响应，过冲摆动则减小  // 前瞻D权重（控制S弯切换速度）
#define DUAL_PD_GYRO_D_CURVE_ADD         0.18f  // 弯道时陀螺阻尼增强
#define DUAL_PD_EXIT_HEAD_KEEP           0.75f  // 出弯时降低前瞻环，防止回直线抖动

#define SMALL_OSC_DERR_TH                2.2f   // 小误差高频摆动判定阈值
#define SMALL_OSC_KP_MUL                 0.80f
#define SMALL_OSC_KD_MUL                 1.22f
#define SMALL_OSC_STEP_MUL               0.68f

// 外漂判定：偏差方向和弯向一致时，通常是车身沿外线漂移
#define OUTSIDE_PREVIEW_TH               3.4f
#define OUTSIDE_ERR_TH                   2.4f
#define OUTSIDE_FF_SUPPRESS              0.52f
#define OUTSIDE_PULL_K                   0.24f
#define OUTSIDE_STEP_MUL                 1.18f
#define OUTSIDE_SPEED_KEEP               0.88f
#define OUTSIDE_HOLD_CYCLES              6

// S弯换向辅助：off还在上一弯外侧、pre已看到下一弯时，让前瞻先带头。
// 第二个弯仍晚：增大 S_BEND_HEAD_BOOST / S_BEND_STEP_MUL；直线或出弯摆动则减小。
#define S_BEND_PREVIEW_TH                12.0f
#define S_BEND_OFFSET_TH                 6.0f
#define S_BEND_LAT_MUL                   0.62f
#define S_BEND_HEAD_BOOST                1.36f
#define S_BEND_STEP_MUL                  1.34f

// ===================== 速度调度参数 =====================
#define CURVE_SCORE_PREVIEW_K            0.58f
#define CURVE_SCORE_DPV_K                0.82f
#define CURVE_SCORE_OFFSET_K             0.12f

#define SPEED_CURVE_K                    1.58f
#define SPEED_STEER_K                    0.64f
#define SPEED_CURVE_MIN                  0.34f
#define SPEED_STEER_MIN                  0.38f
#define SPEED_SCALE_MIN                  0.24f
#define SPEED_SCALE_MAX                  1.04f
#define SPEED_CONF_MIN                   0.82f

#define ENTRY_PREVIEW_TH                 2.2f
#define ENTRY_DPREVIEW_TH                0.08f
#define ENTRY_BRAKE_KEEP                 0.84f

#define TRANS_BRAKE_PREVIEW_TH           2.8f
#define TRANS_BRAKE_DPV_TH               0.14f
#define TRANS_BRAKE_KEEP                 0.78f

#define HARD_BRAKE_PREVIEW_TH            4.8f
#define HARD_BRAKE_DPV_TH                0.30f
#define HARD_BRAKE_KEEP                  0.42f
#define HARD_BRAKE_HOLD_CYCLES           16

#define EXIT_PREVIEW_TH                  2.8f
#define EXIT_DPREVIEW_TH                 0.12f
#define EXIT_STEER_TH                    5.8f
#define EXIT_RELEASE_BOOST               1.03f

#define TARGET_STEP_UP_STRAIGHT          20.0f
#define TARGET_STEP_UP_CURVE_MIN         6.0f
#define TARGET_STEP_DOWN                 360.0f

// ===================== 后轮差速参数 =====================
#define DIFF_STEER_DEADBAND              4.0f
#define DIFF_GAIN_BASE                   0.86f
#define DIFF_GAIN_CURVE_ADD              0.30f
#define DIFF_RIGHT_MUL                   1.16f
#define DIFF_LEFT_MUL                    1.00f
#define DIFF_OUTSIDE_MUL                 1.16f

#define DIFF_FILTER_ALPHA                0.22f
#define DIFF_STEP_BASE                   12.0f
#define DIFF_STEP_CURVE_ADD              8.0f

#define DIFF_CAP_RATIO_BASE              0.16f
#define DIFF_CAP_RATIO_CURVE_ADD         0.08f
#define DIFF_CAP_MIN                     14

#define DIFF_INNER_MIN_STEER_TH          12.0f
#define DIFF_INNER_MIN_CURVE_TH          0.45f
#define DIFF_INNER_MIN_RATIO             0.18f

volatile uint8 print_flag = 0;

// 基础速度（0~90）
int target_speed_base = 28;

// 舵机主参数：提速后优先调这三个
float servo_kp = 0.35f;
float servo_kd = 0.11f;
float servo_kff = 0.18f;

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

// Ackermann差速查表（3~20度）
static float diff_gamma_from_steer(float abs_steer)
{
    static const float gamma_lut[21] = {
        0.0000f, 0.0000f, 0.0000f, 0.0382f, 0.0512f, 0.0643f, 0.0774f,
        0.0913f, 0.1053f, 0.1201f, 0.1346f, 0.1498f, 0.1669f, 0.1835f,
        0.2017f, 0.2198f, 0.2442f, 0.2684f, 0.2928f, 0.3262f, 0.3550f
    };
    int idx_low;
    int idx_high;
    float frac;

    if (abs_steer <= 0.0f) return 0.0f;
    if (abs_steer <= 3.0f) return gamma_lut[3] * (abs_steer / 3.0f);
    if (abs_steer >= 20.0f) return gamma_lut[20];

    idx_low = (int)abs_steer;
    if (idx_low < 3) idx_low = 3;
    if (idx_low >= 20) idx_low = 19;
    idx_high = idx_low + 1;
    frac = abs_steer - (float)idx_low;

    return gamma_lut[idx_low] + (gamma_lut[idx_high] - gamma_lut[idx_low]) * frac;
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
    static uint8 outside_hold_cnt = 0;

    static float offset_filtered = 0.0f;
    static float preview_filtered = 0.0f;
    static float preview_last = 0.0f;
    static float gyro_z_filtered = 0.0f;
    static float err_last = 0.0f;
    static float errd_filtered = 0.0f;
    static float steer_raw_filtered = 0.0f;
    static float steer_out_last = 0.0f;
    static float target_pulses_ramped = 0.0f;
    static float diff_filtered = 0.0f;
    static float diff_limited = 0.0f;

    int offset_raw;
    int preview_raw_i;
    float preview_raw;

    float offset_alpha;
    float offset_ctrl;
    float abs_err;
    float errd_raw;
    float errd_abs;

    float preview_delta;
    float preview_delta_abs;
    float abs_preview;

    float curve_score;
    float curve_intensity;

    float kp_mul;
    float kd_mul;
    float step_base;
    float blend_t;

    float kp_now;
    float kd_now;
    float lat_d_now;
    float head_kp_now;
    float head_kd_now;
    float lat_term;
    float head_term;
    float ff_limit;

    float steer_raw;
    float steer_alpha;
    float steer_step_limit;
    float delta_out;
    float steer_out;
    float abs_steer;

    int turn_dir;
    uint8 outside_drift;
    uint8 outside_raw;
    uint8 s_bend_switch;
    uint8 entry_phase;
    uint8 exit_phase;

    float conf_scale;
    float speed_scale_curve;
    float speed_scale_steer;
    float speed_scale_conf;
    float speed_scale_total;
    float steer_norm;
    float steer_limit_now;

    int base_pulses;
    int target_pulses_des;
    int target_pulses_min;
    int target_pulses_max;
    int target_pulses;
    float target_step_up;

    float diff_gamma;
    float diff_gain;
    float diff_cmd;
    float diff_step_limit;
    float diff_delta;
    int diff_speed;
    int diff_cap;
    float diff_cap_ratio;

    int left_target_pulses;
    int right_target_pulses;
    int inner_min_pulses;

    offset_raw = (int)camera_bias_raw;
    if (offset_raw > 90) offset_raw = 90;
    if (offset_raw < -90) offset_raw = -90;

    preview_raw_i = (int)camera_preview_raw;
    if (preview_raw_i > 90) preview_raw_i = 90;
    if (preview_raw_i < -90) preview_raw_i = -90;
    preview_raw = (float)preview_raw_i;

    if (!lpf_init)
    {
        offset_filtered = (float)offset_raw;
        preview_filtered = preview_raw;
        preview_last = preview_raw;
        gyro_z_filtered = 0.0f;
        err_last = 0.0f;
        errd_filtered = 0.0f;
        steer_raw_filtered = 0.0f;
        steer_out_last = 0.0f;
        target_pulses_ramped = 0.0f;
        diff_filtered = 0.0f;
        diff_limited = 0.0f;
        hard_brake_cnt = 0;
        outside_hold_cnt = 0;
        lpf_init = 1;
    }

    abs_preview = abs_f(preview_filtered);
    if (abs_preview >= 3.0f)
        offset_alpha = OFFSET_LPF_ALPHA_CURVE;
    else
        offset_alpha = OFFSET_LPF_ALPHA_STRAIGHT;

    offset_filtered = offset_filtered * (1.0f - offset_alpha) + (float)offset_raw * offset_alpha;
    preview_filtered = preview_filtered * (1.0f - PREVIEW_LPF_ALPHA) + preview_raw * PREVIEW_LPF_ALPHA;

    preview_delta = preview_filtered - preview_last;
    preview_last = preview_filtered;
    preview_delta_abs = abs_f(preview_delta);

    offset_ctrl = offset_filtered;
    if (offset_ctrl <= SERVO_DEADBAND_PIX && offset_ctrl >= -SERVO_DEADBAND_PIX)
    {
        offset_ctrl = 0.0f;
    }
    else if (offset_ctrl > 0.0f)
    {
        offset_ctrl -= SERVO_DEADBAND_PIX;
    }
    else
    {
        offset_ctrl += SERVO_DEADBAND_PIX;
    }

    imu660ra_get_gyro();
    gyro_z_filtered = gyro_z_filtered * (1.0f - GYRO_LPF_ALPHA) + imu660ra_gyro_transition(imu660ra_gyro_z) * GYRO_LPF_ALPHA;

    abs_err = abs_f(offset_ctrl);
    errd_raw = offset_ctrl - err_last;
    err_last = offset_ctrl;
    errd_filtered = errd_filtered * (1.0f - ERRD_LPF_ALPHA) + errd_raw * ERRD_LPF_ALPHA;
    errd_abs = abs_f(errd_filtered);

    curve_score = CURVE_SCORE_PREVIEW_K * abs_f(preview_filtered)
                + CURVE_SCORE_DPV_K * preview_delta_abs
                + CURVE_SCORE_OFFSET_K * abs_err;
    curve_intensity = curve_score / 12.0f;
    curve_intensity = clamp_f(curve_intensity, 0.0f, 1.0f);

    turn_dir = (preview_filtered >= 0.0f) ? 1 : -1;
    outside_raw = 0;
    if ((abs_f(preview_filtered) >= OUTSIDE_PREVIEW_TH) &&
        (abs_err >= OUTSIDE_ERR_TH) &&
        (((offset_ctrl > 0.0f) && (turn_dir > 0)) || ((offset_ctrl < 0.0f) && (turn_dir < 0))))
    {
        outside_raw = 1;
    }

    if (outside_raw)
    {
        outside_hold_cnt = OUTSIDE_HOLD_CYCLES;
    }
    else if (outside_hold_cnt > 0)
    {
        outside_hold_cnt--;
    }
    outside_drift = (outside_hold_cnt > 0) ? 1 : 0;

    s_bend_switch = 0;
    if ((abs_f(preview_filtered) >= S_BEND_PREVIEW_TH) &&
        (abs_err >= S_BEND_OFFSET_TH) &&
        (((offset_ctrl > 0.0f) && (preview_filtered < 0.0f)) || ((offset_ctrl < 0.0f) && (preview_filtered > 0.0f))))
    {
        s_bend_switch = 1;
    }

    entry_phase = 0;
    if ((abs_f(preview_filtered) >= ENTRY_PREVIEW_TH) && (preview_delta_abs >= ENTRY_DPREVIEW_TH))
    {
        entry_phase = 1;
    }

    exit_phase = 0;
    if ((abs_f(preview_filtered) <= EXIT_PREVIEW_TH) &&
        (preview_delta_abs <= EXIT_DPREVIEW_TH) &&
        (abs_f(steer_out_last) <= EXIT_STEER_TH))
    {
        exit_phase = 1;
    }

    // 分段增益：直线稳、弯道快
    if (abs_err <= ERR_SMALL_TH)
    {
        kp_mul = KP_MUL_SMALL;
        kd_mul = KD_MUL_SMALL;
        step_base = STEP_SMALL;
    }
    else if (abs_err <= ERR_MID_TH)
    {
        blend_t = (abs_err - ERR_SMALL_TH) / (ERR_MID_TH - ERR_SMALL_TH);
        kp_mul = KP_MUL_SMALL + (KP_MUL_MID - KP_MUL_SMALL) * blend_t;
        kd_mul = KD_MUL_SMALL + (KD_MUL_MID - KD_MUL_SMALL) * blend_t;
        step_base = STEP_SMALL + (STEP_MID - STEP_SMALL) * blend_t;
    }
    else
    {
        kp_mul = KP_MUL_LARGE;
        kd_mul = KD_MUL_LARGE;
        step_base = STEP_LARGE;
    }

    // 直线小误差高频摆动抑制
    if ((abs_err <= ERR_SMALL_TH) && (errd_abs >= SMALL_OSC_DERR_TH))
    {
        kp_mul *= SMALL_OSC_KP_MUL;
        kd_mul *= SMALL_OSC_KD_MUL;
        step_base *= SMALL_OSC_STEP_MUL;
    }

    // 双PD融合：横向PD + 前瞻PD + 陀螺阻尼
    kp_now = servo_kp * (1.0f + 0.55f * curve_intensity) * kp_mul;
    kd_now = servo_kd * (1.0f + DUAL_PD_GYRO_D_CURVE_ADD * curve_intensity) * kd_mul;
    lat_d_now = (servo_kp * DUAL_PD_LAT_D_BASE) * (1.0f + DUAL_PD_LAT_D_CURVE_ADD * curve_intensity) * kd_mul;

    head_kp_now = servo_kff * (1.0f + DUAL_PD_HEAD_P_CURVE_ADD * curve_intensity);
    head_kd_now = head_kp_now * STEER_FF_DPREVIEW_K * DUAL_PD_HEAD_D_GAIN;

    lat_term = (-kp_now * offset_ctrl) - (lat_d_now * errd_filtered);
    head_term = (head_kp_now * preview_filtered) + (head_kd_now * preview_delta);

    if (s_bend_switch)
    {
        // S弯第二个弯：近处off仍带着上一弯残差，先让远端pre决定换向。
        lat_term *= S_BEND_LAT_MUL;
        head_term *= S_BEND_HEAD_BOOST;
    }

    ff_limit = STEER_FF_LIMIT_BASE + STEER_FF_LIMIT_CURVE_ADD * curve_intensity;
    if ((abs_err <= STEER_FF_SUPPRESS_ERR_TH) && (abs_f(preview_filtered) <= STEER_FF_SUPPRESS_PREVIEW_TH))
    {
        head_term *= STEER_FF_SUPPRESS_MUL;
    }

    if (outside_drift)
    {
        head_term *= OUTSIDE_FF_SUPPRESS;
    }

    if (exit_phase)
    {
        head_term *= DUAL_PD_EXIT_HEAD_KEEP;
    }

    head_term = clamp_f(head_term, -ff_limit, ff_limit);

    steer_raw = lat_term + head_term - (kd_now * gyro_z_filtered);

    if (outside_drift)
    {
        steer_raw += (-OUTSIDE_PULL_K * offset_ctrl);
    }

    if (steer_raw >= 0.0f)
        steer_raw = steer_raw * RIGHT_TURN_STEER_MUL;

    steer_alpha = STEER_RAW_ALPHA_STRAIGHT + (STEER_RAW_ALPHA_CURVE - STEER_RAW_ALPHA_STRAIGHT) * curve_intensity;
    if (outside_drift)
    {
        steer_alpha += 0.08f;
        if (steer_alpha > 0.85f) steer_alpha = 0.85f;
    }

    steer_raw_filtered = steer_raw_filtered * (1.0f - steer_alpha) + steer_raw * steer_alpha;
    steer_raw = clamp_steer_f(steer_raw_filtered);

    steer_step_limit = step_base * (1.0f + 0.40f * curve_intensity);
    if (entry_phase)
    {
        steer_step_limit *= 1.25f;
    }
    if (s_bend_switch)
    {
        steer_step_limit *= S_BEND_STEP_MUL;
    }
    if (outside_drift)
    {
        steer_step_limit *= OUTSIDE_STEP_MUL;
    }

    if ((preview_delta_abs >= STEP_REVERSE_DPV_TH) &&
        ((steer_out_last > 0.0f && steer_raw < 0.0f) || (steer_out_last < 0.0f && steer_raw > 0.0f)))
    {
        steer_step_limit *= STEP_REVERSE_MUL;
    }

    if (steer_step_limit < STEP_MIN) steer_step_limit = STEP_MIN;

    delta_out = steer_raw - steer_out_last;
    if (delta_out > steer_step_limit)
        steer_out = steer_out_last + steer_step_limit;
    else if (delta_out < -steer_step_limit)
        steer_out = steer_out_last - steer_step_limit;
    else
        steer_out = steer_raw;

    steer_out = clamp_steer_f(steer_out);
    steer_out_last = steer_out;
    abs_steer = abs_f(steer_out);

    servo_set_angle(SERVO_CENTER + steer_out);

    conf_scale = (float)camera_confidence / 100.0f;
    conf_scale = clamp_f(conf_scale, 0.0f, 1.0f);

    if (conf_scale < 0.55f)
    {
        curve_intensity = clamp_f(curve_intensity + 0.18f, 0.0f, 1.0f);
    }

    speed_scale_curve = 1.0f / (1.0f + SPEED_CURVE_K * curve_intensity * curve_intensity);
    if (speed_scale_curve < SPEED_CURVE_MIN) speed_scale_curve = SPEED_CURVE_MIN;

    steer_limit_now = steer_limit_for_side(steer_out);
    if (steer_limit_now < 1.0f) steer_limit_now = 1.0f;
    steer_norm = abs_steer / steer_limit_now;
    if (steer_norm > 1.0f) steer_norm = 1.0f;

    speed_scale_steer = 1.0f - SPEED_STEER_K * steer_norm * steer_norm;
    if (speed_scale_steer < SPEED_STEER_MIN) speed_scale_steer = SPEED_STEER_MIN;

    speed_scale_conf = SPEED_CONF_MIN + (1.0f - SPEED_CONF_MIN) * conf_scale;
    speed_scale_total = speed_scale_curve * speed_scale_steer * speed_scale_conf;

    if (entry_phase)
    {
        speed_scale_total *= ENTRY_BRAKE_KEEP;
    }

    if ((abs_f(preview_filtered) >= TRANS_BRAKE_PREVIEW_TH) && (preview_delta_abs >= TRANS_BRAKE_DPV_TH))
    {
        speed_scale_total *= TRANS_BRAKE_KEEP;
    }

    if ((abs_f(preview_filtered) >= HARD_BRAKE_PREVIEW_TH) && (preview_delta_abs >= HARD_BRAKE_DPV_TH))
    {
        hard_brake_cnt = HARD_BRAKE_HOLD_CYCLES;
    }

    if (hard_brake_cnt > 0)
    {
        speed_scale_total *= HARD_BRAKE_KEEP;
        hard_brake_cnt--;
    }

    if (outside_drift)
    {
        speed_scale_total *= OUTSIDE_SPEED_KEEP;
    }

    if (exit_phase && !outside_drift)
    {
        speed_scale_total *= EXIT_RELEASE_BOOST;
    }

    speed_scale_total = clamp_f(speed_scale_total, SPEED_SCALE_MIN, SPEED_SCALE_MAX);

    base_pulses = (int)((float)(((long)target_speed_base * MAX_SPEED_PULSES) / 90) * BASE_SPEED_MATCH_GAIN);

    target_pulses_des = (int)((float)base_pulses * speed_scale_total);
    target_pulses_min = (int)((float)base_pulses * SPEED_SCALE_MIN);
    target_pulses_max = (int)((float)base_pulses * SPEED_SCALE_MAX);

    if (target_pulses_des < target_pulses_min) target_pulses_des = target_pulses_min;
    if (target_pulses_des > target_pulses_max) target_pulses_des = target_pulses_max;

    if (target_pulses_ramped <= 1.0f)
        target_pulses_ramped = (float)target_pulses_des;

    target_step_up = TARGET_STEP_UP_STRAIGHT - (TARGET_STEP_UP_STRAIGHT - TARGET_STEP_UP_CURVE_MIN) * curve_intensity;
    if (target_step_up < TARGET_STEP_UP_CURVE_MIN) target_step_up = TARGET_STEP_UP_CURVE_MIN;

    if ((float)target_pulses_des > target_pulses_ramped + target_step_up)
        target_pulses_ramped += target_step_up;
    else if ((float)target_pulses_des < target_pulses_ramped - TARGET_STEP_DOWN)
        target_pulses_ramped -= TARGET_STEP_DOWN;
    else
        target_pulses_ramped = (float)target_pulses_des;

    target_pulses = (int)(target_pulses_ramped + 0.5f);
    if (target_pulses < target_pulses_min) target_pulses = target_pulses_min;
    if (target_pulses > target_pulses_max) target_pulses = target_pulses_max;

    if (abs_steer <= DIFF_STEER_DEADBAND)
    {
        diff_cmd = 0.0f;
    }
    else
    {
        diff_gamma = diff_gamma_from_steer(abs_steer);
        diff_gain = DIFF_GAIN_BASE + DIFF_GAIN_CURVE_ADD * curve_intensity;
        diff_cmd = (float)target_pulses * diff_gamma * diff_gain;

        if (steer_out > 0.0f)
            diff_cmd *= DIFF_RIGHT_MUL;
        else if (steer_out < 0.0f)
            diff_cmd = -diff_cmd * DIFF_LEFT_MUL;
    }

    if (outside_drift)
    {
        diff_cmd *= DIFF_OUTSIDE_MUL;
    }

    if (conf_scale < 0.55f)
    {
        diff_cmd *= 0.72f;
    }

    diff_filtered = diff_filtered * (1.0f - DIFF_FILTER_ALPHA) + diff_cmd * DIFF_FILTER_ALPHA;
    diff_step_limit = DIFF_STEP_BASE + DIFF_STEP_CURVE_ADD * curve_intensity;

    diff_delta = diff_filtered - diff_limited;
    if (diff_delta > diff_step_limit)
        diff_limited += diff_step_limit;
    else if (diff_delta < -diff_step_limit)
        diff_limited -= diff_step_limit;
    else
        diff_limited = diff_filtered;

    if (diff_limited >= 0.0f)
        diff_speed = (int)(diff_limited + 0.5f);
    else
        diff_speed = (int)(diff_limited - 0.5f);

    diff_cap_ratio = DIFF_CAP_RATIO_BASE + DIFF_CAP_RATIO_CURVE_ADD * curve_intensity;
    diff_cap = (int)((float)target_pulses * diff_cap_ratio);
    if (diff_cap < DIFF_CAP_MIN) diff_cap = DIFF_CAP_MIN;

    if (diff_speed > diff_cap) diff_speed = diff_cap;
    if (diff_speed < -diff_cap) diff_speed = -diff_cap;

    left_target_pulses = target_pulses + diff_speed;
    right_target_pulses = target_pulses - diff_speed;

    if ((abs_steer >= DIFF_INNER_MIN_STEER_TH) && (curve_intensity >= DIFF_INNER_MIN_CURVE_TH))
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





