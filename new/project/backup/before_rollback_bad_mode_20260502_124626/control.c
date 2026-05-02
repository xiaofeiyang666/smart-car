#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "brushless.h"

// ===================== 速度映射 =====================
#define MAX_SPEED_PULSES                 4000
#define BASE_SPEED_MATCH_GAIN            1.05f

// ===================== 舵机基础参数 =====================
#define SERVO_OUT_LIMIT_LEFT             (SERVO_CENTER - SERVO_L_MAX)
#define SERVO_OUT_LIMIT_RIGHT            (SERVO_R_MAX - SERVO_CENTER)
#define SERVO_DEADBAND_PIX               1.2f
#define RIGHT_TURN_STEER_MUL             1.18f
#define LEFT_TURN_STEER_MUL              1.00f

#define BIAS_LPF_ALPHA_STRAIGHT          0.28f
#define BIAS_LPF_ALPHA_CURVE             0.56f
#define PREVIEW_LPF_ALPHA                0.50f
#define SLOPE_LPF_ALPHA                  0.46f
#define CURVE_LPF_ALPHA                  0.40f
#define GYRO_LPF_ALPHA                   0.30f
#define DBIAS_LPF_ALPHA                  0.38f
#define DPREVIEW_LPF_ALPHA               0.42f

#define HEAD_SLOPE_GAIN                  0.45f
#define HEAD_CURVE_GAIN                  0.18f
#define HEAD_DPREVIEW_GAIN               0.22f
#define LAT_D_BASE                       0.30f
#define GYRO_D_BASE                      1.00f
#define FF_LIMIT_BASE                    2.0f
#define FF_LIMIT_CURVE_ADD               2.0f
#define CONTROL_LOW_CONF_FORCE_Q         40
#define LOW_CONF_RECOVER_BIAS_TH         12.0f
#define LOW_CONF_RECOVER_BIAS_ALPHA      0.52f
#define LOW_CONF_RECOVER_KP_MUL          1.55f
#define LOW_CONF_RECOVER_LAT_D_MUL       0.85f
#define LOW_CONF_RECOVER_HEAD_MUL        0.45f
#define LOW_CONF_RECOVER_GYRO_MUL        0.45f
#define LOW_CONF_RECOVER_ALPHA_MIN       0.62f
#define LOW_CONF_RECOVER_STEP_MIN        5.8f
#define LOW_CONF_RECOVER_SPEED_KEEP      0.78f
#define LOW_CONF_RECOVER_DIFF_MUL        1.10f

#define CURVE_SCORE_PREVIEW_K            0.50f
#define CURVE_SCORE_SLOPE_K              0.30f
#define CURVE_SCORE_CURVE_K              0.18f
#define CURVE_SCORE_DPREVIEW_K           0.62f
#define CURVE_SCORE_OFFSET_K             0.10f

#define SPEED_CURVE_K                    1.42f
#define SPEED_STEER_K                    0.60f
#define SPEED_SCALE_MIN                  0.24f
#define SPEED_SCALE_MAX                  1.04f
#define SPEED_STEER_MIN                  0.40f
#define SPEED_QUALITY_MIN                0.70f

#define TARGET_STEP_UP_STRAIGHT          18.0f
#define TARGET_STEP_UP_CURVE_MIN         5.0f
#define TARGET_STEP_DOWN                 360.0f

// ===================== 后轮差速参数 =====================
#define DIFF_STEER_DEADBAND              4.0f
#define DIFF_GAIN_BASE                   0.78f
#define DIFF_GAIN_CURVE_ADD              0.28f
#define DIFF_RIGHT_MUL                   1.14f
#define DIFF_LEFT_MUL                    1.00f
#define DIFF_FILTER_ALPHA                0.22f
#define DIFF_STEP_BASE                   10.0f
#define DIFF_STEP_CURVE_ADD              8.0f
#define DIFF_CAP_RATIO_BASE              0.15f
#define DIFF_CAP_RATIO_CURVE_ADD         0.08f
#define DIFF_CAP_MIN                     12
#define DIFF_INNER_MIN_STEER_TH          12.0f
#define DIFF_INNER_MIN_CURVE_TH          0.45f
#define DIFF_INNER_MIN_RATIO             0.18f

#define MODE_PARAM_COUNT                 5
#define CONTROL_DEBUG_MODE_LOW_CONF_RECOVER 5

typedef struct
{
    float kp_mul;
    float kd_mul;
    float head_mul;
    float gyro_mul;
    float steer_alpha;
    float step_limit;
    float speed_keep;
    float diff_mul;
} control_mode_param_t;

static const control_mode_param_t control_mode_params[MODE_PARAM_COUNT] =
{
    // STRAIGHT：压前瞻、加阻尼，解决回直线来回摆。
    {0.72f, 1.34f, 0.34f, 1.12f, 0.24f, 1.20f, 1.03f, 0.55f},
    // SMALL_CURVE：小弯近似直线走，少打方向。
    {0.86f, 1.16f, 0.72f, 1.08f, 0.42f, 2.40f, 1.00f, 0.78f},
    // CURVE：普通弯道提前转向，但不再多层条件叠加。
    {1.10f, 1.02f, 1.08f, 1.16f, 0.66f, 5.80f, 0.82f, 1.00f},
    // S_TRANS：连续反向弯，允许更快换向并降速。
    {1.02f, 1.08f, 1.34f, 1.22f, 0.72f, 6.60f, 0.72f, 1.02f},
    // LOW_CONF：小偏差保守，大偏差由救车分支放开近处偏差闭环。
    {0.86f, 1.35f, 0.24f, 0.70f, 0.38f, 3.20f, 0.56f, 0.70f}
};

volatile uint8 print_flag = 0;
volatile int16 control_debug_steer_out = 0;
volatile int16 control_debug_curve_x100 = 0;
volatile int16 control_debug_gyro_z = 0;
volatile uint8 control_debug_mode = CAMERA_TRACK_STRAIGHT;

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
    static float bias_filtered = 0.0f;
    static float preview_filtered = 0.0f;
    static float slope_filtered = 0.0f;
    static float curve_filtered = 0.0f;
    static float gyro_z_filtered = 0.0f;
    static float bias_last = 0.0f;
    static float preview_last = 0.0f;
    static float dbias_filtered = 0.0f;
    static float dpreview_filtered = 0.0f;
    static float steer_raw_filtered = 0.0f;
    static float steer_out_last = 0.0f;
    static float target_pulses_ramped = 0.0f;
    static float diff_filtered = 0.0f;
    static float diff_limited = 0.0f;

    int bias_raw_i;
    int preview_raw_i;
    int slope_raw_i;
    int curve_raw_i;
    uint8 mode;
    uint8 low_conf_recover_raw;
    uint8 low_conf_recover;
    const control_mode_param_t *param;

    float bias_raw;
    float preview_raw;
    float slope_raw;
    float curve_raw;
    float quality_scale;
    float bias_alpha;
    float bias_ctrl;
    float abs_bias;
    float dbias_raw;
    float dpreview_raw;
    float abs_preview;
    float abs_slope;
    float abs_curve;
    float curve_score;
    float curve_intensity;

    float kp_now;
    float kd_now;
    float lat_d_now;
    float gyro_d_now;
    float head_kp_now;
    float lat_term;
    float head_term;
    float ff_limit;
    float steer_raw;
    float steer_alpha;
    float steer_step_limit;
    float delta_out;
    float steer_out;
    float abs_steer;

    float speed_scale_curve;
    float speed_scale_steer;
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

    bias_raw_i = (int)camera_bias_raw;
    preview_raw_i = (int)camera_preview_raw;
    slope_raw_i = (int)camera_slope_raw;
    curve_raw_i = (int)camera_curve_raw;
    if (bias_raw_i > 90) bias_raw_i = 90;
    if (bias_raw_i < -90) bias_raw_i = -90;
    if (preview_raw_i > 90) preview_raw_i = 90;
    if (preview_raw_i < -90) preview_raw_i = -90;
    if (slope_raw_i > 90) slope_raw_i = 90;
    if (slope_raw_i < -90) slope_raw_i = -90;
    if (curve_raw_i > 90) curve_raw_i = 90;
    if (curve_raw_i < -90) curve_raw_i = -90;

    bias_raw = (float)bias_raw_i;
    preview_raw = (float)preview_raw_i;
    slope_raw = (float)slope_raw_i;
    curve_raw = (float)curve_raw_i;

    mode = camera_track_mode;
    if (mode >= MODE_PARAM_COUNT) mode = CAMERA_TRACK_LOW_CONF;
    if (camera_quality < CONTROL_LOW_CONF_FORCE_Q) mode = CAMERA_TRACK_LOW_CONF;
    param = &control_mode_params[mode];
    low_conf_recover_raw = ((mode == CAMERA_TRACK_LOW_CONF) && (abs_f(bias_raw) >= LOW_CONF_RECOVER_BIAS_TH)) ? 1 : 0;

    if (!lpf_init)
    {
        bias_filtered = bias_raw;
        preview_filtered = preview_raw;
        slope_filtered = slope_raw;
        curve_filtered = curve_raw;
        bias_last = bias_raw;
        preview_last = preview_raw;
        gyro_z_filtered = 0.0f;
        dbias_filtered = 0.0f;
        dpreview_filtered = 0.0f;
        steer_raw_filtered = 0.0f;
        steer_out_last = 0.0f;
        target_pulses_ramped = 0.0f;
        diff_filtered = 0.0f;
        diff_limited = 0.0f;
        lpf_init = 1;
    }

    if ((mode == CAMERA_TRACK_CURVE) || (mode == CAMERA_TRACK_S_TRANS))
        bias_alpha = BIAS_LPF_ALPHA_CURVE;
    else
        bias_alpha = BIAS_LPF_ALPHA_STRAIGHT;
    if (mode == CAMERA_TRACK_LOW_CONF)
    {
        bias_alpha = low_conf_recover_raw ? LOW_CONF_RECOVER_BIAS_ALPHA : 0.22f;
    }

    bias_filtered = bias_filtered * (1.0f - bias_alpha) + bias_raw * bias_alpha;
    preview_filtered = preview_filtered * (1.0f - PREVIEW_LPF_ALPHA) + preview_raw * PREVIEW_LPF_ALPHA;
    slope_filtered = slope_filtered * (1.0f - SLOPE_LPF_ALPHA) + slope_raw * SLOPE_LPF_ALPHA;
    curve_filtered = curve_filtered * (1.0f - CURVE_LPF_ALPHA) + curve_raw * CURVE_LPF_ALPHA;

    dbias_raw = bias_filtered - bias_last;
    dpreview_raw = preview_filtered - preview_last;
    bias_last = bias_filtered;
    preview_last = preview_filtered;
    dbias_filtered = dbias_filtered * (1.0f - DBIAS_LPF_ALPHA) + dbias_raw * DBIAS_LPF_ALPHA;
    dpreview_filtered = dpreview_filtered * (1.0f - DPREVIEW_LPF_ALPHA) + dpreview_raw * DPREVIEW_LPF_ALPHA;

    bias_ctrl = bias_filtered;
    if (bias_ctrl <= SERVO_DEADBAND_PIX && bias_ctrl >= -SERVO_DEADBAND_PIX)
        bias_ctrl = 0.0f;
    else if (bias_ctrl > 0.0f)
        bias_ctrl -= SERVO_DEADBAND_PIX;
    else
        bias_ctrl += SERVO_DEADBAND_PIX;

    imu660ra_get_gyro();
    gyro_z_filtered = gyro_z_filtered * (1.0f - GYRO_LPF_ALPHA) + imu660ra_gyro_transition(imu660ra_gyro_z) * GYRO_LPF_ALPHA;

    abs_bias = abs_f(bias_ctrl);
    abs_preview = abs_f(preview_filtered);
    abs_slope = abs_f(slope_filtered);
    abs_curve = abs_f(curve_filtered);

    curve_score = CURVE_SCORE_PREVIEW_K * abs_preview
                + CURVE_SCORE_SLOPE_K * abs_slope
                + CURVE_SCORE_CURVE_K * abs_curve
                + CURVE_SCORE_DPREVIEW_K * abs_f(dpreview_filtered)
                + CURVE_SCORE_OFFSET_K * abs_bias;
    curve_intensity = clamp_f(curve_score / 14.0f, 0.0f, 1.0f);
    low_conf_recover = ((mode == CAMERA_TRACK_LOW_CONF) &&
                        ((abs_bias >= LOW_CONF_RECOVER_BIAS_TH) || low_conf_recover_raw)) ? 1 : 0;
    if (low_conf_recover)
    {
        curve_intensity = clamp_f(curve_intensity + 0.20f, 0.0f, 1.0f);
    }

    kp_now = servo_kp * (1.0f + 0.35f * curve_intensity) * param->kp_mul;
    kd_now = servo_kd * param->kd_mul;
    lat_d_now = servo_kp * LAT_D_BASE * (1.0f + 0.25f * curve_intensity) * param->kd_mul;
    gyro_d_now = servo_kd * GYRO_D_BASE * (1.0f + 0.25f * curve_intensity) * param->gyro_mul;
    head_kp_now = servo_kff * (1.0f + 0.30f * curve_intensity) * param->head_mul;
    if (low_conf_recover)
    {
        kp_now *= LOW_CONF_RECOVER_KP_MUL;
        lat_d_now *= LOW_CONF_RECOVER_LAT_D_MUL;
        gyro_d_now *= LOW_CONF_RECOVER_GYRO_MUL;
        head_kp_now *= LOW_CONF_RECOVER_HEAD_MUL;
    }

    lat_term = (-kp_now * bias_ctrl) - (lat_d_now * dbias_filtered);
    head_term = head_kp_now * (preview_filtered + HEAD_SLOPE_GAIN * slope_filtered + HEAD_CURVE_GAIN * curve_filtered)
              + head_kp_now * HEAD_DPREVIEW_GAIN * dpreview_filtered;

    ff_limit = FF_LIMIT_BASE + FF_LIMIT_CURVE_ADD * curve_intensity;
    if (mode == CAMERA_TRACK_STRAIGHT) ff_limit *= 0.65f;
    if (mode == CAMERA_TRACK_LOW_CONF) ff_limit *= 0.55f;
    head_term = clamp_f(head_term, -ff_limit, ff_limit);

    steer_raw = lat_term + head_term - gyro_d_now * gyro_z_filtered;
    if (steer_raw >= 0.0f)
        steer_raw = steer_raw * RIGHT_TURN_STEER_MUL;

    steer_alpha = param->steer_alpha + 0.10f * curve_intensity;
    if (low_conf_recover && (steer_alpha < LOW_CONF_RECOVER_ALPHA_MIN)) steer_alpha = LOW_CONF_RECOVER_ALPHA_MIN;
    else if (mode == CAMERA_TRACK_LOW_CONF && steer_alpha > 0.38f) steer_alpha = 0.38f;
    if (steer_alpha > 0.82f) steer_alpha = 0.82f;
    steer_raw_filtered = steer_raw_filtered * (1.0f - steer_alpha) + steer_raw * steer_alpha;
    steer_raw = clamp_steer_f(steer_raw_filtered);

    steer_step_limit = param->step_limit * (1.0f + 0.35f * curve_intensity);
    if ((mode != CAMERA_TRACK_LOW_CONF) &&
        (abs_f(dpreview_filtered) >= 1.0f) &&
        ((steer_out_last > 0.0f && steer_raw < 0.0f) || (steer_out_last < 0.0f && steer_raw > 0.0f)))
    {
        steer_step_limit *= 1.18f;
    }
    if (low_conf_recover && (steer_step_limit < LOW_CONF_RECOVER_STEP_MIN)) steer_step_limit = LOW_CONF_RECOVER_STEP_MIN;
    if (steer_step_limit < 0.55f) steer_step_limit = 0.55f;

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

    quality_scale = SPEED_QUALITY_MIN + (1.0f - SPEED_QUALITY_MIN) * ((float)camera_quality / 100.0f);
    quality_scale = clamp_f(quality_scale, SPEED_QUALITY_MIN, 1.0f);
    speed_scale_curve = 1.0f / (1.0f + SPEED_CURVE_K * curve_intensity * curve_intensity);

    steer_limit_now = steer_limit_for_side(steer_out);
    if (steer_limit_now < 1.0f) steer_limit_now = 1.0f;
    steer_norm = abs_steer / steer_limit_now;
    if (steer_norm > 1.0f) steer_norm = 1.0f;
    speed_scale_steer = 1.0f - SPEED_STEER_K * steer_norm * steer_norm;
    if (speed_scale_steer < SPEED_STEER_MIN) speed_scale_steer = SPEED_STEER_MIN;

    speed_scale_total = param->speed_keep * speed_scale_curve * speed_scale_steer * quality_scale;
    if (low_conf_recover) speed_scale_total *= LOW_CONF_RECOVER_SPEED_KEEP;
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
        diff_gain = (DIFF_GAIN_BASE + DIFF_GAIN_CURVE_ADD * curve_intensity) * param->diff_mul;
        if (low_conf_recover) diff_gain *= LOW_CONF_RECOVER_DIFF_MUL;
        diff_cmd = (float)target_pulses * diff_gamma * diff_gain;
        if (steer_out > 0.0f)
            diff_cmd *= DIFF_RIGHT_MUL;
        else if (steer_out < 0.0f)
            diff_cmd = -diff_cmd * DIFF_LEFT_MUL;
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
    if (mode == CAMERA_TRACK_LOW_CONF) diff_cap_ratio *= 0.70f;
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

    control_debug_steer_out = (int16)steer_out;
    control_debug_curve_x100 = (int16)(curve_intensity * 100.0f + 0.5f);
    control_debug_gyro_z = (int16)gyro_z_filtered;
    control_debug_mode = low_conf_recover ? CONTROL_DEBUG_MODE_LOW_CONF_RECOVER : mode;

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
