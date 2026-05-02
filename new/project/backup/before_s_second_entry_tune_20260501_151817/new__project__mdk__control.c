#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "brushless.h"

// ===================== 速度映射 =====================
// MAX_SPEED_PULSES：速度闭环最大脉冲标尺；只有编码器标定或电机极限变化时才改。
#define MAX_SPEED_PULSES                 4000
// BASE_SPEED_MATCH_GAIN：实际速度比target_speed_base慢就升，过快就降；每次调0.02~0.05。
#define BASE_SPEED_MATCH_GAIN            1.05f

// ===================== 舵机基础参数 =====================
#define SERVO_OUT_LIMIT_LEFT             (SERVO_CENTER - SERVO_L_MAX)
#define SERVO_OUT_LIMIT_RIGHT            (SERVO_R_MAX - SERVO_CENTER)
// 死区：直道小抖可略升，入弯迟钝或小弯不进可略降。
#define SERVO_DEADBAND_PIX               1.2f

// 误差分段：off小于SMALL走稳态参数，大于MID走大弯参数；小弯摆动大就升SMALL，响应慢就降SMALL。
#define ERR_SMALL_TH                     3.5f
#define ERR_MID_TH                       10.0f

// 横向P倍率：直线/小弯抖动先降SMALL，大弯外漂或入弯慢再升LARGE。
#define KP_MUL_SMALL                     0.58f
#define KP_MUL_MID                       0.92f
#define KP_MUL_LARGE                     1.20f

// 阻尼倍率：回直线左右摆就升SMALL/MID；弯中明显跟不上且舵角被压住就降LARGE。
#define KD_MUL_SMALL                     1.26f
#define KD_MUL_MID                       1.00f
#define KD_MUL_LARGE                     0.86f

// 舵机输出斜率：直道顿挫就降SMALL，S弯第二弯进不去可升MID/LARGE或S_TRANSITION_STEP_MUL。
#define STEP_SMALL                       1.10f
#define STEP_MID                         3.20f
#define STEP_LARGE                       7.60f
#define STEP_MIN                         0.60f

// 换向保护：pre快速反向时放宽舵机步进。S弯换向慢就升MUL；回正大摆就降MUL或升TH。
#define STEP_REVERSE_DPV_TH              1.00f
#define STEP_REVERSE_MUL                 1.42f

// 左右舵不对称补偿：某一方向总是不够就升对应倍率；一侧过冲就降。
#define RIGHT_TURN_STEER_MUL             1.18f
#define LEFT_TURN_STEER_MUL              1.00f

// 滤波：ALPHA越大越快也越抖。回直线抖就降直线项，弯道响应慢就升曲线项。
#define OFFSET_LPF_ALPHA_STRAIGHT        0.30f
#define OFFSET_LPF_ALPHA_CURVE           0.62f
#define PREVIEW_LPF_ALPHA                0.58f
// 陀螺滤波：gz噪声导致舵机抖就降，车身转动阻尼跟不上就升。
#define GYRO_LPF_ALPHA                   0.30f
#define ERRD_LPF_ALPHA                   0.40f
#define STEER_RAW_ALPHA_STRAIGHT         0.22f
#define STEER_RAW_ALPHA_CURVE            0.78f

// 前瞻前馈：pre/far越大越提前转。大弯入弯慢升HEAD/FF，回直线摆动先降FF或增加straight lock。
#define STEER_FF_DPREVIEW_K              0.26f
#define STEER_FF_LIMIT_BASE              2.2f
#define STEER_FF_LIMIT_CURVE_ADD         1.4f
#define STEER_FF_SUPPRESS_ERR_TH         4.2f
#define STEER_FF_SUPPRESS_PREVIEW_TH     3.2f
#define STEER_FF_SUPPRESS_MUL            0.35f

// 双PD：横向PD贴中线，前瞻PD提前入弯，陀螺项提供横摆角速度阻尼。
#define DUAL_PD_LAT_D_BASE               0.32f
#define DUAL_PD_LAT_D_CURVE_ADD          0.30f
#define DUAL_PD_HEAD_P_CURVE_ADD         0.30f
#define DUAL_PD_HEAD_D_GAIN              0.90f
#define DUAL_PD_GYRO_D_CURVE_ADD         0.18f
#define DUAL_PD_EXIT_HEAD_KEEP           0.75f

// far远前瞻：S弯和大弯提前量。第二个弯进不去可升HEAD_GAIN；误把直道当弯就升ACTIVE_TH或降GAIN。
#define FAR_PREVIEW_LPF_ALPHA            0.42f
#define FAR_PREVIEW_ACTIVE_TH            4.0f
#define FAR_PREVIEW_HEAD_GAIN            0.16f

// 回直线锁定：pre/far/off/dpre都很小时压住舵机残余输出。回直线仍摆就放宽TH或增大RAW_DECAY抑制；出弯不进直线就收紧TH。
#define STRAIGHT_LOCK_PREVIEW_TH         2.2f
#define STRAIGHT_LOCK_FAR_TH             3.8f
#define STRAIGHT_LOCK_DPV_TH             0.20f
#define STRAIGHT_LOCK_ERR_TH             4.2f
#define STRAIGHT_LOCK_HOLD_CYCLES        10
#define STRAIGHT_LOCK_KP_MUL             0.72f
#define STRAIGHT_LOCK_KD_MUL             1.32f
#define STRAIGHT_LOCK_HEAD_MUL           0.20f
#define STRAIGHT_LOCK_STEP_MUL           0.60f
#define STRAIGHT_LOCK_RAW_DECAY          0.70f

// 小弯近似直线：只在near/far同向且曲率小的时候生效。小弯仍摆就降HEAD/STEP；小弯贴弯不够就升PREVIEW_MAX或HEAD。
#define SMALL_CURVE_PREVIEW_MIN          2.0f
#define SMALL_CURVE_PREVIEW_MAX          5.8f
#define SMALL_CURVE_FAR_MAX              8.5f
#define SMALL_CURVE_ERR_TH               5.0f
#define SMALL_CURVE_KP_MUL               0.80f
#define SMALL_CURVE_KD_MUL               1.10f
#define SMALL_CURVE_HEAD_MUL             0.62f
#define SMALL_CURVE_STEP_MUL             0.78f
#define SMALL_CURVE_SPEED_KEEP           1.03f

// S弯第二个弯预告：near与far相反或near很小但far已明显时，提前给far方向前馈并降速。
// 第二个弯跑直线就降FAR_TH或升HEAD_GAIN；误触发导致直道摆动就升FAR_TH/CONF_MIN或降HEAD_GAIN。
#define S_TRANSITION_FAR_TH              12.0f
#define S_TRANSITION_PREVIEW_MAX         24.0f
#define S_TRANSITION_CONF_MIN            15
#define S_TRANSITION_STRONG_FAR_TH       28.0f
#define S_TRANSITION_STRONG_PREVIEW_MAX  10.0f
#define S_TRANSITION_HEAD_GAIN           0.24f
#define S_TRANSITION_FF_LIMIT_ADD        0.8f
#define S_TRANSITION_STEP_MUL            1.08f
#define S_TRANSITION_SPEED_KEEP          0.78f

#define CONTROL_RING_NONE                0
#define CONTROL_RING_ENTER               1
#define CONTROL_RING_INSIDE              2
#define CONTROL_RING_EXIT                3
// 环岛控制：环岛进不去先确认camera的ring/rs是否触发；已触发但舵角不够再升FF_LIMIT_ADD或相机RING_PREVIEW_BIAS。
#define RING_FF_LIMIT_ADD                1.0f
#define RING_STEP_MUL                    1.02f
#define RING_SPEED_KEEP_ENTER            0.74f
#define RING_SPEED_KEEP_INSIDE           0.64f
#define RING_SPEED_KEEP_EXIT             0.72f
#define RING_DIFF_MUL                    0.88f
#define RING_EXIT_HEAD_KEEP              0.78f

// 小误差振荡：off很小但st左右跳时生效；仍抖就降KP/STEP或升KD。
#define SMALL_OSC_DERR_TH                2.2f
#define SMALL_OSC_KP_MUL                 0.80f
#define SMALL_OSC_KD_MUL                 1.22f
#define SMALL_OSC_STEP_MUL               0.68f

// 外漂判定：偏差方向和弯向一致时，通常是车身沿外线漂移；大弯冲出外侧就升PULL或降速度，误拉内线就降PULL。
#define OUTSIDE_PREVIEW_TH               3.4f
#define OUTSIDE_ERR_TH                   2.4f
#define OUTSIDE_FF_SUPPRESS              0.52f
#define OUTSIDE_PULL_K                   0.24f
#define OUTSIDE_STEP_MUL                 1.18f
#define OUTSIDE_SPEED_KEEP               0.88f
#define OUTSIDE_HOLD_CYCLES              6

// ===================== 速度调度参数 =====================
// 曲率评分权重：pre/far/dpre/off越大越降速。S弯冲出可升DPV/FAR；直道速度上不去就降这些权重。
#define CURVE_SCORE_PREVIEW_K            0.58f
#define CURVE_SCORE_FAR_K                0.20f
#define CURVE_SCORE_DPV_K                0.82f
#define CURVE_SCORE_OFFSET_K             0.12f

#define SPEED_CURVE_K                    1.58f
#define SPEED_STEER_K                    0.64f
#define SPEED_CURVE_MIN                  0.34f
#define SPEED_STEER_MIN                  0.38f
#define SPEED_SCALE_MIN                  0.24f
#define SPEED_SCALE_MAX                  1.04f
#define SPEED_CONF_MIN                   0.82f

// 入弯/连续弯/急弯刹车：冲弯就降低KEEP或降低TH；弯道太慢就升KEEP。
#define ENTRY_PREVIEW_TH                 2.8f
#define ENTRY_DPREVIEW_TH                0.12f
#define ENTRY_BRAKE_KEEP                 0.84f
#define TRANS_BRAKE_PREVIEW_TH           3.2f
#define TRANS_BRAKE_DPV_TH               0.20f
#define TRANS_BRAKE_KEEP                 0.78f
#define HARD_BRAKE_PREVIEW_TH            4.8f
#define HARD_BRAKE_DPV_TH                0.30f
#define HARD_BRAKE_KEEP                  0.42f
#define HARD_BRAKE_HOLD_CYCLES           16

// 出弯释放：回直线抖就降BOOST或收紧exit条件；出弯速度慢就升BOOST。
#define EXIT_PREVIEW_TH                  2.8f
#define EXIT_DPREVIEW_TH                 0.12f
#define EXIT_STEER_TH                    5.8f
#define EXIT_RELEASE_BOOST               1.03f

// 目标速度斜率：加速太突兀就降STEP_UP；弯后提速慢就升STRAIGHT。
#define TARGET_STEP_UP_STRAIGHT          20.0f
#define TARGET_STEP_UP_CURVE_MIN         6.0f
#define TARGET_STEP_DOWN                 360.0f

// ===================== 后轮差速参数 =====================
// 差速只辅助转向。入弯推头可升GAIN/CAP，弯中内轮拖死或摆动就降GAIN/CAP。
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
volatile int16 control_debug_steer_out = 0;
volatile int16 control_debug_curve_x100 = 0;
volatile int16 control_debug_gyro_z = 0;
volatile uint8 control_debug_straight_lock = 0;
volatile uint8 control_debug_s_transition = 0;

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
    static uint8 straight_lock_cnt = 0;

    static float offset_filtered = 0.0f;
    static float preview_filtered = 0.0f;
    static float preview_far_filtered = 0.0f;
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
    int preview_far_raw_i;
    float preview_raw;
    float preview_far_raw;

    float offset_alpha;
    float offset_ctrl;
    float abs_err;
    float errd_raw;
    float errd_abs;

    float preview_delta;
    float preview_delta_abs;
    float abs_preview;
    float abs_far_preview;
    uint8 same_curve;
    uint8 small_curve_line;
    uint8 s_transition_line;
    uint8 straight_lock_raw;
    uint8 straight_lock;

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
    uint8 entry_phase;
    uint8 exit_phase;
    uint8 ring_active;
    uint8 ring_state_local;
    int ring_dir_local;

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

    preview_far_raw_i = (int)camera_preview_far_raw;
    if (preview_far_raw_i > 90) preview_far_raw_i = 90;
    if (preview_far_raw_i < -90) preview_far_raw_i = -90;
    preview_far_raw = (float)preview_far_raw_i;

    if (!lpf_init)
    {
        offset_filtered = (float)offset_raw;
        preview_filtered = preview_raw;
        preview_far_filtered = preview_far_raw;
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
        straight_lock_cnt = 0;
        lpf_init = 1;
    }

    abs_preview = abs_f(preview_filtered);
    if (abs_preview >= 3.0f)
        offset_alpha = OFFSET_LPF_ALPHA_CURVE;
    else
        offset_alpha = OFFSET_LPF_ALPHA_STRAIGHT;

    offset_filtered = offset_filtered * (1.0f - offset_alpha) + (float)offset_raw * offset_alpha;
    preview_filtered = preview_filtered * (1.0f - PREVIEW_LPF_ALPHA) + preview_raw * PREVIEW_LPF_ALPHA;
    preview_far_filtered = preview_far_filtered * (1.0f - FAR_PREVIEW_LPF_ALPHA) + preview_far_raw * FAR_PREVIEW_LPF_ALPHA;

    preview_delta = preview_filtered - preview_last;
    preview_last = preview_filtered;
    preview_delta_abs = abs_f(preview_delta);
    abs_preview = abs_f(preview_filtered);
    abs_far_preview = abs_f(preview_far_filtered);
    same_curve = (((preview_filtered >= 0.0f) && (preview_far_filtered >= 0.0f)) ||
                  ((preview_filtered <= 0.0f) && (preview_far_filtered <= 0.0f))) ? 1 : 0;

    ring_state_local = camera_ring_state;
    ring_dir_local = (int)camera_ring_dir;
    ring_active = ((ring_state_local != CONTROL_RING_NONE) && (ring_dir_local != 0)) ? 1 : 0;

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

    curve_score = CURVE_SCORE_PREVIEW_K * abs_preview
                + CURVE_SCORE_FAR_K * abs_far_preview
                + CURVE_SCORE_DPV_K * preview_delta_abs
                + CURVE_SCORE_OFFSET_K * abs_err;
    curve_intensity = curve_score / 12.0f;
    curve_intensity = clamp_f(curve_intensity, 0.0f, 1.0f);

    if (abs_preview >= 1.0f)
        turn_dir = (preview_filtered >= 0.0f) ? 1 : -1;
    else
        turn_dir = (preview_far_filtered >= 0.0f) ? 1 : -1;
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

    straight_lock_raw = 0;
    if (!ring_active &&
        (abs_preview <= STRAIGHT_LOCK_PREVIEW_TH) &&
        (abs_far_preview <= STRAIGHT_LOCK_FAR_TH) &&
        (preview_delta_abs <= STRAIGHT_LOCK_DPV_TH) &&
        (abs_err <= STRAIGHT_LOCK_ERR_TH) &&
        (camera_confidence >= 45))
    {
        straight_lock_raw = 1;
    }

    if (straight_lock_raw)
    {
        straight_lock_cnt = STRAIGHT_LOCK_HOLD_CYCLES;
    }
    else if (straight_lock_cnt > 0)
    {
        straight_lock_cnt--;
    }
    straight_lock = (straight_lock_cnt > 0) ? 1 : 0;

    small_curve_line = 0;
    if (!ring_active && !straight_lock && same_curve &&
        (abs_preview >= SMALL_CURVE_PREVIEW_MIN) &&
        (abs_preview <= SMALL_CURVE_PREVIEW_MAX) &&
        (abs_far_preview <= SMALL_CURVE_FAR_MAX) &&
        (abs_err <= SMALL_CURVE_ERR_TH) &&
        (camera_confidence >= 45))
    {
        small_curve_line = 1;
    }

    s_transition_line = 0;
    if (!ring_active && !straight_lock &&
        (abs_far_preview >= S_TRANSITION_FAR_TH) &&
        (abs_preview <= S_TRANSITION_PREVIEW_MAX) &&
        ((((preview_filtered > 1.0f) && (preview_far_filtered < -S_TRANSITION_FAR_TH)) ||
          ((preview_filtered < -1.0f) && (preview_far_filtered > S_TRANSITION_FAR_TH)) ||
          ((abs_preview <= 3.0f) && (abs_far_preview >= S_TRANSITION_FAR_TH)))) &&
        ((camera_confidence >= S_TRANSITION_CONF_MIN) ||
         ((abs_far_preview >= S_TRANSITION_STRONG_FAR_TH) && (abs_preview <= S_TRANSITION_STRONG_PREVIEW_MAX))))
    {
        s_transition_line = 1;
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

    if (small_curve_line)
    {
        kp_mul *= SMALL_CURVE_KP_MUL;
        kd_mul *= SMALL_CURVE_KD_MUL;
        step_base *= SMALL_CURVE_STEP_MUL;
    }

    if (straight_lock)
    {
        kp_mul *= STRAIGHT_LOCK_KP_MUL;
        kd_mul *= STRAIGHT_LOCK_KD_MUL;
        step_base *= STRAIGHT_LOCK_STEP_MUL;
    }

    // 双PD融合：横向PD + 前瞻PD + 陀螺阻尼
    kp_now = servo_kp * (1.0f + 0.55f * curve_intensity) * kp_mul;
    kd_now = servo_kd * (1.0f + DUAL_PD_GYRO_D_CURVE_ADD * curve_intensity) * kd_mul;
    lat_d_now = (servo_kp * DUAL_PD_LAT_D_BASE) * (1.0f + DUAL_PD_LAT_D_CURVE_ADD * curve_intensity) * kd_mul;

    head_kp_now = servo_kff * (1.0f + DUAL_PD_HEAD_P_CURVE_ADD * curve_intensity);
    head_kd_now = head_kp_now * STEER_FF_DPREVIEW_K * DUAL_PD_HEAD_D_GAIN;

    lat_term = (-kp_now * offset_ctrl) - (lat_d_now * errd_filtered);
    head_term = (head_kp_now * preview_filtered) + (head_kd_now * preview_delta);
    if (same_curve && (abs_far_preview >= FAR_PREVIEW_ACTIVE_TH))
    {
        head_term += head_kp_now * FAR_PREVIEW_HEAD_GAIN * preview_far_filtered;
    }
    if (s_transition_line)
    {
        head_term += head_kp_now * S_TRANSITION_HEAD_GAIN * preview_far_filtered;
    }

    ff_limit = STEER_FF_LIMIT_BASE + STEER_FF_LIMIT_CURVE_ADD * curve_intensity;
    if (ring_active)
    {
        ff_limit += RING_FF_LIMIT_ADD;
    }
    if (s_transition_line)
    {
        ff_limit += S_TRANSITION_FF_LIMIT_ADD;
    }
    if ((abs_err <= STEER_FF_SUPPRESS_ERR_TH) && (abs_preview <= STEER_FF_SUPPRESS_PREVIEW_TH))
    {
        head_term *= STEER_FF_SUPPRESS_MUL;
    }

    if (small_curve_line)
    {
        head_term *= SMALL_CURVE_HEAD_MUL;
    }

    if (straight_lock)
    {
        head_term *= STRAIGHT_LOCK_HEAD_MUL;
    }

    if (outside_drift)
    {
        head_term *= OUTSIDE_FF_SUPPRESS;
    }

    if (exit_phase)
    {
        head_term *= DUAL_PD_EXIT_HEAD_KEEP;
    }

    if (ring_active && (ring_state_local == CONTROL_RING_EXIT))
    {
        head_term *= RING_EXIT_HEAD_KEEP;
    }

    head_term = clamp_f(head_term, -ff_limit, ff_limit);

    steer_raw = lat_term + head_term - (kd_now * gyro_z_filtered);

    if (outside_drift)
    {
        steer_raw += (-OUTSIDE_PULL_K * offset_ctrl);
    }

    if (straight_lock)
    {
        steer_raw *= STRAIGHT_LOCK_RAW_DECAY;
        steer_raw_filtered *= STRAIGHT_LOCK_RAW_DECAY;
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
    if (outside_drift)
    {
        steer_step_limit *= OUTSIDE_STEP_MUL;
    }
    if (small_curve_line)
    {
        steer_step_limit *= SMALL_CURVE_STEP_MUL;
    }
    if (s_transition_line)
    {
        steer_step_limit *= S_TRANSITION_STEP_MUL;
    }
    if (straight_lock)
    {
        steer_step_limit *= STRAIGHT_LOCK_STEP_MUL;
    }
    if (ring_active)
    {
        steer_step_limit *= RING_STEP_MUL;
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

    control_debug_steer_out = (int16)steer_out;
    control_debug_curve_x100 = (int16)(curve_intensity * 100.0f + 0.5f);
    control_debug_gyro_z = (int16)gyro_z_filtered;
    control_debug_straight_lock = straight_lock;
    control_debug_s_transition = s_transition_line;

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

    if (small_curve_line)
    {
        speed_scale_total *= SMALL_CURVE_SPEED_KEEP;
    }

    if (s_transition_line)
    {
        speed_scale_total *= S_TRANSITION_SPEED_KEEP;
    }

    if (ring_active)
    {
        if (ring_state_local == CONTROL_RING_INSIDE)
            speed_scale_total *= RING_SPEED_KEEP_INSIDE;
        else if (ring_state_local == CONTROL_RING_EXIT)
            speed_scale_total *= RING_SPEED_KEEP_EXIT;
        else
            speed_scale_total *= RING_SPEED_KEEP_ENTER;
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

    if (ring_active)
    {
        diff_cmd *= RING_DIFF_MUL;
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





