#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "brushless.h"

// 速度映射：将速度指令（0~90）换算为编码器目标脉冲
#define MAX_SPEED_PULSES 4000

// ===================== 调参总说明 =====================
// 1) 先关直道加速（STRAIGHT_ACCEL_ENABLE=0）把车调稳，再开加速。
// 2) 先调稳态转向（直线不抖、弯道不甩），再提 target_speed_base。
// 3) 每次只改一组参数，建议每次改动 5%~10%。

// ------------------------- 视觉特征与滤波 -------------------------
#define PREVIEW_Y_NEAR                  (MT9V03X_H - 14) // 近前瞻行；进弯慢可上移(减小)，直线抖动可下移(增大)
#define PREVIEW_Y_FAR                   30               // 远前瞻行；进弯晚可减小，远处噪声大可增大

#define OFFSET_LPF_ALPHA_STRAIGHT       0.22f            // 直道偏差低通；直道抖动大就减小，直道纠偏慢就增大
#define OFFSET_LPF_ALPHA_CURVE          0.50f            // 弯道偏差低通；弯道响应慢就增大，弯中抖动就减小
#define PREVIEW_LPF_ALPHA               0.42f            // 前瞻低通；冲弯可增大，误触发可减小
#define GYRO_LPF_ALPHA                  0.30f            // 陀螺低通；噪声大减小，滞后大增大

// ------------------------- 舵机基础参数 -------------------------
#define SERVO_DEADBAND_PIX              6                // 偏差死区；直线抖动大增大，细小偏差不修正减小
#define SERVO_OUT_LIMIT                 23.0f            // 软件舵角限幅；转不过弯增大，机械干涉风险减小

#define SERVO_RATE_STRAIGHT             0.55f            // 直道每周期最大打角变化；直道抖动减小，直道纠偏慢增大
#define SERVO_RATE_ENTRY                1.90f            // 入弯打角速率；入弯慢增大，入弯抽动减小
#define SERVO_RATE_APEX                 2.40f            // 弯中打角速率；弯中转不过去增大，弯中摆动减小
#define SERVO_RATE_EXIT                 1.25f            // 出弯打角速率；出弯回正慢增大，出弯晃动减小
#define SERVO_RATE_RECOVERY             2.80f            // 救车打角速率；救车不够快增大，救车时抽动减小

// 状态增益调度（在 servo_kp / servo_kd / servo_kff 基础上乘以下列倍数）
#define STEER_GAIN_STRAIGHT_MUL         0.85f            // 直道比例倍数；直道抖动大减小
#define STEER_GAIN_ENTRY_MUL            1.05f            // 入弯比例倍数；入弯慢增大
#define STEER_GAIN_APEX_MUL             1.15f            // 弯中比例倍数；贴外线增大，摆动大减小
#define STEER_GAIN_EXIT_MUL             0.95f            // 出弯比例倍数；出弯外抛增大，出弯抖动减小
#define STEER_GAIN_RECOVERY_MUL         1.20f            // 救车比例倍数；救车不够增大

#define STEER_GYRO_STRAIGHT_MUL         1.15f            // 直道陀螺阻尼倍数；直道摆动大增大
#define STEER_GYRO_ENTRY_MUL            1.00f            // 入弯陀螺阻尼倍数
#define STEER_GYRO_APEX_MUL             0.95f            // 弯中陀螺阻尼倍数；弯中迟钝可减小
#define STEER_GYRO_EXIT_MUL             1.05f            // 出弯陀螺阻尼倍数；出弯来回晃增大
#define STEER_GYRO_RECOVERY_MUL         1.00f            // 救车陀螺阻尼倍数

#define STEER_FF_STRAIGHT_MUL           0.65f            // 直道前馈倍数；直道误打角可减小
#define STEER_FF_ENTRY_MUL              1.00f            // 入弯前馈倍数；入弯慢可增大
#define STEER_FF_APEX_MUL               1.20f            // 弯中前馈倍数；大弯贴外线可增大
#define STEER_FF_EXIT_MUL               0.85f            // 出弯前馈倍数；出弯抖动可减小
#define STEER_FF_RECOVERY_MUL           1.10f            // 救车前馈倍数
#define PREVIEW_FF_LIMIT                7.8f             // 前馈限幅；冲击大减小，弯道不够转增大

// ------------------------- 状态机阈值 -------------------------
#define STATE_STRAIGHT_PREVIEW_TH       2.8f             // 直道判定前瞻阈值；太敏感增大，直道识别慢减小
#define STATE_STRAIGHT_OFFSET_TH        2.0f             // 直道判定偏差阈值；直道误判弯道增大
#define STATE_STRAIGHT_GYRO_TH          40.0f            // 直道判定角速度阈值；车身晃动仍被判直道则减小

#define STATE_ENTRY_PREVIEW_TH          6.0f             // 入弯判定前瞻阈值；入弯晚减小
#define STATE_ENTRY_DPREVIEW_TH         0.85f            // 入弯变化率阈值；长直后冲弯减小
#define STATE_APEX_PREVIEW_TH           8.8f             // 弯心判定前瞻阈值；弯中切换慢减小

#define STATE_EXIT_PREVIEW_TH           4.8f             // 出弯判定前瞻阈值；出弯过早加速减小，出弯慢增大
#define STATE_EXIT_OFFSET_TH            2.4f             // 出弯判定偏差阈值；出弯不稳减小

#define STATE_RECOVERY_OFFSET_TH        17.0f            // 救车偏差阈值；冲出时来不及救减小，误触发增大

#define STATE_ENTRY_HOLD_CYCLES         6                // 入弯最短保持周期（20ms*6=120ms）
#define STATE_APEX_HOLD_CYCLES          8                // 弯心最短保持周期
#define STATE_EXIT_HOLD_CYCLES          5                // 出弯最短保持周期
#define STATE_RECOVERY_HOLD_CYCLES      8                // 救车最短保持周期

// 长直后额外刹车：抑制“长直冲弯”
#define LONG_STRAIGHT_COUNT_TH          26               // 判定长直线周期数；过于频繁触发可增大
#define LONG_STRAIGHT_BRAKE_CYCLES      8                // 入弯额外刹车持续周期
#define LONG_STRAIGHT_ENTRY_KEEP        0.84f            // 长直入弯额外速度保持；冲弯则减小，掉速过多则增大

// ------------------------- 速度规划 -------------------------
#define SPEED_KEEP_ENTRY                0.78f            // 入弯状态速度保持；入弯冲出减小
#define SPEED_KEEP_APEX_BASE            0.68f            // 弯中状态基础速度保持；弯中不稳减小
#define SPEED_KEEP_EXIT                 0.86f            // 出弯状态速度保持；出弯慢增大，出弯甩动减小
#define SPEED_KEEP_RECOVERY             0.60f            // 救车状态速度保持；救车不住减小

#define SPEED_CURVE_PREVIEW_SCALE       11.5f            // 前瞻归一化尺度；弯道减速太早增大，太晚减小
#define SPEED_CURVE_K                   0.95f            // 曲率降速强度；弯道过快不稳增大
#define SPEED_CURVE_MIN                 0.62f            // 曲率降速下限；弯中太慢增大，冲弯减小

// 目标速度斜坡（抑制速度突变导致车身不稳）
#define TARGET_STEP_UP_STRAIGHT         16.0f            // 直道每周期最大升速脉冲；突然加速则减小
#define TARGET_STEP_UP_TURN             8.0f             // 弯道每周期最大升速脉冲；弯中还在提速则减小
#define TARGET_STEP_DOWN                64.0f            // 每周期最大降速脉冲；减速不及时则增大
#define TARGET_SCALE_MIN                0.42f            // 相对基础速度最小比例；过慢可增大
#define TARGET_SCALE_MAX                1.15f            // 相对基础速度最大比例；过猛加速可减小

// ------------------------- 直道加速（单逻辑） -------------------------
#define STRAIGHT_ACCEL_ENABLE           0                // 0关闭先调稳，1开启再提尾速
#define STRAIGHT_SPEED_BOOST            1.03f            // 直道加速倍率；加速突兀减小
#define STRAIGHT_BOOST_PREVIEW_TH       2.2f             // 直道加速前瞻阈值；弯前误加速减小
#define STRAIGHT_BOOST_OFFSET_TH        1.2f             // 直道加速偏差阈值；车身未正还加速则减小
#define STRAIGHT_BOOST_GYRO_TH          32.0f            // 直道加速角速度阈值；姿态不稳还加速则减小
#define STRAIGHT_BOOST_DPREVIEW_TH      0.25f            // 直道加速前瞻变化率阈值；长直后冲弯则减小
#define STRAIGHT_BOOST_STABLE_CYCLES    12               // 稳定周期门槛；加速触发太频繁可增大

// ------------------------- 后轮差速（温和） -------------------------
#define DIFF_K_STRAIGHT                 0.00f            // 直道差速系数；直道建议保持 0
#define DIFF_K_ENTRY                    0.55f            // 入弯差速系数；入弯转向不足可增大
#define DIFF_K_APEX                     0.78f            // 弯中差速系数；贴外线可增大，后轮抢方向减小
#define DIFF_K_EXIT                     0.42f            // 出弯差速系数；出弯指向慢可增大
#define DIFF_K_RECOVERY                 0.68f            // 救车差速系数；救车不足可增大
#define DIFF_FILTER_ALPHA               0.30f            // 差速低通；后轮抖动大减小

#define DIFF_MAX_CAP_RATIO              0.22f            // 差速上限占比；后轮主导姿态减小
#define DIFF_MIN_CAP                    14               // 差速下限保护；低速差速无感可增大

#define DIFF_INNER_MIN_RATIO            0.36f            // 弯中内轮最小速度比例；内轮拖死可增大
#define DIFF_INNER_MIN_STEER_TH         9.0f             // 启用内轮最小速度的舵角阈值
#define DIFF_INNER_MIN_PREVIEW_TH       8.0f             // 启用内轮最小速度的前瞻阈值

volatile uint8 print_flag = 0;
volatile int16 ctrl_dbg_offset_raw = 0;
volatile int16 ctrl_dbg_offset_filt = 0;
volatile int16 ctrl_dbg_preview_x10 = 0;
volatile int16 ctrl_dbg_steer_x10 = 0;
volatile int16 ctrl_dbg_state = 0;

// 手动可调主增益（先调这三个）
float servo_kp = 0.30f;                                   // 比例增益；入弯慢增大，摆动大减小
float servo_kd = 0.12f;                                   // 陀螺阻尼；摆动大增大，转向迟滞减小
float servo_kff = 0.22f;                                  // 前瞻前馈；贴外线增大，弯前过冲减小

// 基础速度指令（0~90）
int target_speed_base = 15;                               // 先稳后快；稳态后再按 +1 提速

PID servo_pid = {0};

typedef enum
{
    TRACK_STRAIGHT = 0,
    TRACK_ENTRY    = 1,
    TRACK_APEX     = 2,
    TRACK_EXIT     = 3,
    TRACK_RECOVERY = 4
} track_state_e;

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

    // 保留原 PID 结构，不改底层接口
    PID_Init(&servo_pid, 0.1f, 0.0f, 0.1f, 0, 40);
}

void control_loop(void)
{
    static uint8 loop_cnt = 0;

    static float gyro_z_filtered = 0.0f;
    static float offset_filtered = 0.0f;
    static float preview_filtered = 0.0f;
    static float preview_last = 0.0f;

    static float servo_out_last = 0.0f;
    static float target_pulses_ramped = 0.0f;
    static float diff_filtered = 0.0f;

    static uint8 lpf_init = 0;
    static uint8 track_state = TRACK_STRAIGHT;
    static uint8 state_hold = 0;
    static uint16 straight_counter = 0;
    static uint8 long_straight_brake_cnt = 0;

    int near_y;
    int far_y;
    int near_mid;
    int far_mid;

    int offset_raw;
    int offset;

    float preview_raw;
    float preview_delta;

    float offset_lpf_alpha;
    float gyro_z_actual;

    float abs_preview;
    float abs_offset;
    float abs_gyro;

    uint8 straight_cond;
    uint8 entry_cond;
    uint8 apex_cond;
    uint8 exit_cond;
    uint8 recovery_cond;

    uint8 prev_state;

    float kp_mul;
    float kd_mul;
    float kff_mul;
    float rate_limit;

    float ff_term;
    float steer_raw;
    float steer_out;
    float delta_out;
    float current_angle;

    int base_pulses;
    int target_pulses;
    int target_pulses_des;
    int target_pulses_min;
    int target_pulses_max;

    float speed_keep_state;
    float speed_keep_curve;
    float speed_keep_total;
    float curve_norm;

    float straight_boost_scale;
    float accel_step_up;

    float abs_steer;
    float diff_k;
    float diff_cmd;
    int diff_speed;
    int diff_cap;
    int inner_min_pulses;

    int left_target_pulses;
    int right_target_pulses;

    // 1) 计算远近前瞻
    near_y = PREVIEW_Y_NEAR;
    if (near_y >= MT9V03X_H) near_y = MT9V03X_H - 1;
    if (near_y <= search_end_line) near_y = search_end_line + 1;

    far_y = PREVIEW_Y_FAR;
    if (far_y <= search_end_line) far_y = search_end_line + 1;
    if (far_y >= MT9V03X_H) far_y = MT9V03X_H - 1;

    near_mid = mid_line_list[near_y];
    far_mid = mid_line_list[far_y];

    preview_raw = (float)(far_mid - near_mid);            // 有符号前瞻（带方向）
    offset_raw = (int)final_mid_line - MID_W;             // 有符号横向偏差
    ctrl_dbg_offset_raw = (int16)offset_raw;

    if (!lpf_init)
    {
        offset_filtered = (float)offset_raw;
        preview_filtered = preview_raw;
        preview_last = preview_raw;
        target_pulses_ramped = 0.0f;
        lpf_init = 1;
    }

    // 2) 低通滤波
    if (abs_f(preview_filtered) >= STATE_ENTRY_PREVIEW_TH)
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

    // 3) 获取陀螺仪
    imu660ra_get_gyro();
    gyro_z_actual = imu660ra_gyro_transition(imu660ra_gyro_z);
    gyro_z_filtered = (1.0f - GYRO_LPF_ALPHA) * gyro_z_filtered + GYRO_LPF_ALPHA * gyro_z_actual;

    // 4) 滤波值转整数偏差并加死区
    if (offset_filtered >= 0.0f)
    {
        offset = (int)(offset_filtered + 0.5f);
    }
    else
    {
        offset = (int)(offset_filtered - 0.5f);
    }

    if (offset <= SERVO_DEADBAND_PIX && offset >= -SERVO_DEADBAND_PIX)
    {
        offset = 0;
    }
    ctrl_dbg_offset_filt = (int16)offset;
    ctrl_dbg_preview_x10 = (int16)(preview_filtered * 10.0f);

    abs_preview = abs_f(preview_filtered);
    abs_offset = abs_f((float)offset);
    abs_gyro = abs_f(gyro_z_filtered);

    // 5) 状态判定条件
    straight_cond = (abs_preview <= STATE_STRAIGHT_PREVIEW_TH) &&
                    (abs_offset <= STATE_STRAIGHT_OFFSET_TH) &&
                    (abs_gyro <= STATE_STRAIGHT_GYRO_TH);

    entry_cond = (abs_preview >= STATE_ENTRY_PREVIEW_TH) ||
                 (abs_f(preview_delta) >= STATE_ENTRY_DPREVIEW_TH);

    apex_cond = (abs_preview >= STATE_APEX_PREVIEW_TH) ||
                (abs_offset >= (STATE_STRAIGHT_OFFSET_TH + 3.0f));

    exit_cond = (abs_preview <= STATE_EXIT_PREVIEW_TH) &&
                (abs_offset <= STATE_EXIT_OFFSET_TH) &&
                (abs_gyro <= STATE_STRAIGHT_GYRO_TH);

    recovery_cond = (abs_offset >= STATE_RECOVERY_OFFSET_TH);

    // 直道累计：用于长直后提前重刹
    if (straight_cond)
    {
        if (straight_counter < 60000) straight_counter++;
    }
    else if (track_state == TRACK_STRAIGHT)
    {
        straight_counter = 0;
    }

    // 6) 状态机（带最短保持，避免抖动切换）
    prev_state = track_state;
    ctrl_dbg_state = (int16)track_state;

    if (state_hold > 0) state_hold--;

    switch (track_state)
    {
    case TRACK_STRAIGHT:
        if (recovery_cond)
        {
            track_state = TRACK_RECOVERY;
            state_hold = STATE_RECOVERY_HOLD_CYCLES;
        }
        else if (entry_cond)
        {
            track_state = TRACK_ENTRY;
            state_hold = STATE_ENTRY_HOLD_CYCLES;
        }
        break;

    case TRACK_ENTRY:
        if (recovery_cond)
        {
            track_state = TRACK_RECOVERY;
            state_hold = STATE_RECOVERY_HOLD_CYCLES;
        }
        else if ((state_hold == 0) && apex_cond)
        {
            track_state = TRACK_APEX;
            state_hold = STATE_APEX_HOLD_CYCLES;
        }
        else if ((state_hold == 0) && straight_cond && !entry_cond)
        {
            track_state = TRACK_STRAIGHT;
        }
        break;

    case TRACK_APEX:
        if (recovery_cond)
        {
            track_state = TRACK_RECOVERY;
            state_hold = STATE_RECOVERY_HOLD_CYCLES;
        }
        else if ((state_hold == 0) && exit_cond)
        {
            track_state = TRACK_EXIT;
            state_hold = STATE_EXIT_HOLD_CYCLES;
        }
        break;

    case TRACK_EXIT:
        if (recovery_cond)
        {
            track_state = TRACK_RECOVERY;
            state_hold = STATE_RECOVERY_HOLD_CYCLES;
        }
        else if ((state_hold == 0) && straight_cond)
        {
            track_state = TRACK_STRAIGHT;
        }
        else if ((state_hold == 0) && entry_cond)
        {
            track_state = TRACK_ENTRY;
            state_hold = STATE_ENTRY_HOLD_CYCLES;
        }
        break;

    default:    // TRACK_RECOVERY
        if ((state_hold == 0) && (abs_offset <= STATE_EXIT_OFFSET_TH))
        {
            if (entry_cond)
            {
                track_state = TRACK_ENTRY;
                state_hold = STATE_ENTRY_HOLD_CYCLES;
            }
            else
            {
                track_state = TRACK_STRAIGHT;
            }
        }
        break;
    }

    // 记录长直后入弯刹车触发
    if ((prev_state == TRACK_STRAIGHT) && (track_state == TRACK_ENTRY))
    {
        if (straight_counter >= LONG_STRAIGHT_COUNT_TH)
        {
            long_straight_brake_cnt = LONG_STRAIGHT_BRAKE_CYCLES;
        }
        straight_counter = 0;
    }
    else if (track_state != TRACK_ENTRY)
    {
        long_straight_brake_cnt = 0;
    }

    // 7) 转向增益调度
    kp_mul = 1.0f;
    kd_mul = 1.0f;
    kff_mul = 1.0f;
    rate_limit = SERVO_RATE_STRAIGHT;

    switch (track_state)
    {
    case TRACK_STRAIGHT:
        kp_mul = STEER_GAIN_STRAIGHT_MUL;
        kd_mul = STEER_GYRO_STRAIGHT_MUL;
        kff_mul = STEER_FF_STRAIGHT_MUL;
        rate_limit = SERVO_RATE_STRAIGHT;
        break;
    case TRACK_ENTRY:
        kp_mul = STEER_GAIN_ENTRY_MUL;
        kd_mul = STEER_GYRO_ENTRY_MUL;
        kff_mul = STEER_FF_ENTRY_MUL;
        rate_limit = SERVO_RATE_ENTRY;
        break;
    case TRACK_APEX:
        kp_mul = STEER_GAIN_APEX_MUL;
        kd_mul = STEER_GYRO_APEX_MUL;
        kff_mul = STEER_FF_APEX_MUL;
        rate_limit = SERVO_RATE_APEX;
        break;
    case TRACK_EXIT:
        kp_mul = STEER_GAIN_EXIT_MUL;
        kd_mul = STEER_GYRO_EXIT_MUL;
        kff_mul = STEER_FF_EXIT_MUL;
        rate_limit = SERVO_RATE_EXIT;
        break;
    default:
        kp_mul = STEER_GAIN_RECOVERY_MUL;
        kd_mul = STEER_GYRO_RECOVERY_MUL;
        kff_mul = STEER_FF_RECOVERY_MUL;
        rate_limit = SERVO_RATE_RECOVERY;
        break;
    }

    // 8) 连续舵机控制：反馈 + 前馈 + 速率限制
    ff_term = servo_kff * kff_mul * preview_filtered;
    ff_term = clamp_f(ff_term, -PREVIEW_FF_LIMIT, PREVIEW_FF_LIMIT);

    steer_raw = (-(servo_kp * kp_mul) * (float)offset) - ((servo_kd * kd_mul) * gyro_z_filtered) + ff_term;
    steer_raw = clamp_f(steer_raw, -SERVO_OUT_LIMIT, SERVO_OUT_LIMIT);

    delta_out = steer_raw - servo_out_last;
    if (delta_out > rate_limit)
    {
        steer_out = servo_out_last + rate_limit;
    }
    else if (delta_out < -rate_limit)
    {
        steer_out = servo_out_last - rate_limit;
    }
    else
    {
        steer_out = steer_raw;
    }

    steer_out = clamp_f(steer_out, -SERVO_OUT_LIMIT, SERVO_OUT_LIMIT);
    servo_out_last = steer_out;
    ctrl_dbg_steer_x10 = (int16)(steer_out * 10.0f);

    current_angle = SERVO_CENTER + steer_out;
    servo_set_angle(current_angle);

    // 9) 速度规划：状态速度 * 曲率速度
    switch (track_state)
    {
    case TRACK_STRAIGHT:
        speed_keep_state = 1.00f;
        break;
    case TRACK_ENTRY:
        speed_keep_state = SPEED_KEEP_ENTRY;
        break;
    case TRACK_APEX:
        speed_keep_state = SPEED_KEEP_APEX_BASE;
        break;
    case TRACK_EXIT:
        speed_keep_state = SPEED_KEEP_EXIT;
        break;
    default:
        speed_keep_state = SPEED_KEEP_RECOVERY;
        break;
    }

    curve_norm = abs_preview / SPEED_CURVE_PREVIEW_SCALE;
    if (curve_norm < 0.0f) curve_norm = 0.0f;
    if (curve_norm > 1.6f) curve_norm = 1.6f;

    speed_keep_curve = 1.0f / (1.0f + SPEED_CURVE_K * curve_norm * curve_norm);
    if (speed_keep_curve < SPEED_CURVE_MIN) speed_keep_curve = SPEED_CURVE_MIN;

    speed_keep_total = speed_keep_state * speed_keep_curve;

    // 长直后入弯额外减速
    if ((track_state == TRACK_ENTRY) && (long_straight_brake_cnt > 0))
    {
        speed_keep_total *= LONG_STRAIGHT_ENTRY_KEEP;
        long_straight_brake_cnt--;
    }

    // 单逻辑直道加速（可开关）
    straight_boost_scale = 1.0f;
    if (STRAIGHT_ACCEL_ENABLE == 1)
    {
        if ((track_state == TRACK_STRAIGHT) &&
            (straight_counter >= STRAIGHT_BOOST_STABLE_CYCLES) &&
            (abs_preview <= STRAIGHT_BOOST_PREVIEW_TH) &&
            (abs_offset <= STRAIGHT_BOOST_OFFSET_TH) &&
            (abs_gyro <= STRAIGHT_BOOST_GYRO_TH) &&
            (abs_f(preview_delta) <= STRAIGHT_BOOST_DPREVIEW_TH))
        {
            straight_boost_scale = STRAIGHT_SPEED_BOOST;
        }
    }

    speed_keep_total *= straight_boost_scale;
    speed_keep_total = clamp_f(speed_keep_total, TARGET_SCALE_MIN, TARGET_SCALE_MAX);

    base_pulses = (int)(((long)target_speed_base * MAX_SPEED_PULSES) / 90);
    target_pulses_des = (int)((float)base_pulses * speed_keep_total);

    target_pulses_min = (int)((float)base_pulses * TARGET_SCALE_MIN);
    target_pulses_max = (int)((float)base_pulses * TARGET_SCALE_MAX);
    if (target_pulses_des < target_pulses_min) target_pulses_des = target_pulses_min;
    if (target_pulses_des > target_pulses_max) target_pulses_des = target_pulses_max;

    // 速度斜坡：弯中慢升速、全程快降速
    if (target_pulses_ramped <= 1.0f)
    {
        target_pulses_ramped = (float)target_pulses_des;
    }

    if (track_state == TRACK_STRAIGHT)
    {
        accel_step_up = TARGET_STEP_UP_STRAIGHT;
    }
    else
    {
        accel_step_up = TARGET_STEP_UP_TURN;
    }

    if ((float)target_pulses_des > target_pulses_ramped + accel_step_up)
    {
        target_pulses_ramped += accel_step_up;
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

    // 10) 后轮差速：按状态给温和增益，并做上限保护
    abs_steer = abs_f(steer_out);

    switch (track_state)
    {
    case TRACK_STRAIGHT:
        diff_k = DIFF_K_STRAIGHT;
        break;
    case TRACK_ENTRY:
        diff_k = DIFF_K_ENTRY;
        break;
    case TRACK_APEX:
        diff_k = DIFF_K_APEX;
        break;
    case TRACK_EXIT:
        diff_k = DIFF_K_EXIT;
        break;
    default:
        diff_k = DIFF_K_RECOVERY;
        break;
    }

    if (abs_steer < 2.0f)
    {
        diff_k *= 0.30f;
    }

    diff_cmd = steer_out * diff_k;
    diff_filtered = diff_filtered * (1.0f - DIFF_FILTER_ALPHA) + diff_cmd * DIFF_FILTER_ALPHA;
    diff_speed = (int)diff_filtered;

    diff_cap = (int)((float)target_pulses * DIFF_MAX_CAP_RATIO);
    if (diff_cap < DIFF_MIN_CAP) diff_cap = DIFF_MIN_CAP;
    if (track_state == TRACK_STRAIGHT)
    {
        diff_cap = diff_cap / 2;
        if (diff_cap < 8) diff_cap = 8;
    }

    if (diff_speed > diff_cap) diff_speed = diff_cap;
    else if (diff_speed < -diff_cap) diff_speed = -diff_cap;

    left_target_pulses = target_pulses + diff_speed;
    right_target_pulses = target_pulses - diff_speed;

    // 急弯内轮最低速度保护，防止内轮被压死后车身僵硬
    if ((track_state != TRACK_STRAIGHT) &&
        (abs_steer >= DIFF_INNER_MIN_STEER_TH) &&
        (abs_preview >= DIFF_INNER_MIN_PREVIEW_TH))
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

    // 11) 编码器 + 电机 PID（保留原 PID 架构）
    encoder_update();

    left_motor_speed_pid_calc(left_target_pulses, left_speed);
    right_motor_speed_pid_calc(right_target_pulses, right_speed);

    set_motor_speed((int)left_motor_speedpid.output, (int)right_motor_speedpid.output);

    // 周期打印触发
    loop_cnt++;
    if (loop_cnt >= 5)
    {
        loop_cnt = 0;
        print_flag = 1;
    }
}