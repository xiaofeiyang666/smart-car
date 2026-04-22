#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "brushless.h"
//#include "camera_new.h"

// 速度映射：将速度指令（0~90）换算为编码器目标脉冲
#define MAX_SPEED_PULSES 4000

// ===================== 调整记录 =====================
// 1) 保持当前“非赛车边线状态机”结构：纯中线循迹 + 轻量前瞻
// 2) 本次目标：十字环岛段降低外漂风险，提升入弯贴内能力并保持基础速度
// 3) 重点策略：入弯前瞻减速、前瞻打角前馈、出弯提前加速、S弯反向切换加速
// 4) 直道加速：普通档 + 激进档（仅在“很直且很稳”触发）

// ------------------------- 转向参数 -------------------------
#define SERVO_DEADBAND_PIX          4       // 像素死区：进一步抑制高速直线微抖
#define SERVO_SMALL_ERR_TH          5       // 小误差阈值
#define SERVO_MID_ERR_TH            12      // 中误差阈值

#define SERVO_KP_SMALL_MUL          1.00f   // 小误差区比例增益回调，降低直线来回修正
#define SERVO_KP_MID_MUL            1.06f   // 中误差区比例增益放大倍数

#define SERVO_MIN_EFFECTIVE_OUT     0.72f    // 舵机最小有效输出（避免小误差不动作）
#define SERVO_EFFECTIVE_ERR_TH      3       // 达到该误差后启用最小有效输出

#define SERVO_STEP_LIMIT_SMALL      1.00f    // 每20ms小误差区最大打角变化（降低直线抖动）
#define SERVO_STEP_LIMIT_MID        3.50f    // 每20ms中误差区最大打角变化
#define SERVO_STEP_LIMIT_LARGE      11.20f   // 每20ms大误差区最大打角变化（恢复转弯能力）
#define SERVO_OUT_LIMIT             24.0f   // 软件总打角限幅（按用户要求放宽）
// 反向切弯加速：用于S弯从第一弯切到第二弯时快速回中，避免迟滞出线
#define SERVO_STEP_LIMIT_REVERSE_MUL 1.95f
#define SERVO_REVERSE_ERR_TH        2.0f

// 弯中自动降速：打角越大，基础速度越低，减小车身摆动导致的二次过冲
#define CURVE_SPEED_KEEP_MILD       0.93f
#define CURVE_SPEED_KEEP_MID        0.82f
#define CURVE_SPEED_KEEP_HEAVY      0.72f

#define GYRO_LPF_ALPHA              0.43f   // 陀螺仪一阶低通系数（速度15折中）

// ------------------------- 前瞻参数 -------------------------
#define PREVIEW_Y_NEAR              (MT9V03X_H - 14) // 近处参考行
#define PREVIEW_Y_FAR               30               // 远处参考行（增强前瞻）
#define PREVIEW_FF_ENABLE_TH        4.0f             // 前瞻前馈启动阈值上调，避免直线轻微噪声触发前馈
#define PREVIEW_FF_K                0.32f            // 前瞻前馈增益（增强提前打角）
#define PREVIEW_FF_MAX              7.6f             // 前瞻前馈最大补偿（增强前瞻）

#define PREVIEW_SHARP_TH_LOW        8.8f             // 前瞻减速中档阈值（速度15提前介入）
#define PREVIEW_SHARP_TH_HIGH       12.8f            // 前瞻减速高档阈值（速度15提前介入）
#define PREVIEW_SPEED_KEEP_MID      0.76f            // 前瞻减速中档速度保持（速度15稳入弯）
#define PREVIEW_SPEED_KEEP_HIGH     0.64f            // 前瞻减速高档速度保持（速度15稳入弯）

// ------------------------- 出入弯参数 -------------------------
#define CORNER_ENTRY_PREVIEW_TH         7.2f     // 入弯判定前瞻阈值（速度15提前触发）
#define CORNER_ENTRY_DPREVIEW_TH        0.58f    // 入弯判定前瞻增长阈值（速度15提前触发）
#define CORNER_ENTRY_SPEED_KEEP         0.76f    // 入弯阶段速度保持（速度15更稳）
#define CORNER_ENTRY_STRONG_PREVIEW_TH  11.2f    // 强入弯阈值（速度15提前识别急弯）
#define CORNER_ENTRY_STRONG_SPEED_KEEP  0.66f    // 强入弯速度保持（速度15更稳）

#define CORNER_EXIT_PREVIEW_TH          7.8f     // 出弯判定前瞻阈值（收紧，防止误判）
#define CORNER_EXIT_STEER_TH            5.0f     // 出弯判定舵角阈值（收紧，防止误判）
#define CORNER_EXIT_GYRO_TH             100.0f   // 出弯判定角速度阈值（收紧，防止误判）

#define CORNER_EXIT_OFFSET_TH           2.2f
#define CORNER_EXIT_BOOST               1.00f    // 出弯提前加速系数（关闭额外加速，抑制外抛）

// 十字环岛/连续弯防外抛：按弯道状态给中线目标增加“向弯内”偏置
#define CORNER_INNER_BIAS_KEEP_TH         5.2f    // 弯道保持阈值（小于此值开始释放偏置）
#define CORNER_INNER_BIAS_ENTRY           2.6f    // 入弯初段内线偏置（像素）
#define CORNER_INNER_BIAS_HOLD            3.6f    // 弯中保持内线偏置（像素）
#define CORNER_INNER_BIAS_STRONG          5.0f    // 急弯/环岛段内线偏置（像素）
#define CORNER_INNER_BIAS_RISE            1.00f   // 每周期偏置上升步进
#define CORNER_INNER_BIAS_FALL            2.20f   // 每周期偏置下降步进（更快回零，减少直线残余偏置）

// 弯道外抛抑制：当偏差与弯向同侧时，优先回中，避免贴外线
#define CURVE_CENTER_ERR_TH             2.4f
#define CURVE_OUTSIDE_FF_SUPPRESS       0.12f
#define CURVE_OUTSIDE_PULL_K            0.36f
#define CURVE_OUTSIDE_SPEED_KEEP        0.82f
#define CURVE_OUTSIDE_STEP_MUL          1.55f

#define CORNER_STEER_BOOST_PREVIEW_TH   7.8f     // 入弯时提高打角灵敏度阈值
#define CORNER_STEER_BOOST_MUL          1.28f    // 入弯打角增强系数
#define CORNER_STEP_LIMIT_MUL           1.60f    // 入弯打角变化上限增强系数
#define CORNER_RESCUE_PREVIEW_TH         12.4f    // 急弯兜底判定阈值
#define CORNER_RESCUE_STEER_TH           9.0f     // 大舵角兜底阈值
#define CORNER_RESCUE_SPEED_KEEP         0.80f    // 急弯兜底速度保持

// ------------------------- 电子差速参数 -------------------------
#define EDIFF_K_SMALL                   0.70f    // 小舵角差速系数
#define EDIFF_K_MID                     1.10f    // 中舵角差速系数
#define EDIFF_K_LARGE                   1.65f    // 大舵角差速系数
#define EDIFF_PREVIEW_BOOST_TH          11.0f    // 大曲率增强阈值
#define EDIFF_PREVIEW_BOOST_MUL         1.50f    // 大曲率差速增强倍数
#define EDIFF_REVERSE_BOOST_MUL         1.20f    // S弯反向切换时差速增强倍数
#define EDIFF_MAX                       3.00f    // 差速系数上限（收紧，防止后轮过强）
#define EDIFF_GLOBAL_GAIN               1.08f    // 后轮差速全局增强倍数
#define EDIFF_OUTSIDE_DIFF_MUL          1.10f    // 外漂时差速附加倍数（轻量介入）
#define EDIFF_EXIT_DIFF_MUL             1.02f    // 出弯阶段差速附加倍数（轻量介入）
#define EDIFF_INNER_MIN_RATIO           0.26f    // 弯道内侧轮最低速度比例（保留小速并提升转向能力）
#define EDIFF_INNER_MIN_STEER_TH        8.0f     // 启用内轮最低速度的舵角阈值
#define EDIFF_INNER_MIN_PREVIEW_TH      9.0f     // 启用内轮最低速度的前瞻阈值
#define EDIFF_SPEED_CAP_RATIO           0.30f    // 差速速度上限占比（按当前目标速度限差速）
#define EDIFF_SPEED_CAP_MIN             18       // 差速速度最小上限（防止低速过强差速）

// ------------------------- 直道加速参数 -------------------------
// 普通直道加速：较保守，触发范围更大
#define STRAIGHT_SPEED_BOOST            1.05f
#define STRAIGHT_BOOST_STEER_TH         1.9f
#define STRAIGHT_BOOST_PREVIEW_TH       4.2f
#define STRAIGHT_BOOST_GYRO_TH          70.0f

// 激进直道加速：仅在“很直且很稳”时触发
#define STRAIGHT_SPEED_BOOST_AGGR       1.07f
#define STRAIGHT_AGGR_STEER_TH          1.2f
#define STRAIGHT_AGGR_PREVIEW_TH        2.6f
#define STRAIGHT_AGGR_GYRO_TH           40.0f
#define STRAIGHT_AGGR_OFFSET_TH         2.0f
#define STRAIGHT_AGGR_DPREVIEW_TH       0.9f

volatile uint8 print_flag = 0;

float servo_kp = 0.34f;  // 视觉偏差基础比例系数（应急增强转向）
float servo_kd = 0.09f;  // 陀螺仪阻尼系数（减小转向迟滞）

// 基础速度指令（0~90）
int target_speed_base = 15;

PID servo_pid = {0};

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

    PID_Init(&servo_pid, 0.1f, 0.0f, 0.1f, 0, 40);
}

void control_loop(void)
{
    int target_pulses;
    static uint8 loop_cnt = 0;
    static float gyro_z_filtered = 0.0f;
    static float servo_out_last = 0.0f;
    static float offset_filtered = 0.0f;
    static uint8 offset_lpf_init = 0;

    // 前瞻量低通与趋势，用于入弯/出弯判定
    static float preview_abs_filtered = 0.0f;
    static float preview_abs_last = 0.0f;
    static uint8 preview_lpf_init = 0;
    static float corner_inner_bias_state = 0.0f;

    int near_y;
    int far_y;
    int near_mid;
    int far_mid;
    float turn_preview;
    float abs_turn_preview;
    int turn_dir;
    float preview_lpf_alpha;
    float preview_delta;
    float preview_delta_abs;
    float desired_inner_bias;
    float corner_inner_bias_now;
    uint8 corner_inner_entry;
    uint8 corner_inner_hold;
    uint8 corner_inner_strong;

    int offset;
    int deadband_pix;
    float offset_lpf_alpha;

    float gyro_z_actual;
    float abs_offset;
    float kp_now;
    float step_limit;
    float step_limit_now;
    float delta_out;
    float servo_out_raw;
    float servo_out;
    float current_angle;
    float ff_term;

    float abs_steer;
    float diff_k;
    int diff_speed;
    int left_target_pulses;
    int right_target_pulses;
    int inner_min_pulses;
    int diff_speed_cap;

    float speed_scale;
    float preview_speed_scale;
    float entry_exit_scale;
    float straight_boost_scale;
    int base_pulses;
    uint8 entry_phase;
    uint8 exit_phase;
    uint8 reverse_switching;
    uint8 outside_drift;

    // 1) 前瞻信息：由远近中线差得到弯道变化趋势
    near_y = PREVIEW_Y_NEAR;
    if (near_y >= MT9V03X_H) near_y = MT9V03X_H - 1;
    if (near_y <= search_end_line) near_y = search_end_line + 1;

    far_y = PREVIEW_Y_FAR;
    if (far_y <= search_end_line) far_y = search_end_line + 1;
    if (far_y >= MT9V03X_H) far_y = MT9V03X_H - 1;

    near_mid = mid_line_list[near_y];
    far_mid = mid_line_list[far_y];
    turn_preview = (float)(far_mid - near_mid);
    abs_turn_preview = abs_f(turn_preview);
    turn_dir = (turn_preview >= 0.0f) ? 1 : -1;

    if (!preview_lpf_init)
    {
        preview_abs_filtered = abs_turn_preview;
        preview_abs_last = abs_turn_preview;
        preview_lpf_init = 1;
    }

                preview_lpf_alpha = 0.40f;
    preview_abs_filtered = preview_abs_filtered * (1.0f - preview_lpf_alpha) + abs_turn_preview * preview_lpf_alpha;
    preview_delta = preview_abs_filtered - preview_abs_last;
    preview_abs_last = preview_abs_filtered;
    preview_delta_abs = abs_f(preview_delta);

    // 十字环岛/复合弯道：用“向弯内目标偏置”抑制长期贴外线
    corner_inner_entry = ((preview_abs_filtered >= CORNER_ENTRY_PREVIEW_TH) &&
                          ((preview_delta >= CORNER_ENTRY_DPREVIEW_TH) ||
                           (preview_abs_filtered >= CORNER_ENTRY_STRONG_PREVIEW_TH)));
    corner_inner_hold = (preview_abs_filtered >= CORNER_INNER_BIAS_KEEP_TH);
    corner_inner_strong = (preview_abs_filtered >= CORNER_ENTRY_STRONG_PREVIEW_TH);

    desired_inner_bias = 0.0f;
    if (corner_inner_hold)
    {
        desired_inner_bias = CORNER_INNER_BIAS_HOLD;
        if (corner_inner_entry) desired_inner_bias = CORNER_INNER_BIAS_ENTRY;
        if (corner_inner_strong) desired_inner_bias = CORNER_INNER_BIAS_STRONG;
    }

    if (desired_inner_bias > corner_inner_bias_state)
    {
        corner_inner_bias_state += CORNER_INNER_BIAS_RISE;
        if (corner_inner_bias_state > desired_inner_bias) corner_inner_bias_state = desired_inner_bias;
    }
    else
    {
        corner_inner_bias_state -= CORNER_INNER_BIAS_FALL;
        if (corner_inner_bias_state < desired_inner_bias) corner_inner_bias_state = desired_inner_bias;
    }
    corner_inner_bias_now = corner_inner_bias_state;
    // 2) 基础偏差：纯中线循迹，不使用赛车边线状态机
    offset = (int)final_mid_line - MID_W;

    // 3) 偏差低通：直线更稳，弯道保持响应
    if (!offset_lpf_init)
    {
        offset_filtered = (float)offset;
        offset_lpf_init = 1;
    }

    if (preview_abs_filtered >= PREVIEW_FF_ENABLE_TH)
    {
                                offset_lpf_alpha = 0.50f;
    }
    else
    {
                        offset_lpf_alpha = 0.16f;
    }

    offset_filtered = offset_filtered * (1.0f - offset_lpf_alpha) + (float)offset * offset_lpf_alpha;
    if (offset_filtered >= 0.0f)
    {
        offset = (int)(offset_filtered + 0.5f);
    }
    else
    {
        offset = (int)(offset_filtered - 0.5f);
    }

        // 将目标中线向弯内偏置：仅在弯道保持阶段生效，避免直线残余偏置引发抖动
    if (corner_inner_hold)
    {
        if (turn_dir > 0)
        {
            offset += (int)(corner_inner_bias_now + 0.5f);
        }
        else
        {
            offset -= (int)(corner_inner_bias_now + 0.5f);
        }
    }

        // 4) 图像死区处理：直线放大死区，抑制左右来回微修正
    deadband_pix = SERVO_DEADBAND_PIX;
    if (preview_abs_filtered < PREVIEW_FF_ENABLE_TH)
    {
        deadband_pix = SERVO_DEADBAND_PIX + 1;
    }
    if (offset <= deadband_pix && offset >= -deadband_pix)
    {
        offset = 0;
    }

    // 5) 获取陀螺仪并进行一阶低通
    imu660ra_get_gyro();
    gyro_z_actual = imu660ra_gyro_transition(imu660ra_gyro_z);
    gyro_z_filtered = (1.0f - GYRO_LPF_ALPHA) * gyro_z_filtered + GYRO_LPF_ALPHA * gyro_z_actual;
    // 6) 比例增益分段：小误差稳、中误差准、大误差快
    abs_offset = abs_f((float)offset);
    outside_drift = 0;
    if ((preview_abs_filtered >= PREVIEW_FF_ENABLE_TH) &&
        (abs_offset >= CURVE_CENTER_ERR_TH) &&
        (((offset > 0) && (turn_dir > 0)) || ((offset < 0) && (turn_dir < 0))))
    {
        outside_drift = 1;
    }
    if (abs_offset <= SERVO_SMALL_ERR_TH)
    {
        kp_now = servo_kp * SERVO_KP_SMALL_MUL;
        step_limit = SERVO_STEP_LIMIT_SMALL;
    }
    else if (abs_offset <= SERVO_MID_ERR_TH)
    {
        kp_now = servo_kp * SERVO_KP_MID_MUL;
        step_limit = SERVO_STEP_LIMIT_MID;
    }
    else
    {
        kp_now = servo_kp;
        step_limit = SERVO_STEP_LIMIT_LARGE;
    }

    // 7) 融合PD：视觉偏差 + 陀螺仪阻尼
    servo_out_raw = (-kp_now * (float)offset) - (servo_kd * gyro_z_filtered);

    // 8) 前瞻前馈：提前建立转向（入弯更及时）
    ff_term = 0.0f;
    if (preview_abs_filtered > PREVIEW_FF_ENABLE_TH)
    {
        ff_term = PREVIEW_FF_K * (preview_abs_filtered - PREVIEW_FF_ENABLE_TH);
        if (ff_term > PREVIEW_FF_MAX) ff_term = PREVIEW_FF_MAX;
        if (outside_drift)
        {
            ff_term *= CURVE_OUTSIDE_FF_SUPPRESS;
        }
        servo_out_raw += (float)turn_dir * ff_term;
    }

    // 入弯阶段适当增强转向，避免高速入弯打角偏晚
    if (preview_abs_filtered >= CORNER_STEER_BOOST_PREVIEW_TH)
    {
        servo_out_raw *= CORNER_STEER_BOOST_MUL;
    }
    // 外漂时额外施加回中拉力，避免出弯持续贴外线
    if (outside_drift)
    {
        servo_out_raw += (-CURVE_OUTSIDE_PULL_K * (float)offset);
    }

    // 9) 舵机静区补偿：防止小偏差长期不动，后续突发大修正
    if ((abs_offset >= SERVO_EFFECTIVE_ERR_TH) && (abs_f(servo_out_raw) < SERVO_MIN_EFFECTIVE_OUT))
    {
        servo_out_raw = (servo_out_raw >= 0.0f) ? SERVO_MIN_EFFECTIVE_OUT : -SERVO_MIN_EFFECTIVE_OUT;
    }

    // 10) 反向切换加速：S弯切换方向时提升回中速度
    step_limit_now = step_limit;
    reverse_switching = 0;
    if ((abs_offset >= SERVO_REVERSE_ERR_TH) &&
        ((servo_out_last > 0.0f && servo_out_raw < 0.0f) ||
         (servo_out_last < 0.0f && servo_out_raw > 0.0f)))
    {
        reverse_switching = 1;
        step_limit_now = step_limit * SERVO_STEP_LIMIT_REVERSE_MUL;
    }

    // 入弯时进一步放宽打角变化上限，保证“来得及打角”
    if ((preview_abs_filtered >= CORNER_ENTRY_PREVIEW_TH) && (abs_offset >= SERVO_SMALL_ERR_TH))
    {
        step_limit_now *= CORNER_STEP_LIMIT_MUL;
    }
    // 外漂时允许更快改向，避免迟滞导致持续贴外线
    if (outside_drift)
    {
        step_limit_now *= CURVE_OUTSIDE_STEP_MUL;
    }
    // 11) 打角变化限速：限制每周期突变，提升车身稳定性
    delta_out = servo_out_raw - servo_out_last;
    if (delta_out > step_limit_now)
    {
        servo_out = servo_out_last + step_limit_now;
    }
    else if (delta_out < -step_limit_now)
    {
        servo_out = servo_out_last - step_limit_now;
    }
    else
    {
        servo_out = servo_out_raw;
    }
    // 12) 固定限幅：已回退防打死卡住逻辑，仅保留固定舵角限幅
    servo_out = clamp_f(servo_out, -SERVO_OUT_LIMIT, SERVO_OUT_LIMIT);
    servo_out_last = servo_out;
    current_angle = SERVO_CENTER + servo_out;
    servo_set_angle(current_angle);

    // 13) 电子差速增强：大舵角 + 大曲率 + S弯反切时提高后轮差速
    abs_steer = abs_f(servo_out);
    if (abs_steer <= 3.0f)
    {
        diff_k = EDIFF_K_SMALL;
    }
    else if (abs_steer <= 8.0f)
    {
        diff_k = EDIFF_K_MID;
    }
    else
    {
        diff_k = EDIFF_K_LARGE;
    }

    if (preview_abs_filtered >= EDIFF_PREVIEW_BOOST_TH)
    {
        diff_k *= EDIFF_PREVIEW_BOOST_MUL;
    }

    if (reverse_switching)
    {
        diff_k *= EDIFF_REVERSE_BOOST_MUL;
    }

    if (diff_k > EDIFF_MAX) diff_k = EDIFF_MAX;

    // 14) 弯中自动降速：打角越大，基础速度保持比例越低
    if (abs_steer <= 3.0f)
    {
        speed_scale = 1.00f;
    }
    else if (abs_steer <= 8.0f)
    {
        speed_scale = CURVE_SPEED_KEEP_MILD;
    }
    else if (abs_steer <= 12.0f)
    {
        speed_scale = CURVE_SPEED_KEEP_MID;
    }
    else
    {
        speed_scale = CURVE_SPEED_KEEP_HEAVY;
    }
    // 外漂时额外轻降速，优先保证回中能力
    if (outside_drift)
    {
        speed_scale *= CURVE_OUTSIDE_SPEED_KEEP;
    }
    // 15) 前瞻减速：还没打到大角前先减速，避免“冲到弯里再修正”
    preview_speed_scale = 1.00f;
    if (preview_abs_filtered >= PREVIEW_SHARP_TH_HIGH)
    {
        preview_speed_scale = PREVIEW_SPEED_KEEP_HIGH;
    }
    else if (preview_abs_filtered >= PREVIEW_SHARP_TH_LOW)
    {
        preview_speed_scale = PREVIEW_SPEED_KEEP_MID;
    }
    // 急弯兜底：若前瞻曲率或舵角过大，再做一层保命减速
    if ((preview_abs_filtered >= CORNER_RESCUE_PREVIEW_TH) || (abs_steer >= CORNER_RESCUE_STEER_TH))
    {
        speed_scale *= CORNER_RESCUE_SPEED_KEEP;
    }

    // 16) 出入弯速度因子：入弯主动压速，出弯条件满足后提前拉速
    entry_phase = ((preview_abs_filtered >= CORNER_ENTRY_PREVIEW_TH) &&
                   ((preview_delta >= CORNER_ENTRY_DPREVIEW_TH) ||
                    (preview_abs_filtered >= CORNER_ENTRY_STRONG_PREVIEW_TH)));

    // 出弯判定增加“曲率快速回落”通道，避免出弯加速触发偏晚
    exit_phase = (((preview_abs_filtered <= CORNER_EXIT_PREVIEW_TH) &&
                   (abs_steer <= CORNER_EXIT_STEER_TH) &&
                   (abs_f(gyro_z_filtered) <= CORNER_EXIT_GYRO_TH) &&
                   (abs_offset <= CORNER_EXIT_OFFSET_TH)) ||
                  ((preview_delta <= -1.2f) &&
                   (preview_abs_filtered <= PREVIEW_SHARP_TH_LOW) &&
                   (abs_steer <= (CORNER_EXIT_STEER_TH + 1.5f)) &&
                   (abs_offset <= (CORNER_EXIT_OFFSET_TH + 1.0f))));

    entry_exit_scale = 1.00f;
    if (entry_phase)
    {
        entry_exit_scale = CORNER_ENTRY_SPEED_KEEP;
        if (preview_abs_filtered >= CORNER_ENTRY_STRONG_PREVIEW_TH)
        {
            entry_exit_scale = CORNER_ENTRY_STRONG_SPEED_KEEP;
        }
    }
    else if (exit_phase)
    {
        entry_exit_scale = CORNER_EXIT_BOOST;
        if (outside_drift)
        {
            entry_exit_scale = 1.00f;
        }
    }

    // 17) 直道自动加速：普通档 + 激进档（仅在非入弯阶段启用）
    straight_boost_scale = 1.00f;
    if (!entry_phase && !exit_phase)
    {
        // 激进档：只有在“很直且很稳”时才触发，提升直道尾速
        if ((abs_steer <= STRAIGHT_AGGR_STEER_TH) &&
            (preview_abs_filtered <= STRAIGHT_AGGR_PREVIEW_TH) &&
            (abs_f(gyro_z_filtered) <= STRAIGHT_AGGR_GYRO_TH) &&
            (abs_offset <= STRAIGHT_AGGR_OFFSET_TH) &&
            (preview_delta_abs <= STRAIGHT_AGGR_DPREVIEW_TH))
        {
            straight_boost_scale = STRAIGHT_SPEED_BOOST_AGGR;
        }
        else if ((abs_steer <= STRAIGHT_BOOST_STEER_TH) &&
                 (preview_abs_filtered <= STRAIGHT_BOOST_PREVIEW_TH) &&
                 (abs_f(gyro_z_filtered) <= STRAIGHT_BOOST_GYRO_TH))
        {
            straight_boost_scale = STRAIGHT_SPEED_BOOST;
        }
    }

    diff_speed = (int)(servo_out * diff_k * EDIFF_GLOBAL_GAIN);
    // 外漂与出弯阶段额外提高后轮差速参与度，抑制出弯贴外线
    if (outside_drift)
    {
        diff_speed = (int)((float)diff_speed * EDIFF_OUTSIDE_DIFF_MUL);
    }
    else if (exit_phase)
    {
        diff_speed = (int)((float)diff_speed * EDIFF_EXIT_DIFF_MUL);
    }

    base_pulses = (int)(((long)target_speed_base * MAX_SPEED_PULSES) / 90);
    target_pulses = (int)((float)base_pulses * speed_scale * preview_speed_scale * entry_exit_scale * straight_boost_scale);

    // 限制速度目标范围，防止异常抖动导致速度环突变
    if (target_pulses < (int)((float)base_pulses * 0.40f)) target_pulses = (int)((float)base_pulses * 0.40f);
    if (target_pulses > (int)((float)base_pulses * 1.35f)) target_pulses = (int)((float)base_pulses * 1.35f);

    // 差速上限保护：限制后轮差速对整车姿态的主导，避免弯中被后轮“卡住”
    diff_speed_cap = (int)((float)target_pulses * EDIFF_SPEED_CAP_RATIO);
    if (diff_speed_cap < EDIFF_SPEED_CAP_MIN) diff_speed_cap = EDIFF_SPEED_CAP_MIN;
    if (diff_speed > diff_speed_cap) diff_speed = diff_speed_cap;
    else if (diff_speed < -diff_speed_cap) diff_speed = -diff_speed_cap;

    encoder_update();

    left_target_pulses = target_pulses + diff_speed;
    right_target_pulses = target_pulses - diff_speed;

    // 急弯时给内侧轮保留一个很小的最低速度，避免完全压死导致姿态僵硬
    if ((abs_steer >= EDIFF_INNER_MIN_STEER_TH) &&
        (preview_abs_filtered >= EDIFF_INNER_MIN_PREVIEW_TH))
    {
        inner_min_pulses = (int)((float)base_pulses * EDIFF_INNER_MIN_RATIO);
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

    left_motor_speed_pid_calc(left_target_pulses, left_speed);
    right_motor_speed_pid_calc(right_target_pulses, right_speed);

    set_motor_speed((int)left_motor_speedpid.output, (int)right_motor_speedpid.output);

    loop_cnt++;
    if(loop_cnt >= 5)
    {
        loop_cnt = 0;
        print_flag = 1;
    }
}




















