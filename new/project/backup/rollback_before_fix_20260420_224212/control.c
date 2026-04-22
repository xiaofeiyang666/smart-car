#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "brushless.h"
//#include "camera_new.h"

// 速度映射：将速度指令（0~90）换算为编码器目标脉冲
#define MAX_SPEED_PULSES 4000                       // 最大速度映射脉冲；调参：整体速度上不去且电机余量够可增大，速度环易饱和/过冲可减小

// ===================== 调整记录 =====================
// 1) 保持当前结构：纯中线循迹 + 轻量前瞻
// 2) 本次目标：在保持环岛关闭前提下，基础速度提升到15，并强化后轮差速参与度
// 3) 重点策略：入弯前瞻减速、前瞻打角前馈、出弯提前加速、S弯反向切换加速
// 4) 直道加速：普通档 + 激进档（仅在“很直且很稳”触发）

// ------------------------- 转向参数 -------------------------
#define SERVO_DEADBAND_PIX          3       // 像素死区；调参：直线抖动大则增大，小偏差长期不修正/出弯慢则减小
#define SERVO_SMALL_ERR_TH          5       // 小误差阈值；调参：直线来回修正多则增大，轻微偏航纠正慢则减小
#define SERVO_MID_ERR_TH            12      // 中误差阈值；调参：中等弯响应慢则减小，弯中抖动频繁则增大

#define SERVO_KP_SMALL_MUL          1.15f   // 小误差比例增益倍数；调参：直线摆动大则减小，直线修正迟钝则增大
#define SERVO_KP_MID_MUL            1.06f   // 中误差比例增益倍数；调参：入弯跟随慢则增大，弯中来回摆则减小

#define SERVO_MIN_EFFECTIVE_OUT     0.72f   // 舵机最小有效输出；调参：小角度不动则增大，直线细抖明显则减小
#define SERVO_EFFECTIVE_ERR_TH      3       // 启用最小有效输出阈值；调参：小误差不纠正则减小，直线抖动则增大

#define SERVO_STEP_LIMIT_SMALL      1.40f   // 小误差每周期最大打角变化；调参：直线抖动则减小，出弯回中慢则增大
#define SERVO_STEP_LIMIT_MID        3.50f   // 中误差每周期最大打角变化；调参：一般弯响应慢则增大，抖动则减小
#define SERVO_STEP_LIMIT_LARGE      11.20f  // 大误差每周期最大打角变化；调参：急弯来不及转则增大，打角过猛则减小
#define SERVO_OUT_LIMIT             24.0f   // 软件总打角限幅；调参：物理未触碰且转不过去可增大，接近卡胎/不稳则减小
// 反向切弯加速：用于S弯从第一弯切到第二弯时快速回中，避免迟滞出线
#define SERVO_STEP_LIMIT_REVERSE_MUL 1.95f  // S弯反向切换增益；调参：第二弯跟不上则增大，反切抖动则减小
#define SERVO_REVERSE_ERR_TH        2.0f    // 判定反向切换误差阈值；调参：反向介入太晚则减小，误触发过多则增大

// 弯中自动降速：打角越大，基础速度越低，减小车身摆动导致的二次过冲
#define CURVE_SPEED_KEEP_MILD       0.93f   // 轻弯速度保持；调参：轻弯外抛则减小，轻弯过慢则增大
#define CURVE_SPEED_KEEP_MID        0.82f   // 中弯速度保持；调参：中弯推头外抛则减小，中弯拖速则增大
#define CURVE_SPEED_KEEP_HEAVY      0.72f   // 急弯速度保持；调参：急弯冲出则减小，急弯掉速过多则增大

#define GYRO_LPF_ALPHA              0.43f   // 陀螺低通系数；调参：噪声抖动大则减小，转向阻尼滞后则增大

// ------------------------- 前瞻参数 -------------------------
#define PREVIEW_Y_NEAR              (MT9V03X_H - 14) // 近处前瞻行；调参：近处抖动大可再靠下，近场响应慢可上移
#define PREVIEW_Y_FAR               30      // 远处前瞻行；调参：入弯晚则减小(看更远)，远处噪声大则增大
#define PREVIEW_FF_ENABLE_TH        3.4f    // 前馈启动阈值；调参：入弯晚则减小，直线误触发则增大
#define PREVIEW_FF_K                0.32f   // 前馈增益；调参：入弯打角不足则增大，弯前过冲/摆动则减小
#define PREVIEW_FF_MAX              7.6f    // 前馈上限；调参：急弯仍来不及可增大，舵机猛打抖动则减小

#define PREVIEW_SHARP_TH_LOW        8.8f    // 中档减速阈值；调参：中弯减速偏晚则减小，直道被误减速则增大
#define PREVIEW_SHARP_TH_HIGH       12.8f   // 高档减速阈值；调参：急弯仍冲出则减小，急弯前降速过早则增大
#define PREVIEW_SPEED_KEEP_MID      0.76f   // 中档减速保持；调参：中弯外抛则减小，中弯过慢则增大
#define PREVIEW_SPEED_KEEP_HIGH     0.64f   // 高档减速保持；调参：急弯外抛则减小，急弯拖速太多则增大

// ------------------------- 出入弯参数 -------------------------
#define CORNER_ENTRY_PREVIEW_TH         7.2f  // 入弯判定阈值；调参：入弯晚则减小，直道误判入弯则增大
#define CORNER_ENTRY_DPREVIEW_TH        0.58f // 入弯变化率阈值；调参：长直后入弯慢则减小，抖动误触发则增大
#define CORNER_ENTRY_SPEED_KEEP         0.76f // 入弯速度保持；调参：入弯推头则减小，入弯过慢则增大
#define CORNER_ENTRY_STRONG_PREVIEW_TH  11.2f // 强入弯阈值；调参：急弯识别晚则减小，误触发强减速则增大
#define CORNER_ENTRY_STRONG_SPEED_KEEP  0.66f // 强入弯速度保持；调参：急弯冲出则减小，急弯太慢则增大

#define CORNER_EXIT_PREVIEW_TH          7.8f  // 出弯判定前瞻阈值；调参：出弯加速太晚则增大，出弯外抛则减小
#define CORNER_EXIT_STEER_TH            5.0f  // 出弯判定舵角阈值；调参：出弯触发太晚则增大，弯中误出弯则减小
#define CORNER_EXIT_GYRO_TH             100.0f// 出弯判定角速度阈值；调参：出弯慢则增大，姿态未稳就加速则减小

#define CORNER_EXIT_OFFSET_TH           2.2f  // 出弯判定偏差阈值；调参：出弯触发晚则增大，贴外线还加速则减小
#define CORNER_EXIT_BOOST               1.00f // 出弯加速倍率；调参：出弯慢可增大，出弯外抛严重则减小

// 弯道外抛抑制：当偏差与弯向同侧时，优先回中，避免贴外线
#define CURVE_CENTER_ERR_TH             2.4f  // 外漂判定偏差阈值；调参：外漂修正慢则减小，误判过多则增大
#define CURVE_OUTSIDE_FF_SUPPRESS       0.12f // 外漂时前馈抑制；调参：外漂仍加重则减小，转向发木则增大
#define CURVE_OUTSIDE_PULL_K            0.36f // 外漂回中拉力；调参：出弯贴外线则增大，弯中抖动则减小
#define CURVE_OUTSIDE_SPEED_KEEP        0.82f // 外漂附加降速保持；调参：外漂严重则减小，速度掉太多则增大
#define CURVE_OUTSIDE_STEP_MUL          1.55f // 外漂时打角放宽倍数；调参：修正慢则增大，抽动过猛则减小

#define CORNER_STEER_BOOST_PREVIEW_TH   7.8f  // 入弯转向增强阈值；调参：入弯晚则减小，直道误增强则增大
#define CORNER_STEER_BOOST_MUL          1.28f // 入弯转向增强倍数；调参：入弯角度不够则增大，入弯摆动则减小
#define CORNER_STEP_LIMIT_MUL           1.60f // 入弯步进放宽倍数；调参：入弯慢则增大，舵机跳变则减小
#define CORNER_RESCUE_PREVIEW_TH         12.4f // 急弯兜底阈值；调参：兜底介入晚则减小，误兜底降速则增大
#define CORNER_RESCUE_STEER_TH           9.0f  // 急弯兜底舵角阈值；调参：急弯保护不够则减小，触发过多则增大
#define CORNER_RESCUE_SPEED_KEEP         0.80f // 急弯兜底速度保持；调参：急弯冲出则减小，兜底后过慢则增大

// ------------------------- 电子差速参数 -------------------------
#define EDIFF_K_SMALL                   0.70f  // 小舵角差速系数；调参：直线被后轮干扰则减小，轻弯转向不足则增大
#define EDIFF_K_MID                     1.10f  // 中舵角差速系数；调参：中弯指向不足则增大，中弯拖拽感则减小
#define EDIFF_K_LARGE                   1.60f  // 大舵角差速系数；调参：急弯车头不肯进则增大，弯中卡滞/甩尾则减小
#define EDIFF_PREVIEW_BOOST_TH          11.0f  // 大曲率差速增强阈值；调参：急弯增强太晚则减小，误增强则增大
#define EDIFF_PREVIEW_BOOST_MUL         1.45f  // 大曲率差速增强倍数；调参：急弯转不过去则增大，后轮抢方向则减小
#define EDIFF_REVERSE_BOOST_MUL         1.20f  // S弯反切差速增强倍数；调参：第二弯跟不上则增大，反切不稳则减小
#define EDIFF_MAX                        3.00f // 差速系数上限；调参：后轮作用不够可增大，后轮主导姿态则减小
#define EDIFF_GLOBAL_GAIN               1.08f  // 差速全局倍率；调参：整体转向乏力则增大，弯中卡滞/抖动则减小
#define EDIFF_OUTSIDE_DIFF_MUL          1.06f  // 外漂时差速附加；调参：出弯贴外线则增大，外漂时摆尾则减小
#define EDIFF_EXIT_DIFF_MUL             1.02f  // 出弯时差速附加；调参：出弯指向慢则增大，出弯不稳则减小
#define EDIFF_INNER_MIN_RATIO           0.26f  // 内侧轮最低速度比例；调参：转向不灵可减小，内轮拖死/卡顿则增大
#define EDIFF_INNER_MIN_STEER_TH        8.0f   // 启用内轮最低速度舵角阈值；调参：介入太晚则减小，直线误介入则增大
#define EDIFF_INNER_MIN_PREVIEW_TH      9.0f   // 启用内轮最低速度前瞻阈值；调参：急弯介入晚则减小，误介入则增大
#define EDIFF_SPEED_CAP_RATIO           0.30f  // 差速速度上限占比；调参：差速作用弱则增大，差速过猛导致抖动则减小
#define EDIFF_SPEED_CAP_MIN             18      // 差速最小上限；调参：低速弯转向不足则增大，低速扯拽明显则减小

// ------------------------- 直道加速参数 -------------------------
// 普通直道加速：较保守，触发范围更大
#define STRAIGHT_SPEED_BOOST            1.05f  // 普通直道加速倍率；调参：直道偏慢则增大，入弯来不及则减小
#define STRAIGHT_BOOST_STEER_TH         1.9f   // 普通直道舵角阈值；调参：加速触发少则增大，弯前误加速则减小
#define STRAIGHT_BOOST_PREVIEW_TH       4.2f   // 普通直道前瞻阈值；调参：直道加速少则增大，临近弯仍加速则减小
#define STRAIGHT_BOOST_GYRO_TH          70.0f  // 普通直道角速度阈值；调参：触发少则增大，车身未稳就加速则减小

// 激进直道加速：仅在“很直且很稳”时触发
#define STRAIGHT_SPEED_BOOST_AGGR       1.07f  // 激进直道加速倍率；调参：极限尾速不足则增大，长直后冲弯则减小
#define STRAIGHT_AGGR_STEER_TH          1.2f   // 激进加速舵角阈值；调参：激进触发少则增大，弯前误触发则减小
#define STRAIGHT_AGGR_PREVIEW_TH        2.6f   // 激进加速前瞻阈值；调参：触发少则增大，接近弯道仍触发则减小
#define STRAIGHT_AGGR_GYRO_TH           40.0f  // 激进加速角速度阈值；调参：激进触发少则增大，姿态不稳则减小
#define STRAIGHT_AGGR_OFFSET_TH         2.0f   // 激进加速偏差阈值；调参：触发少则增大，中心偏差大还加速则减小
#define STRAIGHT_AGGR_DPREVIEW_TH       0.9f   // 激进加速前瞻变化率阈值；调参：触发少则增大，弯前误加速则减小

volatile uint8 print_flag = 0;

float servo_kp = 0.34f;  // 转向比例主增益；调参：入弯角度不够/跟随慢则增大，直线与弯中过冲抖动则减小
float servo_kd = 0.07f;  // 陀螺阻尼增益；调参：抖动大则增大，转向迟滞/入弯慢则减小

// 基础速度指令（0~90）
int target_speed_base = 15; // 基础目标速度；调参：整体提速先小步+1递增，若冲弯/外抛先回退并配合弯道减速参数

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

    int offset;
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
                        offset_lpf_alpha = 0.22f;
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

    // 4) 图像死区处理：避免小抖动触发无效转向
    if (offset <= SERVO_DEADBAND_PIX && offset >= -SERVO_DEADBAND_PIX)
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











