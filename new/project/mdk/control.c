#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"
#include "camera.h"
#include "shoot.h"

/* ===================== 底盘速度参数 =====================
 * target_speed_base 是基础速度目标，单位是“5ms 内编码器计数”。
 * 例：target_speed_base = 100，表示速度环每 5ms 希望编码器读数约为 100。
 *
 * target_speed_curve_min 是弯道最低目标速度。
 * 当前使用参考工程 Dream_speed 同类思路：舵机打得越大，速度从 base 线性降到 curve_min。
 * 调参建议：
 * 1. 先把 80 跑稳，再提高 target_speed_base 到 100/120。
 * 2. 如果高速入弯甩、满舵过弯吃力，先降低 target_speed_curve_min。
 * 3. 如果弯道明显太慢、出弯拖沓，再提高 target_speed_curve_min。
 */
#define SPEED_TARGET_MIN          0
#define SPEED_TARGET_MAX          900
#define BASE_SPEED_MATCH_GAIN     1.00f
#define CURVE_SPEED_MIN_DEFAULT   75
#define SHOOT_APPROACH_SPEED      10
#define SHOOT_REVERSE_PWM         16
#define SHOOT_FORWARD_PWM         12

/* ===================== 舵机输出参数 =====================
 * SERVO_OUT_LIMIT_LEFT/RIGHT 由 servo.h 的机械限幅自动换算而来。
 * SERVO_DEADBAND_PIX 是视觉误差死区，误差小于该值时不打舵。
 *   增大：直线更稳，但小偏差修正更迟钝。
 *   减小：修正更灵敏，但直线更容易左右抖。
 * SERVO_DUTY_STEP_DEG 是单次控制周期最大舵机变化量，限制舵机突变。
 *   增大：入弯反应更快，但高速更容易甩。
 *   减小：动作更柔和，但可能入弯晚。
 * GYRO_STEER_LIMIT_DEG 是陀螺阻尼最大等效舵角，防止 gyro 项压过视觉。
 */
#define SERVO_OUT_LIMIT_LEFT      (SERVO_DUTY_CENTER - SERVO_DUTY_L_MAX)
#define SERVO_OUT_LIMIT_RIGHT     (SERVO_DUTY_R_MAX - SERVO_DUTY_CENTER)
#define SERVO_DEADBAND_PIX        3.0f
#define SERVO_DUTY_STEP_DEG       5.5f
#define STEER_OUT_STEP            (SERVO_DUTY_PER_DEGREE * SERVO_DUTY_STEP_DEG)
#define GYRO_STEER_LIMIT_DEG      2.5f
#define GYRO_STEER_LIMIT          (SERVO_DUTY_PER_DEGREE * GYRO_STEER_LIMIT_DEG)

/* ===================== 左右轮差速参数 =====================
 * 差速只跟舵机输出幅度有关，满舵时达到 DIFF_MAX_RATIO。
 * DIFF_STEER_DEADBAND：小舵角内不差速，避免直线左右轮目标乱跳。
 * DIFF_MAX_RATIO：
 *   0.00 = 关闭差速，最接近借鉴工程当前实际运行方式。
 *   0.10~0.20 = 温和差速，适合先稳定高速过弯。
 *   0.30 以上 = 差速很强，可能帮助急弯，但也更容易甩尾和左右摆。
 */
//#define DIFF_STEER_DEADBAND       2.0f
//#define DIFF_MAX_RATIO            0.2f

/* ===================== 左右轮非对称差速参数 ===================== */
#define DIFF_STEER_DEADBAND       2.0f   // 舵角死区，直道不差速
// 差速系数分离：内轮负责“拽”车头（减速多），外轮负责维持动能（加速少）
#define DIFF_RATIO_INNER          0.30f  // 内轮最大减速比例 (相当于旧工程的 kp_dif_jian)
#define DIFF_RATIO_OUTER          0.10f  // 外轮最大加速比例 (相当于旧工程的 kp_dif_jia)

volatile uint8 print_flag = 0;
volatile int16 imu_gyro_z_dps_x10 = 0;
volatile int16 control_debug_preview_raw = 0;
volatile int16 control_debug_near_bias_raw = 0;
volatile int16 control_debug_used_bias_x10 = 0;
volatile int16 control_debug_preview_filtered_x10 = 0;
volatile int16 control_debug_preview_far_raw = 0;
volatile int16 control_debug_curve_raw = 0;
volatile int16 control_debug_steer_p_x100 = 0;
volatile int16 control_debug_steer_kp2_x100 = 0;
volatile int16 control_debug_steer_ff_x100 = 0;
volatile int16 control_debug_steer_out_x100 = 0;
volatile int16 control_debug_left_target = 0;
volatile int16 control_debug_right_target = 0;
volatile int16 control_debug_left_speed = 0;
volatile int16 control_debug_right_speed = 0;
volatile int16 control_debug_diff_speed = 0;
volatile int16 control_debug_left_pwm = 0;
volatile int16 control_debug_right_pwm = 0;
volatile uint8 control_debug_camera_confidence = 0;
volatile uint8 control_debug_valid_line_cnt = 0;
volatile uint8 control_debug_lost_left_cnt = 0;
volatile uint8 control_debug_lost_right_cnt = 0;
volatile uint8 control_debug_curve_exit_hold_cnt = 0;
volatile int16 control_debug_speed_scale_x100 = 0;
volatile uint8 control_debug_route_mode = 0;
volatile uint8 control_debug_cross_state = 0;
volatile uint8 control_debug_cross_left_open_cnt = 0;
volatile uint8 control_debug_cross_right_open_cnt = 0;
volatile uint8 control_debug_cross_both_open_cnt = 0;
volatile uint8 control_debug_left_control = 0;
volatile uint8 control_debug_right_control = 0;
volatile uint8 control_debug_mid_control = 0;
volatile uint8 control_debug_ring_midpoint = 0;
volatile uint8 control_debug_ring_mid_under = 0;
volatile uint8 control_debug_ring_left115 = 0;
volatile uint8 control_debug_ring_left85 = 0;
volatile uint8 control_debug_ring_left55 = 0;

/* 基础速度目标，单位：5ms 编码器计数。调高速主要改这里。 */
int target_speed_base = 100;
/* 弯道最低速度目标。base 提到 100/120 后，可先保持 80。 */
int target_speed_curve_min = CURVE_SPEED_MIN_DEFAULT;

/* 舵机一次项增益：越大越灵敏，过大表现为 steer_out 经常打到限幅。 */
float servo_kp = 2.5f;
/* 舵机二次项增益：大误差时额外加舵，小误差影响小；过大容易入弯突然打死。 */
float servo_kp2 = 0.013f;
/* 陀螺阻尼增益：imu660ra_gyro_z > 0 为右转，当前公式用正 kg 抑制车头转动。 */
float servo_kg = 0.002f;

static float steer_out_last = 0.0f;

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

static int clamp_i(int x, int min_v, int max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static float clamp_steer_f(float x)
{
    if (x > SERVO_OUT_LIMIT_LEFT) return SERVO_OUT_LIMIT_LEFT;
    if (x < -SERVO_OUT_LIMIT_RIGHT) return -SERVO_OUT_LIMIT_RIGHT;
    return x;
}

static float steer_limit_for_side(float steer)
{
    if (steer >= 0.0f) return SERVO_OUT_LIMIT_LEFT;
    return SERVO_OUT_LIMIT_RIGHT;
}

void control_init(void)
{
    left_motor_speed_pid_init(0.10f, 0.003f, 0.0f, 0, 90);
    right_motor_speed_pid_init(0.10f, 0.003f, 0.0f, 0, 90);
    servo_set_angle(SERVO_CENTER);
}

void control_loop(void)
{
    int bias_raw;
    int preview_raw;
    int preview_far_raw;
    int curve_raw;
    float error;
    float gyro_z_dps;
    float gyro_z_raw;
    float steer_p_term;
    float steer_kp2_term;
    float steer_gyro_term;
    float steer_out;
    float steer_step;
    float abs_steer;
    float abs_steer_deg;
    float steer_limit;
    float curve_ratio;
    int base_pulses;
    int curve_min_pulses;
    int target_pulses;
    int diff_speed = 0;
    int left_target_pulses;
    int right_target_pulses;
    int speed_scale_x100;
		int diff_speed_inner = 0;
    int diff_speed_outer = 0;

    imu660ra_get_gyro();
    gyro_z_dps = imu660ra_gyro_transition(imu660ra_gyro_z);
    gyro_z_raw = (float)imu660ra_gyro_z;
    imu_gyro_z_dps_x10 = (int16)(gyro_z_dps * 10.0f);

    bias_raw = clamp_i((int)camera_bias_raw, -90, 90);
    preview_raw = clamp_i((int)camera_preview_raw, -90, 90);
    preview_far_raw = clamp_i((int)camera_preview_far_raw, -90, 90);
    curve_raw = clamp_i((int)camera_curve_raw, -90, 90);

    //error = (float)preview_raw;
		error = (float)preview_raw * 0.90f + (float)preview_far_raw * 0.10f;
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

    steer_p_term = servo_kp * error;
    steer_kp2_term = servo_kp2 * error * abs_f(error);
    steer_gyro_term = clamp_f(-gyro_z_raw * servo_kg, -GYRO_STEER_LIMIT, GYRO_STEER_LIMIT);
    steer_out = clamp_steer_f(steer_p_term + steer_kp2_term + steer_gyro_term);

    steer_step = steer_out - steer_out_last;
    if (steer_step > STEER_OUT_STEP)
    {
        steer_out = steer_out_last + STEER_OUT_STEP;
    }
    else if (steer_step < -STEER_OUT_STEP)
    {
        steer_out = steer_out_last - STEER_OUT_STEP;
    }
    steer_out = clamp_steer_f(steer_out);
    steer_out_last = steer_out;
    abs_steer = abs_f(steer_out);
    abs_steer_deg = abs_steer / SERVO_DUTY_PER_DEGREE;

    servo_set_duty(SERVO_DUTY_CENTER - steer_out);

    base_pulses = target_speed_base;
    if (base_pulses < SPEED_TARGET_MIN) base_pulses = SPEED_TARGET_MIN;
    if (base_pulses > SPEED_TARGET_MAX) base_pulses = SPEED_TARGET_MAX;
    base_pulses = (int)((float)base_pulses * BASE_SPEED_MATCH_GAIN);
    curve_min_pulses = target_speed_curve_min;
    if (curve_min_pulses < SPEED_TARGET_MIN) curve_min_pulses = SPEED_TARGET_MIN;
    if (curve_min_pulses > base_pulses) curve_min_pulses = base_pulses;

    steer_limit = steer_limit_for_side(steer_out);
    if (steer_limit < 1.0f) steer_limit = 1.0f;
    curve_ratio = abs_steer / steer_limit;
    if (curve_ratio > 1.0f) curve_ratio = 1.0f;
    target_pulses = base_pulses - (int)((float)(base_pulses - curve_min_pulses) * curve_ratio);
    if (target_pulses < 0) target_pulses = 0;

    if (shoot_stop_request)
    {
        target_pulses = 0;
    }
    else if (shoot_slow_request && target_pulses > SHOOT_APPROACH_SPEED)
    {
        target_pulses = SHOOT_APPROACH_SPEED;
    }

    speed_scale_x100 = 100;
    if (base_pulses > 0)
    {
        speed_scale_x100 = (target_pulses * 100) / base_pulses;
    }

//    diff_speed = 0;
//    if (abs_steer_deg > DIFF_STEER_DEADBAND)
//    {
//        diff_speed = (int)((float)target_pulses * DIFF_MAX_RATIO * (abs_steer / steer_limit));
//        if (diff_speed > (int)((float)target_pulses * DIFF_MAX_RATIO))
//        {
//            diff_speed = (int)((float)target_pulses * DIFF_MAX_RATIO);
//        }
//    }

//    if (steer_out > 0.0f)
//    {
//        left_target_pulses = target_pulses - diff_speed;
//        right_target_pulses = target_pulses + diff_speed;
//    }
//    else if (steer_out < 0.0f)
//    {
//        left_target_pulses = target_pulses + diff_speed;
//        right_target_pulses = target_pulses - diff_speed;
//    }
//    else
//    {
//        left_target_pulses = target_pulses;
//        right_target_pulses = target_pulses;
//    }
// 1. 根据当前实际舵角占比，计算内外轮的差速增量
    if (abs_steer_deg > DIFF_STEER_DEADBAND)
    {
        float steer_ratio = abs_steer / steer_limit;
        if (steer_ratio > 1.0f) steer_ratio = 1.0f; // 限制最大比例为1

        // 分别计算内轮应该减多少，外轮应该加多少
        diff_speed_inner = (int)((float)target_pulses * DIFF_RATIO_INNER * steer_ratio);
        diff_speed_outer = (int)((float)target_pulses * DIFF_RATIO_OUTER * steer_ratio);
    }

    // 2. 根据转弯方向，分配给左右轮
    if (steer_out < 0.0f) // 左转 (左轮是内轮，右轮是外轮)
    {
        left_target_pulses = target_pulses - diff_speed_inner;   // 内轮大减速
        right_target_pulses = target_pulses + diff_speed_outer;  // 外轮小加速
    }
    else if (steer_out > 0.0f) // 右转 (右轮是内轮，左轮是外轮)
    {
        left_target_pulses = target_pulses + diff_speed_outer;   // 外轮小加速
        right_target_pulses = target_pulses - diff_speed_inner;  // 内轮大减速
    }
    else // 直行
    {
        left_target_pulses = target_pulses;
        right_target_pulses = target_pulses;
    }

    if (left_target_pulses < 0) left_target_pulses = 0;
    if (right_target_pulses < 0) right_target_pulses = 0;

    encoder_update();

    if (shoot_reverse_request)
    {
        left_motor_speedpid.error = 0;
        left_motor_speedpid.lastError = 0;
        left_motor_speedpid.prevError = 0;
        left_motor_speedpid.output = 0;
        right_motor_speedpid.error = 0;
        right_motor_speedpid.lastError = 0;
        right_motor_speedpid.prevError = 0;
        right_motor_speedpid.output = 0;
        servo_set_angle(SERVO_CENTER);
        set_motor_speed(-SHOOT_REVERSE_PWM, -SHOOT_REVERSE_PWM);

        control_debug_preview_raw = (int16)preview_raw;
        control_debug_near_bias_raw = (int16)bias_raw;
        control_debug_used_bias_x10 = (int16)(error * 10.0f);
        control_debug_preview_filtered_x10 = (int16)(error * 10.0f);
        control_debug_preview_far_raw = (int16)preview_far_raw;
        control_debug_curve_raw = (int16)curve_raw;
        control_debug_steer_p_x100 = 0;
        control_debug_steer_kp2_x100 = 0;
        control_debug_steer_ff_x100 = 0;
        control_debug_steer_out_x100 = 0;
        control_debug_left_target = -SHOOT_REVERSE_PWM;
        control_debug_right_target = -SHOOT_REVERSE_PWM;
        control_debug_left_speed = left_speed;
        control_debug_right_speed = right_speed;
        control_debug_diff_speed = 0;
        control_debug_left_pwm = -SHOOT_REVERSE_PWM;
        control_debug_right_pwm = -SHOOT_REVERSE_PWM;
        control_debug_camera_confidence = camera_confidence;
        control_debug_valid_line_cnt = camera_valid_line_cnt;
        control_debug_lost_left_cnt = camera_lost_left_cnt;
        control_debug_lost_right_cnt = camera_lost_right_cnt;
        control_debug_curve_exit_hold_cnt = 0;
        control_debug_speed_scale_x100 = 0;
        control_debug_route_mode = camera_route_mode;
        control_debug_cross_state = camera_cross_state;
        control_debug_cross_left_open_cnt = camera_debug_width_80;
        control_debug_cross_right_open_cnt = camera_debug_width_60;
        control_debug_cross_both_open_cnt = camera_debug_mid_60;
        control_debug_left_control = camera_debug_left_control;
        control_debug_right_control = camera_debug_right_control;
        control_debug_mid_control = camera_debug_mid_control;
        control_debug_ring_midpoint = camera_debug_ring_midpoint;
        control_debug_ring_mid_under = camera_debug_ring_mid_under;
        control_debug_ring_left115 = camera_debug_ring_left115;
        control_debug_ring_left85 = camera_debug_ring_left85;
        control_debug_ring_left55 = camera_debug_ring_left55;
        shoot_task_5ms();
        print_flag = 1;
        return;
    }

    if (shoot_forward_request)
    {
        left_motor_speedpid.error = 0;
        left_motor_speedpid.lastError = 0;
        left_motor_speedpid.prevError = 0;
        left_motor_speedpid.output = 0;
        right_motor_speedpid.error = 0;
        right_motor_speedpid.lastError = 0;
        right_motor_speedpid.prevError = 0;
        right_motor_speedpid.output = 0;
        servo_set_angle(SERVO_CENTER);
        set_motor_speed(SHOOT_FORWARD_PWM, SHOOT_FORWARD_PWM);

        control_debug_preview_raw = (int16)preview_raw;
        control_debug_near_bias_raw = (int16)bias_raw;
        control_debug_used_bias_x10 = (int16)(error * 10.0f);
        control_debug_preview_filtered_x10 = (int16)(error * 10.0f);
        control_debug_preview_far_raw = (int16)preview_far_raw;
        control_debug_curve_raw = (int16)curve_raw;
        control_debug_steer_p_x100 = 0;
        control_debug_steer_kp2_x100 = 0;
        control_debug_steer_ff_x100 = 0;
        control_debug_steer_out_x100 = 0;
        control_debug_left_target = SHOOT_FORWARD_PWM;
        control_debug_right_target = SHOOT_FORWARD_PWM;
        control_debug_left_speed = left_speed;
        control_debug_right_speed = right_speed;
        control_debug_diff_speed = 0;
        control_debug_left_pwm = SHOOT_FORWARD_PWM;
        control_debug_right_pwm = SHOOT_FORWARD_PWM;
        control_debug_camera_confidence = camera_confidence;
        control_debug_valid_line_cnt = camera_valid_line_cnt;
        control_debug_lost_left_cnt = camera_lost_left_cnt;
        control_debug_lost_right_cnt = camera_lost_right_cnt;
        control_debug_curve_exit_hold_cnt = 0;
        control_debug_speed_scale_x100 = 0;
        control_debug_route_mode = camera_route_mode;
        control_debug_cross_state = camera_cross_state;
        control_debug_cross_left_open_cnt = camera_debug_width_80;
        control_debug_cross_right_open_cnt = camera_debug_width_60;
        control_debug_cross_both_open_cnt = camera_debug_mid_60;
        control_debug_left_control = camera_debug_left_control;
        control_debug_right_control = camera_debug_right_control;
        control_debug_mid_control = camera_debug_mid_control;
        control_debug_ring_midpoint = camera_debug_ring_midpoint;
        control_debug_ring_mid_under = camera_debug_ring_mid_under;
        control_debug_ring_left115 = camera_debug_ring_left115;
        control_debug_ring_left85 = camera_debug_ring_left85;
        control_debug_ring_left55 = camera_debug_ring_left55;
        shoot_task_5ms();
        print_flag = 1;
        return;
    }

    if (shoot_stop_request)
    {
        left_motor_speedpid.error = 0;
        left_motor_speedpid.lastError = 0;
        left_motor_speedpid.prevError = 0;
        left_motor_speedpid.output = 0;
        right_motor_speedpid.error = 0;
        right_motor_speedpid.lastError = 0;
        right_motor_speedpid.prevError = 0;
        right_motor_speedpid.output = 0;
        set_motor_speed(0, 0);

        control_debug_preview_raw = (int16)preview_raw;
        control_debug_near_bias_raw = (int16)bias_raw;
        control_debug_used_bias_x10 = (int16)(error * 10.0f);
        control_debug_preview_filtered_x10 = (int16)(error * 10.0f);
        control_debug_preview_far_raw = (int16)preview_far_raw;
        control_debug_curve_raw = (int16)curve_raw;
        control_debug_steer_p_x100 = (int16)(steer_p_term * 100.0f);
        control_debug_steer_kp2_x100 = (int16)(steer_kp2_term * 100.0f);
        control_debug_steer_ff_x100 = (int16)(steer_gyro_term * 100.0f);
        control_debug_steer_out_x100 = (int16)(steer_out * 100.0f);
        control_debug_left_target = 0;
        control_debug_right_target = 0;
        control_debug_left_speed = left_speed;
        control_debug_right_speed = right_speed;
        control_debug_diff_speed = 0;
        control_debug_left_pwm = 0;
        control_debug_right_pwm = 0;
        control_debug_camera_confidence = camera_confidence;
        control_debug_valid_line_cnt = camera_valid_line_cnt;
        control_debug_lost_left_cnt = camera_lost_left_cnt;
        control_debug_lost_right_cnt = camera_lost_right_cnt;
        control_debug_curve_exit_hold_cnt = 0;
        control_debug_speed_scale_x100 = 0;
        control_debug_route_mode = camera_route_mode;
        control_debug_cross_state = camera_cross_state;
        control_debug_cross_left_open_cnt = camera_debug_width_80;
        control_debug_cross_right_open_cnt = camera_debug_width_60;
        control_debug_cross_both_open_cnt = camera_debug_mid_60;
        control_debug_left_control = camera_debug_left_control;
        control_debug_right_control = camera_debug_right_control;
        control_debug_mid_control = camera_debug_mid_control;
        control_debug_ring_midpoint = camera_debug_ring_midpoint;
        control_debug_ring_mid_under = camera_debug_ring_mid_under;
        control_debug_ring_left115 = camera_debug_ring_left115;
        control_debug_ring_left85 = camera_debug_ring_left85;
        control_debug_ring_left55 = camera_debug_ring_left55;
        shoot_task_5ms();
        print_flag = 1;
        return;
    }

    left_motor_speed_pid_calc(left_target_pulses, left_speed);
    right_motor_speed_pid_calc(right_target_pulses, right_speed);

    set_motor_speed((int)left_motor_speedpid.output, (int)right_motor_speedpid.output);

    control_debug_preview_raw = (int16)preview_raw;
    control_debug_near_bias_raw = (int16)bias_raw;
    control_debug_used_bias_x10 = (int16)(error * 10.0f);
    control_debug_preview_filtered_x10 = (int16)(error * 10.0f);
    control_debug_preview_far_raw = (int16)preview_far_raw;
    control_debug_curve_raw = (int16)curve_raw;
    control_debug_steer_p_x100 = (int16)(steer_p_term * 100.0f);
    control_debug_steer_kp2_x100 = (int16)(steer_kp2_term * 100.0f);
    control_debug_steer_ff_x100 = (int16)(steer_gyro_term * 100.0f);
    control_debug_steer_out_x100 = (int16)(steer_out * 100.0f);
    control_debug_left_target = (int16)left_target_pulses;
    control_debug_right_target = (int16)right_target_pulses;
    control_debug_left_speed = left_speed;
    control_debug_right_speed = right_speed;
    control_debug_diff_speed = (int16)diff_speed;
    control_debug_left_pwm = (int16)left_motor_speedpid.output;
    control_debug_right_pwm = (int16)right_motor_speedpid.output;
    control_debug_camera_confidence = camera_confidence;
    control_debug_valid_line_cnt = camera_valid_line_cnt;
    control_debug_lost_left_cnt = camera_lost_left_cnt;
    control_debug_lost_right_cnt = camera_lost_right_cnt;
    control_debug_curve_exit_hold_cnt = 0;
    control_debug_speed_scale_x100 = (int16)speed_scale_x100;
    control_debug_route_mode = camera_route_mode;
    control_debug_cross_state = camera_cross_state;
    control_debug_cross_left_open_cnt = camera_debug_width_80;
		control_debug_cross_right_open_cnt = camera_debug_width_60;
		control_debug_cross_both_open_cnt = camera_debug_mid_60;
    control_debug_left_control = camera_debug_left_control;
    control_debug_right_control = camera_debug_right_control;
    control_debug_mid_control = camera_debug_mid_control;
    control_debug_ring_midpoint = camera_debug_ring_midpoint;
    control_debug_ring_mid_under = camera_debug_ring_mid_under;
    control_debug_ring_left115 = camera_debug_ring_left115;
		control_debug_ring_left85 = camera_debug_ring_left85;
		control_debug_ring_left55 = camera_debug_ring_left55;
		shoot_task_5ms();

    print_flag = 1;
}
