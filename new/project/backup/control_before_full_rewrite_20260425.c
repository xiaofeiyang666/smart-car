	#include "control.h"
	#include "pid.h"
	#include "encoder.h"
	#include "motor.h"
	#include "servo.h"
	#include "camera.h"
	#include "brushless.h"

	// 速度映射：将速度指令（0~90）换算为编码器目标脉冲
	#define MAX_SPEED_PULSES 4000

	#define BASE_SPEED_MATCH_GAIN          1.06f            // 基础速度标定倍率；闭环15比直驱15慢则增大，冲弯则减小
	// ===================== 调参总说明 =====================
	// 1) 先关直道加速（STRAIGHT_ACCEL_ENABLE=0）把车调稳，再开加速。
	// 2) 先调稳态转向（直线不抖、弯道不甩），再提 target_speed_base。
	// 3) 每次只改一组参数，建议每次改动 5%~10%。

	// ------------------------- 视觉特征与滤波 -------------------------
	#define PREVIEW_Y_NEAR                  (MT9V03X_H - 14) // 近前瞻行；进弯慢可上移(减小)，直线抖动可下移(增大)
	#define PREVIEW_Y_FAR                   30               // 远前瞻行；进弯晚可减小，远处噪声大可增大

	#define OFFSET_LPF_ALPHA_STRAIGHT       0.25f            // 直道偏差低通；直线修正慢增大，直线抖动大减小
	#define OFFSET_LPF_ALPHA_CURVE          0.65f            // 弯道偏差低通；入弯慢增大，弯中抖动减小
	#define PREVIEW_LPF_ALPHA               0.36f            // 前瞻低通；冲弯可增大，误触发可减小
	#define GYRO_LPF_ALPHA                  0.30f            // 陀螺低通；噪声大减小，滞后大增大

	// ------------------------- 舵机基础参数 -------------------------
	#define SERVO_DEADBAND_PIX              6                // 偏差死区；直线抖动大增大，直线不修正减小
	#define SERVO_OUT_LIMIT                 20.0f            // 软件舵角限幅；转不过弯增大，机械干涉风险减小
	#define RIGHT_TURN_STEER_MUL            1.20f            // 右转打角补偿倍数；右大弯打角不足则增大，右转过冲则减小
	#define LEFT_TURN_STEER_MUL             1.08f            // 左转打角补偿倍数；通常保持 1.00

	#define SERVO_RATE_STRAIGHT             0.40f            // 直道每周期最大打角变化；直线修正慢增大，直线抽动减小
	#define SERVO_RATE_ENTRY                4.60f            // 入弯打角速率；入弯晚增大，入弯过猛减小
	#define SERVO_RATE_APEX                 5.60f            // 弯中打角速率；大弯贴外线增大，弯中抖动减小
	#define SERVO_RATE_EXIT                 3.20f            // 出弯打角速率；S弯二弯响应慢增大
	#define SERVO_RATE_RECOVERY             5.20f            // 救车打角速率；冲线救不回增大，抽动过猛减小
	#define SERVO_SOFT_DEADBAND_PIX         3.2f             // 软死区像素；小偏差抖动大增大，直线不回正减小
	#define STRAIGHT_SMALL_ERR_TH            5.0f             // 直道小误差阈值；小范围仍抖动增大
	#define STRAIGHT_SMALL_PREVIEW_TH        2.2f             // 直道小误差前瞻阈值；噪声触发多减小
	#define SERVO_RATE_STRAIGHT_SMALL        0.24f            // 直道小误差区速率；直线抽动大减小
	#define STEER_GAIN_STRAIGHT_SMALL_MUL    0.42f            // 直道小误差区比例；轻微偏移修正过猛减小
	#define STEER_GYRO_STRAIGHT_SMALL_MUL    1.80f            // 直道小误差区陀螺阻尼；来回摆动大增大
	#define STEER_FF_STRAIGHT_SMALL_MUL      0.02f            // 直道小误差区前馈；直线噪声打角大减小
	#define STEER_RAW_LPF_ALPHA_STRAIGHT     0.14f            // 直道转向输出低通；直线抖动大减小
	#define STEER_RAW_LPF_ALPHA_CURVE        0.58f            // 弯道转向输出低通；入弯迟滞大增大

	// 状态增益调度（在 servo_kp / servo_kd / servo_kff 基础上乘以下列倍数）
	#define STEER_GAIN_STRAIGHT_MUL         0.60f            // 直道比例倍数；直道抖动大减小
	#define STEER_GAIN_ENTRY_MUL            1.15f            // 入弯比例倍数；入弯晚增大
	#define STEER_GAIN_APEX_MUL             1.35f            // 弯中比例倍数；贴外线增大，摆动大减小
	#define STEER_GAIN_EXIT_MUL             1.30f            // 出弯比例倍数；出弯外抛增大，出弯抖动减小
	#define STEER_GAIN_RECOVERY_MUL         1.15f            // 救车比例倍数；救车不够增大

	#define STEER_GYRO_STRAIGHT_MUL         1.45f            // 直道陀螺阻尼倍数；直线摆动大增大
	#define STEER_GYRO_ENTRY_MUL            1.05f            // 入弯陀螺阻尼倍数
	#define STEER_GYRO_APEX_MUL             1.00f            // 弯中陀螺阻尼倍数；弯中迟钝可减小
	#define STEER_GYRO_EXIT_MUL             1.15f            // 出弯陀螺阻尼倍数；出弯来回晃增大
	#define STEER_GYRO_RECOVERY_MUL         1.08f            // 救车陀螺阻尼倍数

	#define STEER_FF_STRAIGHT_MUL           0.20f            // 直道前馈倍数；直道误打角可减小
	#define STEER_FF_ENTRY_MUL              1.14f            // 入弯前馈倍数；入弯慢可增大
	#define STEER_FF_APEX_MUL               1.35f            // 弯中前馈倍数；大弯贴外线可增大
	#define STEER_FF_EXIT_MUL               0.90f            // 出弯前馈倍数；出弯抖动可减小
	#define STEER_FF_RECOVERY_MUL           1.05f            // 救车前馈倍数
	#define PREVIEW_FF_LIMIT                7.8f             // 前馈限幅；弯道不够转增大，转向冲击大减小
	#define CURVE_OUTSIDE_PREVIEW_TH        3.4f             // 外漂判定前瞻阈值；外漂误判多可增大，抓外线不回可减小
	#define CURVE_CENTER_ERR_TH             2.4f             // 外漂判定偏差阈值；外漂回中慢可减小，直线误触发可增大
	#define CURVE_OUTSIDE_FF_SUPPRESS       0.12f            // 外漂时前馈抑制系数；仍贴外线可减小，转向变钝可增大
	#define CURVE_OUTSIDE_PULL_K            0.36f            // 外漂回中拉力系数；外漂拉不回增大，回中抽动减小
	#define CURVE_OUTSIDE_RATE_MUL          1.55f            // 外漂时舵机速率放大；回中慢增大，摆动大减小
	#define CURVE_OUTSIDE_SPEED_KEEP        0.82f            // 外漂时速度保持；外漂严重减小，出弯拖慢增大

	#define REVERSE_TURN_PREVIEW_TH         2.6f             // S弯换向判定阈值；二弯进不去可减小，误触发可增大
	#define REVERSE_TURN_HOLD_CYCLES        12               // S弯换向辅助持续周期；换向不及时可增大
	#define REVERSE_TURN_RATE_MUL           1.25f            // S弯换向时打角速率倍数；换向慢增大，抽动大减小
	#define REVERSE_TURN_FF_MUL             1.10f            // S弯换向时前馈倍数；二弯贴外线增大，过冲减小
	#define REVERSE_TURN_SPEED_KEEP         0.52f            // S弯换向短时速度保持；飞出减小，掉速过多增大

	#define STRAIGHT_SMALL_GYRO_TH  18.0f

	// ------------------------- 状态机阈值 -------------------------
	#define STATE_STRAIGHT_PREVIEW_TH       3.0f             // 直道判定前瞻阈值；太敏感增大，直道识别慢减小
	#define STATE_STRAIGHT_OFFSET_TH        2.4f             // 直道判定偏差阈值；直道误判弯道增大
	#define STATE_STRAIGHT_GYRO_TH          40.0f            // 直道判定角速度阈值；车身晃动仍判直道则减小

	#define STATE_ENTRY_PREVIEW_TH          3.1f             // 入弯判定前瞻阈值；入弯晚减小，误触发增大
	#define STATE_ENTRY_DPREVIEW_TH         0.24f            // 入弯变化率阈值；长直冲弯减小，误触发增大
	#define STATE_STRAIGHT_PREVIEW_KEEP_TH  3.4f             // 已在直道时的保持阈值；直道误切弯增大
	#define STATE_STRAIGHT_OFFSET_KEEP_TH   3.0f             // 已在直道时的偏差保持阈值
	#define STATE_STRAIGHT_GYRO_KEEP_TH     55.0f            // 已在直道时的角速保持阈值
	#define STATE_STRAIGHT_PREVIEW_BACK_TH  2.6f             // 从弯道回直道阈值；回直道太慢增大
	#define STATE_STRAIGHT_OFFSET_BACK_TH   2.0f             // 从弯道回直道偏差阈值
	#define STATE_STRAIGHT_GYRO_BACK_TH     35.0f            // 从弯道回直道角速阈值
	#define STATE_ENTRY_PREVIEW_ON_TH       3.5f             // 从直道进入弯道阈值；直线误入弯增大
	#define STATE_ENTRY_DPREVIEW_ON_TH      0.30f            // 从直道进入弯道变化率阈值
	#define STATE_ENTRY_PREVIEW_OFF_TH      3.0f             // 非直道下保持入弯判定阈值；S弯衔接慢减小
	#define STATE_ENTRY_DPREVIEW_OFF_TH     0.20f            // 非直道下保持入弯变化率阈值
	#define STATE_APEX_PREVIEW_TH           5.8f             // 弯心判定前瞻阈值；切到弯中心太晚减小

	#define STATE_EXIT_PREVIEW_TH           4.0f             // 出弯判定前瞻阈值；出弯慢增大，过早出弯减小
	#define STATE_EXIT_OFFSET_TH            3.2f             // 出弯判定偏差阈值；出弯回正慢增大，出弯晃动减小

	#define STATE_RECOVERY_OFFSET_TH        17.0f            // 救车偏差阈值；救车太晚减小，误触发增大

	#define STATE_ENTRY_HOLD_CYCLES         3                // 入弯最短保持周期
	#define STATE_APEX_HOLD_CYCLES          6                // 弯心最短保持周期
	#define STATE_EXIT_HOLD_CYCLES          5                // 出弯最短保持周期
	#define STATE_RECOVERY_HOLD_CYCLES      8                // 救车最短保持周期

	// 长直后额外刹车：抑制“长直冲弯”
	#define LONG_STRAIGHT_COUNT_TH          12               // 判定长直线周期数；触发太频繁增大
	#define LONG_STRAIGHT_BRAKE_CYCLES      24               // 入弯额外刹车持续周期；冲弯严重增大
	#define LONG_STRAIGHT_ENTRY_KEEP        0.34f            // 长直入弯额外速度保持；冲弯减小，掉速多增大
	#define HARD_ENTRY_PREVIEW_RAW_TH       3.0f             // 直线末端硬入弯前瞻阈值；冲弯减小，误触发增大
	#define HARD_ENTRY_DPREVIEW_RAW_TH      0.60f            // 直线末端硬入弯变化率阈值；冲弯减小，误触发增大
	#define HARD_ENTRY_BRAKE_CYCLES         24               // 硬入弯重刹持续周期；冲弯严重增大
	#define HARD_ENTRY_BRAKE_KEEP           0.28f            // 硬入弯重刹速度保持；冲弯减小，掉速大增大

	// ------------------------- 速度规划 -------------------------
	#define SPEED_KEEP_ENTRY                0.60f            // 入弯状态速度保持；入弯冲出减小
	#define SPEED_KEEP_APEX_BASE            0.50f            // 弯中状态基础速度保持；弯中不稳减小
	#define SPEED_KEEP_EXIT                 0.50f            // 出弯状态速度保持；连续弯飞出减小
	#define SPEED_KEEP_RECOVERY             0.50f            // 救车状态速度保持；救车不住减小

	#define SPEED_CURVE_PREVIEW_SCALE       7.0f             // 前瞻归一化尺度；弯道减速太晚减小，太早增大
	#define SPEED_CURVE_K                   3.20f            // 曲率降速强度；弯道过快不稳增大
	#define SPEED_CURVE_MIN                 0.60f            // 曲率降速下限；弯中太慢增大，冲弯减小

	// 目标速度斜坡（抑制速度突变导致车身不稳）
	#define TARGET_STEP_UP_STRAIGHT         8.0f             // 直道每周期最大升速脉冲；突加速减小
	#define TARGET_STEP_UP_TURN             1.5f             // 弯道每周期最大升速脉冲；弯中加速明显减小
	#define TARGET_STEP_DOWN                220.0f           // 每周期最大降速脉冲；减速不及时增大
	#define TARGET_SCALE_MIN                0.30f            // 相对基础速度最小比例；过慢可增大
	#define TARGET_SCALE_MAX                1.15f            // 相对基础速度最大比例；过猛加速可减小

	// ------------------------- 直道加速（单逻辑） -------------------------
	#define STRAIGHT_ACCEL_ENABLE           0                // 0关闭先调稳，1开启再提尾速
	#define STRAIGHT_SPEED_BOOST            1.03f            // 直道加速倍率；加速突兀减小
	#define STRAIGHT_BOOST_PREVIEW_TH       2.2f             // 直道加速前瞻阈值；弯前误加速减小
	#define STRAIGHT_BOOST_OFFSET_TH        1.2f             // 直道加速偏差阈值；车身未正还加速则减小
	#define STRAIGHT_BOOST_GYRO_TH          32.0f            // 直道加速角速度阈值；姿态不稳还加速则减小
	#define STRAIGHT_BOOST_DPREVIEW_TH      0.25f            // 直道加速前瞻变化率阈值；长直冲弯则减小
	#define STRAIGHT_BOOST_STABLE_CYCLES    12               // 稳定周期门槛；触发太频繁可增大

	// ------------------------- 后轮差速（温和） -------------------------
	#define DIFF_K_STRAIGHT                 0.00f            // 直道差速系数；直道建议保持 0
	#define DIFF_K_ENTRY                    0.70f            // 入弯差速系数；入弯转向不足可增大
	#define DIFF_K_APEX                     0.98f            // 弯中差速系数；贴外线可增大，后轮抢方向减小
	#define DIFF_K_EXIT                     0.65f            // 出弯差速系数；出弯指向慢可增大
	#define DIFF_K_RECOVERY                 1.05f            // 救车差速系数；救车不足可增大
	#define DIFF_OUTSIDE_BOOST_MUL          1.10f            // 外漂时差速附加倍数；外漂难回中增大，后轮抢方向减小
	#define DIFF_FILTER_ALPHA               0.25f            // 差速低通；后轮抖动大减小

	#define DIFF_MAX_CAP_RATIO              0.24f            // 差速上限占比；后轮主导姿态减小
	#define DIFF_MIN_CAP                    16               // 差速下限保护；低速差速无感可增大

	#define DIFF_INNER_MIN_RATIO            0.26f            // 弯中内轮最小速度比例；内轮拖死可增大
	#define DIFF_INNER_MIN_STEER_TH         12.0f            // 启用内轮最小速度的舵角阈值
	#define DIFF_INNER_MIN_PREVIEW_TH       9.5f             // 启用内轮最小速度的前瞻阈值

	volatile uint8 print_flag = 0;

	// 手动可调主增益（先调这三个）
	float servo_kp = 0.36f;                                   // 比例增益；入弯慢增大，摆动大减小
	float servo_kd = 0.15f;                                   // 陀螺阻尼；摆动大增大，转向迟滞减小
	float servo_kff = 0.25f;                                  // 前瞻前馈；贴外线增大，弯前过冲减小

	// 基础速度指令（0~90）
	int target_speed_base = 20;                               // 先稳后快；稳态后再按 +1 提速

	PID servo_pid = {0};

	typedef enum
	{
			TRACK_STRAIGHT = 0,
			TRACK_ENTRY    = 1,
			TRACK_APEX     = 2,
			TRACK_EXIT     = 3,
			TRACK_RECOVERY = 4
	} track_state_e;

	static float target_mid_filtered = 94.0f; // 动态目标点的平滑滤波变量

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

			static float preview_raw_last = 0.0f;
			static float servo_out_last = 0.0f;
			static float steer_raw_filtered = 0.0f;
			static float target_pulses_ramped = 0.0f;
			static float diff_filtered = 0.0f;

			static uint8 lpf_init = 0;
			static uint8 track_state = TRACK_STRAIGHT;
			static uint8 state_hold = 0;
			static uint16 straight_counter = 0;
			static uint8 long_straight_brake_cnt = 0;
		
			static float rate_limit_smooth = SERVO_RATE_STRAIGHT;

			static uint8 hard_entry_brake_cnt = 0;
			static uint8 reverse_turn_cnt = 0;
			int near_y;
			int far_y;
			int near_mid;
			int far_mid;

			int offset_raw;
			int offset;

			float preview_raw;
			float preview_delta;

			float preview_prev;
			float preview_raw_delta;
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

			uint8 hard_entry_cond;
			uint8 outside_drift;
			uint8 prev_state;

			float kp_mul;
			float kd_mul;
			float kff_mul;
			float rate_limit;
			float steer_raw_alpha;
			float offset_ctrl;
			float abs_offset_ctrl;
			uint8 small_err_straight;

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
			
					// --- 新增：动态目标中点逻辑 (切内线核心) ---
			int target_mid = MID_W; // 默认目标是屏幕中心 94
		
			int target_mid_raw = MID_W; // 瞬时原始目标点
			int shift_val = 15;         // 切线偏移量
			
			// 1) 计算远近前瞻
			near_y = PREVIEW_Y_NEAR;
			if (near_y >= MT9V03X_H) near_y = MT9V03X_H - 1;
			if (near_y <= search_end_line) near_y = search_end_line + 1;

			far_y = PREVIEW_Y_FAR;
			if (far_y <= search_end_line) far_y = search_end_line + 1;
			if (far_y >= MT9V03X_H) far_y = MT9V03X_H - 1;

			near_mid = mid_line_list[near_y];
			far_mid = mid_line_list[far_y];

	//    preview_raw = (float)(far_mid - near_mid);            // 有符号前瞻（带方向）
	//    offset_raw = (int)final_mid_line - MID_W;             // 有符号横向偏差
			preview_raw = (float)(far_mid - near_mid);            // 有符号前瞻（带方向）

			// 只有在处于入弯和弯心状态时，才允许偏移目标点
	//		if (track_state == TRACK_ENTRY || track_state == TRACK_APEX) {
	//				if (preview_filtered > 5.0f) { 
	//						target_mid_raw = MID_W + shift_val; 
	//				} 
	//				else if (preview_filtered < -5.0f) {
	//						target_mid_raw = MID_W - shift_val;
	//				}
	//		}

			if (track_state == TRACK_ENTRY || track_state == TRACK_APEX) {
					if (preview_raw > 5.0f) { 
							target_mid_raw = MID_W + shift_val; 
					} 
					else if (preview_raw < -5.0f) {
							target_mid_raw = MID_W - shift_val;
					}
			}
			
				if (track_state == TRACK_STRAIGHT &&
					abs_f(preview_raw) < 2.0f &&
					abs_f(gyro_z_filtered) < 20.0f)
			{
					target_mid_raw = MID_W;
					target_mid_filtered = target_mid_filtered * 0.90f + (float)MID_W * 0.10f;
			}
			else
			{
					target_mid_filtered = target_mid_filtered * 0.80f + (float)target_mid_raw * 0.20f;
			}

	// 【核心修复：滑动低通滤波】
	// 让目标点像被弹簧拉着一样，柔和地移动到 109，出弯时再柔和地回到 94
	// 0.80f 是惯性(保留上一帧的比重)，0.20f 是跟随率(吸收新目标的比重)。
	// 如果出弯还是轻微晃，把 0.80 改成 0.85 (更平滑)；如果嫌入弯切得慢，改 0.70 (更灵敏)。
			//target_mid_filtered = target_mid_filtered * 0.80f + (float)target_mid_raw * 0.20f;

			offset_raw = (int)final_mid_line - (int)target_mid_filtered; // 基于平滑后的目标计算偏差

			if (!lpf_init)
			{
					offset_filtered = (float)offset_raw;
					preview_filtered = preview_raw;
					preview_last = preview_raw;
					preview_raw_last = preview_raw;
					target_pulses_ramped = 0.0f;
					steer_raw_filtered = 0.0f;
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

			preview_prev = preview_last;
			preview_delta = preview_filtered - preview_last;
			preview_last = preview_filtered;
			preview_raw_delta = preview_raw - preview_raw_last;
			preview_raw_last = preview_raw;

			// 3) 获取陀螺仪
			imu660ra_get_gyro();
			gyro_z_actual = imu660ra_gyro_transition(imu660ra_gyro_z);
			gyro_z_filtered = (1.0f - GYRO_LPF_ALPHA) * gyro_z_filtered + GYRO_LPF_ALPHA * gyro_z_actual;

					// 4) 滤波值转整数偏差，并对转向量使用软死区（避免 0/非0 阶跃）
			if (offset_filtered >= 0.0f)
			{
					offset = (int)(offset_filtered + 0.5f);
			}
			else
			{
					offset = (int)(offset_filtered - 0.5f);
			}

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

			abs_preview = abs_f(preview_filtered);
			abs_offset = abs_f((float)offset);
			abs_offset_ctrl = abs_f(offset_ctrl);
			abs_gyro = abs_f(gyro_z_filtered);

			// 直线末端硬入弯：用于“长直后大弯/S弯进不去”
			hard_entry_cond = (straight_counter >= (LONG_STRAIGHT_COUNT_TH / 2)) &&
												((abs_f(preview_raw) >= HARD_ENTRY_PREVIEW_RAW_TH) ||
												 (abs_f(preview_raw_delta) >= HARD_ENTRY_DPREVIEW_RAW_TH));

					// S弯换向检测：用于“连续弯第二弯响应慢”
			if ((abs_f(preview_prev) >= REVERSE_TURN_PREVIEW_TH) &&
					(abs_preview >= REVERSE_TURN_PREVIEW_TH) &&
					(preview_prev * preview_filtered < 0.0f))
			{
					reverse_turn_cnt = REVERSE_TURN_HOLD_CYCLES;
			}
			else if (reverse_turn_cnt > 0)
			{
					reverse_turn_cnt--;
			}

			// 5) 状态判定条件（加入滞回，抑制直线/入弯来回切换）
			if (track_state == TRACK_STRAIGHT)
			{
					straight_cond = (abs_preview <= STATE_STRAIGHT_PREVIEW_KEEP_TH) &&
													(abs_offset <= STATE_STRAIGHT_OFFSET_KEEP_TH) &&
													(abs_gyro <= STATE_STRAIGHT_GYRO_KEEP_TH);

					entry_cond = (abs_preview >= STATE_ENTRY_PREVIEW_ON_TH) ||
											 (abs_f(preview_delta) >= STATE_ENTRY_DPREVIEW_ON_TH);
			}
			else
			{
					straight_cond = (abs_preview <= STATE_STRAIGHT_PREVIEW_BACK_TH) &&
													(abs_offset <= STATE_STRAIGHT_OFFSET_BACK_TH) &&
													(abs_gyro <= STATE_STRAIGHT_GYRO_BACK_TH);

					entry_cond = (abs_preview >= STATE_ENTRY_PREVIEW_OFF_TH) ||
											 (abs_f(preview_delta) >= STATE_ENTRY_DPREVIEW_OFF_TH);
			}

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

			if (state_hold > 0) state_hold--;

			switch (track_state)
			{
			case TRACK_STRAIGHT:
					if (hard_entry_cond)
					{
							track_state = TRACK_ENTRY;
							state_hold = STATE_ENTRY_HOLD_CYCLES;
							hard_entry_brake_cnt = HARD_ENTRY_BRAKE_CYCLES;
					}
					else if (recovery_cond)
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
					if ((reverse_turn_cnt > 0) && entry_cond)
					{
							track_state = TRACK_ENTRY;
							state_hold = STATE_ENTRY_HOLD_CYCLES;
					}
					else if (recovery_cond)
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
					else if (((state_hold == 0) || (reverse_turn_cnt > 0)) && entry_cond)
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

			// 外漂检测：偏差方向与弯向一致时，判定为“正在向外侧漂”
			outside_drift = 0;
			if ((track_state != TRACK_STRAIGHT) &&
					(abs_preview >= CURVE_OUTSIDE_PREVIEW_TH) &&
					(abs_offset >= CURVE_CENTER_ERR_TH) &&
					(((offset > 0) && (preview_filtered > 0.0f)) ||
					 ((offset < 0) && (preview_filtered < 0.0f))))
			{
					outside_drift = 1;
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

	//    small_err_straight = (track_state == TRACK_STRAIGHT) &&
	//                         (abs_offset_ctrl <= STRAIGHT_SMALL_ERR_TH) &&
	//                         (abs_preview <= STRAIGHT_SMALL_PREVIEW_TH);
			
			

			small_err_straight = (track_state == TRACK_STRAIGHT) &&
													(abs_offset_ctrl <= STRAIGHT_SMALL_ERR_TH) &&
													(abs_preview <= STRAIGHT_SMALL_PREVIEW_TH) &&
													(abs_gyro <= STRAIGHT_SMALL_GYRO_TH);
			
			if (small_err_straight)
			{
					kp_mul = STEER_GAIN_STRAIGHT_SMALL_MUL;
					kd_mul = STEER_GYRO_STRAIGHT_SMALL_MUL;
					kff_mul = STEER_FF_STRAIGHT_SMALL_MUL;
					rate_limit = SERVO_RATE_STRAIGHT_SMALL;
			}

			if (reverse_turn_cnt > 0)
			{
					rate_limit *= REVERSE_TURN_RATE_MUL;
					kff_mul *= REVERSE_TURN_FF_MUL;
					if (track_state != TRACK_STRAIGHT)
					{
							kp_mul *= 1.10f;
					}
			}
			if (outside_drift)
			{
					rate_limit *= CURVE_OUTSIDE_RATE_MUL;
			}

			// 8) 连续舵机控制：反馈 + 前馈 + 速率限制
			ff_term = servo_kff * kff_mul * preview_filtered;
			if (outside_drift)
			{
					ff_term *= CURVE_OUTSIDE_FF_SUPPRESS;
			}
			ff_term = clamp_f(ff_term, -PREVIEW_FF_LIMIT, PREVIEW_FF_LIMIT);

			steer_raw = (-(servo_kp * kp_mul) * offset_ctrl) - ((servo_kd * kd_mul) * gyro_z_filtered) + ff_term;
			if (track_state != TRACK_STRAIGHT)
			{
					if (steer_raw >= 0.0f) steer_raw *= RIGHT_TURN_STEER_MUL;
					else steer_raw *= LEFT_TURN_STEER_MUL;
			}
					if (outside_drift)
			{
					// 外漂时额外施加回中拉力，减少持续贴外线
					steer_raw += (-CURVE_OUTSIDE_PULL_K * offset_ctrl);
			}

			steer_raw_alpha = (track_state == TRACK_STRAIGHT) ? STEER_RAW_LPF_ALPHA_STRAIGHT : STEER_RAW_LPF_ALPHA_CURVE;
			steer_raw_filtered = steer_raw_filtered * (1.0f - steer_raw_alpha) + steer_raw * steer_raw_alpha;
			steer_raw = steer_raw_filtered;
			steer_raw = clamp_f(steer_raw, -SERVO_OUT_LIMIT, SERVO_OUT_LIMIT);

			delta_out = steer_raw - servo_out_last;
			
			rate_limit_smooth = rate_limit_smooth * 0.75f + rate_limit * 0.25f;
			rate_limit = rate_limit_smooth;
			
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

			if (hard_entry_brake_cnt > 0)
			{
					speed_keep_total *= HARD_ENTRY_BRAKE_KEEP;
					hard_entry_brake_cnt--;
			}

			if ((reverse_turn_cnt > 0) && (track_state != TRACK_STRAIGHT))
			{
					speed_keep_total *= REVERSE_TURN_SPEED_KEEP;
			}
			if (outside_drift)
			{
					// 外漂阶段优先回中，先轻降速再给姿态修正余量
					speed_keep_total *= CURVE_OUTSIDE_SPEED_KEEP;
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

			base_pulses = (int)((float)(((long)target_speed_base * MAX_SPEED_PULSES) / 90) * BASE_SPEED_MATCH_GAIN);
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
			if (outside_drift)
			{
					diff_k *= DIFF_OUTSIDE_BOOST_MUL;
			}

			diff_cmd = steer_out * diff_k;
			if (steer_out > 0.0f) 
			{
					// 赋予右转更强的差速力道！数值可调。
					// 1.20f 表示右转时，后轮辅助转向的力道比左转强 20%
					diff_cmd *= 1.20f; 
			}
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

	//    // 急弯内轮最低速度保护，防止内轮被压死后车身僵硬
	//    if ((track_state != TRACK_STRAIGHT) &&
	//        (abs_steer >= DIFF_INNER_MIN_STEER_TH) &&
	//        (abs_preview >= DIFF_INNER_MIN_PREVIEW_TH))
	//    {
	//        inner_min_pulses = (int)((float)base_pulses * DIFF_INNER_MIN_RATIO);
	//        if (inner_min_pulses < 1) inner_min_pulses = 1;

	//        if (left_target_pulses < right_target_pulses)
	//        {
	//            if (left_target_pulses < inner_min_pulses) left_target_pulses = inner_min_pulses;
	//        }
	//        else
	//        {
	//            if (right_target_pulses < inner_min_pulses) right_target_pulses = inner_min_pulses;
	//        }
	//    }

			// 11) 编码器 + 电机 PID
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