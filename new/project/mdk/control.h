#ifndef _CONTROL_H
#define _CONTROL_H

#include "zf_common_headfile.h"

/*
 * target_speed_base is encoder pulses per 5 ms control tick.
 * Motor PID output is still limited in pid_init() by maxOutput.
 */
extern int target_speed_base;
extern int target_speed_curve_min;
extern float servo_kp;
extern float servo_kp2;
extern float servo_kg;

void control_init(void);
void control_loop(void);

extern volatile uint8 print_flag;
extern volatile int16 imu_gyro_z_dps_x10;
extern volatile int16 control_debug_preview_raw;
extern volatile int16 control_debug_near_bias_raw;
extern volatile int16 control_debug_used_bias_x10;
extern volatile int16 control_debug_preview_filtered_x10;
extern volatile int16 control_debug_preview_far_raw;
extern volatile int16 control_debug_curve_raw;
extern volatile int16 control_debug_steer_p_x100;
extern volatile int16 control_debug_steer_kp2_x100;
extern volatile int16 control_debug_steer_ff_x100;
extern volatile int16 control_debug_steer_out_x100;
extern volatile int16 control_debug_left_target;
extern volatile int16 control_debug_right_target;
extern volatile int16 control_debug_left_speed;
extern volatile int16 control_debug_right_speed;
extern volatile int16 control_debug_diff_speed;
extern volatile int16 control_debug_left_pwm;
extern volatile int16 control_debug_right_pwm;
extern volatile uint8 control_debug_camera_confidence;
extern volatile uint8 control_debug_valid_line_cnt;
extern volatile uint8 control_debug_lost_left_cnt;
extern volatile uint8 control_debug_lost_right_cnt;
extern volatile uint8 control_debug_curve_exit_hold_cnt;
extern volatile int16 control_debug_speed_scale_x100;
extern volatile uint8 control_debug_route_mode;
extern volatile uint8 control_debug_cross_state;
extern volatile uint8 control_debug_cross_left_open_cnt;
extern volatile uint8 control_debug_cross_right_open_cnt;
extern volatile uint8 control_debug_cross_both_open_cnt;
extern volatile uint8 control_debug_left_control;
extern volatile uint8 control_debug_right_control;
extern volatile uint8 control_debug_mid_control;
extern volatile uint8 control_debug_ring_midpoint;
extern volatile uint8 control_debug_ring_mid_under;
extern volatile uint8 control_debug_ring_left115;
extern volatile uint8 control_debug_ring_left85;
extern volatile uint8 control_debug_ring_left55;

#endif
