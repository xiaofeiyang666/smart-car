#include "zf_common_headfile.h"
#include "camera.h"
#include "control.h"
#include "servo.h"
#include "motor.h"
#include <stdio.h>
#include "encoder.h"
#include "pid.h"
#include "brushless.h"
#include "Target_Ring.h"
#include "Shoot.h"

#define UART_DEBUG_PRINT_ENABLE 1
#ifndef UART_DEBUG_PRINT_DIV
#define UART_DEBUG_PRINT_DIV 4
#endif

void main(void)
{
#if UART_DEBUG_PRINT_ENABLE
    static char debug_str[224];
    uint8 uart_dbg_div = 0;

    uint8 vofa_route = 0;
    int16 vofa_ring_dir = 0;
    uint8 vofa_cross = 0;
    uint8 vofa_premeet = 0;
    uint8 vofa_candidate = 0;
    uint8 vofa_conf = 0;
    uint8 vofa_valid = 0;
    uint8 vofa_lost_l = 0;
    uint8 vofa_lost_r = 0;
    uint8 vofa_ring_midpoint = 0;
    uint8 vofa_ring_mid_under = 0;
    uint8 vofa_ring_l115 = 0;
    uint8 vofa_ring_l85 = 0;
    uint8 vofa_ring_l55 = 0;
    uint8 vofa_left80 = 0;
    uint8 vofa_right80 = 0;
    uint8 vofa_mid80 = 0;
    uint8 vofa_width80 = 0;
    uint8 vofa_left60 = 0;
    uint8 vofa_right60 = 0;
    uint8 vofa_mid60 = 0;
    uint8 vofa_width60 = 0;
    int16 vofa_preview = 0;
    int16 vofa_far = 0;
    int16 vofa_curve = 0;
    int16 vofa_ff = 0;
    int16 vofa_steer = 0;
    int16 vofa_lt = 0;
    int16 vofa_rt = 0;
    int16 vofa_ls = 0;
    int16 vofa_rs = 0;
    int16 vofa_gyro = 0;
    int16 vofa_speed_scale = 0;
    uint8 vofa_route_hold = 0;
    int16 vofa_left_pwm = 0;
    int16 vofa_right_pwm = 0;
    uint16 vofa_fps = 0;
#endif

    clock_init(SYSTEM_CLOCK_96M);
    debug_init();

    motor_init();
    encoder_init();
    servo_init();
    shoot_init();
    shoot_enable = 1;
    control_init();
    //    fan_init();
    //    set_fan_speed(800, 800);
    imu660ra_init();

#if UART_DEBUG_PRINT_ENABLE
    wireless_uart_init();
#endif

    camara_init();

    pit_ms_init(TIM0_PIT, 5, control_loop);
    pit_ms_init(TIM1_PIT, 1, my_fps_timer_callback);

    EA = 1;

    while (1)
    {
        camara_task();

        if (print_flag)
        {
            print_flag = 0;

#if UART_DEBUG_PRINT_ENABLE
            uart_dbg_div++;
            if (uart_dbg_div >= UART_DEBUG_PRINT_DIV)
            {
                uart_dbg_div = 0;

                EA = 0;
                vofa_route = camera_route_mode;
                vofa_ring_dir = camera_ring_dir;
                vofa_cross = camera_cross_state;
                vofa_premeet = camera_debug_ring_premeet;
                vofa_candidate = camera_debug_ring_candidate;
                vofa_conf = camera_confidence;
                vofa_valid = camera_valid_line_cnt;
                vofa_lost_l = camera_lost_left_cnt;
                vofa_lost_r = camera_lost_right_cnt;
                vofa_ring_midpoint = camera_debug_ring_midpoint;
                vofa_ring_mid_under = camera_debug_ring_mid_under;
                vofa_ring_l115 = camera_debug_ring_left115;
                vofa_ring_l85 = camera_debug_ring_left85;
                vofa_ring_l55 = camera_debug_ring_left55;
                vofa_left80 = camera_debug_left_80;
                vofa_right80 = camera_debug_right_80;
                vofa_mid80 = camera_debug_mid_80;
                vofa_width80 = camera_debug_width_80;
                vofa_left60 = camera_debug_left_60;
                vofa_right60 = camera_debug_right_60;
                vofa_mid60 = camera_debug_mid_60;
                vofa_width60 = camera_debug_width_60;
                vofa_preview = control_debug_preview_raw;
                vofa_far = control_debug_preview_far_raw;
                vofa_curve = control_debug_curve_raw;
                vofa_ff = control_debug_steer_ff_x100;
                vofa_steer = control_debug_steer_out_x100;
                vofa_lt = control_debug_left_target;
                vofa_rt = control_debug_right_target;
                vofa_ls = control_debug_left_speed;
                vofa_rs = control_debug_right_speed;
                vofa_gyro = imu_gyro_z_dps_x10;
                vofa_speed_scale = control_debug_speed_scale_x100;
                vofa_route_hold = control_debug_curve_exit_hold_cnt;
                vofa_left_pwm = control_debug_left_pwm;
                vofa_right_pwm = control_debug_right_pwm;
                vofa_fps = current_fps;
                EA = 1;

                sprintf(debug_str,
                        "%u,%d,%u,%u,%u,%u,%u,%u,",
                        vofa_route,
                        vofa_ring_dir,
                        vofa_cross,
                        vofa_premeet,
                        vofa_candidate,
                        vofa_conf,
                        vofa_valid,
                        vofa_lost_l);
                wireless_uart_send_string(debug_str);

                sprintf(debug_str,
                        "%u,%u,%u,%u,%u,%u,%u,%u,",
                        vofa_lost_r,
                        vofa_ring_midpoint,
                        vofa_ring_mid_under,
                        vofa_ring_l115,
                        vofa_ring_l85,
                        vofa_ring_l55,
                        vofa_left80,
                        vofa_right80);
                wireless_uart_send_string(debug_str);

                sprintf(debug_str,
                        "%u,%u,%u,%u,%d,%d,%d,%d,",
                        vofa_mid80,
                        vofa_width80,
                        vofa_left60,
                        vofa_right60,
                        vofa_preview,
                        vofa_far,
                        vofa_curve,
                        vofa_ff);
                wireless_uart_send_string(debug_str);

                sprintf(debug_str,
                        "%d,%d,%d,%d,%d,%d,%d,%d,%u\r\n",
                        vofa_steer,
                        vofa_lt,
                        vofa_rt,
                        vofa_ls,
                        vofa_rs,
                        vofa_left_pwm,
                        vofa_right_pwm,
                        vofa_speed_scale,
                        vofa_fps);
                wireless_uart_send_string(debug_str);
            }
#endif
        }
    }
}
