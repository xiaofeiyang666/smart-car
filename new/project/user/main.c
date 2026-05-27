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
    static char debug_str[96];
    uint8 uart_dbg_div = 0;

    uint8 vofa_tflag = 0;
    uint8 vofa_tstage = 0;
    uint8 vofa_tx = 0;
    uint8 vofa_ty = 0;
    uint8 vofa_tl = 0;
    uint8 vofa_tr = 0;
    uint8 vofa_tt = 0;
    uint8 vofa_tb = 0;
    int16 vofa_terr = 0;
    uint8 vofa_taim = 0;
    uint8 vofa_tconf = 0;
		uint8 vofa_sstate = 0;
		uint8 vofa_laser = 0;
		int16 vofa_sangle = 0;
		int16 vofa_serr = 0;
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
                vofa_tflag = target_flag;
                vofa_tstage = target_stage;
                vofa_tx = target_x;
                vofa_ty = target_y;
                vofa_tl = target_left;
                vofa_tr = target_right;
                vofa_tt = target_top;
                vofa_tb = target_bottom;
                vofa_terr = target_servo2_input;
                vofa_taim = target_aim_ok;
                vofa_tconf = target_confidence;
								vofa_sstate = shoot_state;
								vofa_laser = shoot_laser_on_flag;
								vofa_sangle = shoot_servo2_angle_x10;
								vofa_serr = shoot_target_error;
                EA = 1;

                sprintf(debug_str, "%u,%u,%u,%u,%u,%u,%u,%u,%d,%u,%u,%u,%u,%d,%d\r\n",
												vofa_tflag,
												vofa_tstage,
												vofa_tx,
												vofa_ty,
												vofa_tl,
												vofa_tr,
												vofa_tt,
												vofa_tb,
												vofa_terr,
												vofa_taim,
												vofa_tconf,
												vofa_sstate,
												vofa_laser,
												vofa_sangle,
												vofa_serr);
								wireless_uart_send_string(debug_str);
            }
#endif
        }
    }
}
