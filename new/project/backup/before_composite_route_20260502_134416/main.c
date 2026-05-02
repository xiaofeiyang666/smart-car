#include "zf_common_headfile.h"
#include "camera.h"
#include "control.h"
#include "servo.h"
#include "motor.h"
#include <stdio.h>
#include "encoder.h"
#include "pid.h"
#include "brushless.h"

// 串口调试开关：0=关闭（提升性能），1=开启（输出fps/off/pre）
#define UART_DEBUG_PRINT_ENABLE    1
// 开启调试时的分频：每N次print_flag输出一次，减小串口阻塞
#define UART_DEBUG_PRINT_DIV       5

void main(void)
{
#if UART_DEBUG_PRINT_ENABLE
    char debug_str[128];
    uint8 uart_dbg_div = 0;
#endif

    clock_init(SYSTEM_CLOCK_96M);
    debug_init();

    // 硬件初始化
    motor_init();
    encoder_init();
    servo_init();
    control_init();
    fan_init();
    set_fan_speed(800, 800);
    imu660ra_init();

#if UART_DEBUG_PRINT_ENABLE
    // 开启无线串口，便于实时查看fps和偏差
    wireless_uart_init();
#endif

    // 摄像头初始化
    camara_init();

    // 控制环20ms
    pit_ms_init(TIM0_PIT, 20, control_loop);
    // 1ms节拍用于统计current_fps
    pit_ms_init(TIM1_PIT, 1, my_fps_timer_callback);

    EA = 1;

    while (1)
    {
        camara_task();

        // print_flag大约100ms触发一次
        if (print_flag)
        {
            print_flag = 0;

#if UART_DEBUG_PRINT_ENABLE
            uart_dbg_div++;
            if (uart_dbg_div >= UART_DEBUG_PRINT_DIV)
            {
                uart_dbg_div = 0;
                sprintf(debug_str, "fps=%u,off=%d,pre=%d,far=%d,conf=%u,sc=%d,st=%d,lk=%u,s=%u,gz=%d\r\n", current_fps, camera_bias_raw, camera_preview_raw, camera_preview_far_raw, camera_confidence, control_debug_curve_x100, control_debug_steer_out, control_debug_straight_lock, control_debug_s_transition, control_debug_gyro_z);
                wireless_uart_send_string(debug_str);
            }
#endif
        }
    }
}


