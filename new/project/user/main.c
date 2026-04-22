#include "zf_common_headfile.h"
#include "camera.h"
#include "control.h"
#include "servo.h"
#include "motor.h"
#include <stdio.h>
#include "encoder.h"
#include "pid.h"
#include "brushless.h"
#include <string.h>
#define WIFI_SSID_TEST "Car"
#define WIFI_PASSWORD_TEST "431431431"

//void main(void)
//{
//    char debug_str[64];
//    clock_init(SYSTEM_CLOCK_96M);
//    debug_init();

//    //初始化
//    motor_init();
//    encoder_init();
//    servo_init();   // 初始化舵机 (中值归位)
//    control_init(); // 初始化 PID 参数
//    fan_init();
//		ips200_init();

//    // 开启定时器中断
//    pit_ms_init(TIM0_PIT, 20, control_loop);
//    EA = 1;

//    // 通信和摄像头初始化
////    while (wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
////    {
////        system_delay_ms(100);
////    }

////    if (1 != WIFI_SPI_AUTO_CONNECT)
////    {
////        while (wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT))
////        {
////            system_delay_ms(100);
////        }
////    }

//    while (mt9v03x_init())
//    {
//        system_delay_ms(100);
//    }

//    if (wireless_uart_init())
//    {
//    }
//    //wireless_uart_send_string("System Init OK!\r\n");

//    while (1)
//    {
//        camara_task();

//        if (print_flag)
//        {
//            print_flag = 0;
//            // 打印：设定速度、当前速度、PID算出的PWM占空比
//            sprintf(debug_str, "%d,%d,%d\n", target_speed_base, left_speed, right_speed);
//            wireless_uart_send_string(debug_str);
//        }
//				//set_motor_speed(10,10);
//    }
//}

void main(void)
{
    char debug_str[64];
    clock_init(SYSTEM_CLOCK_96M);
    debug_init();

    // 2. 依次初始化各个硬件模块
    motor_init();
    encoder_init();
    servo_init();   
    control_init(); 
    fan_init();
		imu660ra_init();
		//wireless_uart_init(); //2026.4.18注释这里
    
    camara_init(); 

    // 开启定时器中断 (20ms周期)
    pit_ms_init(TIM0_PIT, 20, control_loop);
    EA = 1;

    while (1)
    {
        // 3. 循环执行图像处理任务
        camara_task();
				set_fan_speed(800,800);
        if (print_flag)
        {
            print_flag = 0;
						//sprintf(debug_str, "%d,%d,%d,%d\n", left_target_pulses,right_target_pulses, left_speed, right_speed);//2026.4.18注释这里
            //wireless_uart_send_string(debug_str);//2026.4.18注释这里
        }
    }
}
