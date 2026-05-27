#include "camera.h"
#include <string.h>
#include "Target_Ring.h"

#define CAMERA_DEBUG_DRAW_ENABLE   1

uint8 img_threshold = 120;
uint8 left_jidian = 1;
uint8 right_jidian = MT9V03X_W - 2;
uint8 left_line_list[MT9V03X_H];
uint8 right_line_list[MT9V03X_H];
uint8 mid_line_list[MT9V03X_H];
uint8 final_mid_line = MID_W;
volatile uint16 current_fps = 0;

int16 camera_bias_raw = 0;
int16 camera_preview_raw = 0;
int16 camera_preview_far_raw = 0;
int16 camera_curve_raw = 0;
uint8 camera_route_mode = 0;
uint8 camera_valid_line_cnt = 0;
uint8 camera_lost_left_cnt = 0;
uint8 camera_lost_right_cnt = 0;
uint8 camera_confidence = 0;
int8 camera_ring_dir = 0;
uint8 camera_cross_state = 0;
uint8 camera_cross_left_open_cnt = 0;
uint8 camera_cross_right_open_cnt = 0;
uint8 camera_cross_both_open_cnt = 0;
uint8 camera_debug_left_control = 0;
uint8 camera_debug_right_control = MT9V03X_W - 1;
uint8 camera_debug_mid_control = MID_W;
uint8 camera_debug_ring_midpoint = 0;
uint8 camera_debug_ring_mid_under = 0;
uint8 camera_debug_ring_left115 = 0;
uint8 camera_debug_ring_left85 = 0;
uint8 camera_debug_ring_left55 = 0;
uint8 camera_debug_left_80 = 0;
uint8 camera_debug_right_80 = 0;
uint8 camera_debug_mid_80 = MID_W;
uint8 camera_debug_width_80 = 0;

uint8 camera_debug_left_60 = 0;
uint8 camera_debug_right_60 = 0;
uint8 camera_debug_mid_60 = MID_W;
uint8 camera_debug_width_60 = 0;

static volatile uint16 fps_counter = 0;

static void camera_publish_vision(void)
{
    uint8 ea_state;

    memcpy(left_line_list, vision_left_control_line, sizeof(left_line_list));
    memcpy(right_line_list, vision_right_control_line, sizeof(right_line_list));
    memcpy(mid_line_list, vision_mid_line, sizeof(mid_line_list));

    ea_state = EA;
    EA = 0;

    left_jidian = vision_left_edge_line[MT9V03X_H - 1];
    right_jidian = vision_right_edge_line[MT9V03X_H - 1];
    final_mid_line = vision_final_mid;

    camera_bias_raw = vision_error;
    camera_preview_raw = vision_preview_error;
    camera_preview_far_raw = vision_far_error;
    camera_curve_raw = vision_curve_error;
    camera_route_mode = vision_ring_state;
    camera_valid_line_cnt = vision_valid_line_count;
    camera_lost_left_cnt = vision_lost_left_count;
    camera_lost_right_cnt = vision_lost_right_count;
    camera_confidence = vision_confidence;
    camera_ring_dir = vision_ring_dir;
    camera_cross_state = vision_cross_state;
    camera_cross_left_open_cnt = vision_cross_left_open_count;
    camera_cross_right_open_cnt = vision_cross_right_open_count;
    camera_cross_both_open_cnt = vision_cross_both_open_count;
    camera_debug_left_control = vision_debug_left_control;
    camera_debug_right_control = vision_debug_right_control;
    camera_debug_mid_control = vision_debug_mid_control;
    camera_debug_ring_midpoint = vision_debug_ring_midpoint;
    camera_debug_ring_mid_under = vision_debug_ring_mid_under;
    camera_debug_ring_left115 = vision_debug_ring_left115;
    camera_debug_ring_left85 = vision_debug_ring_left85;
    camera_debug_ring_left55 = vision_debug_ring_left55;
		camera_debug_left_80 = vision_debug_left_80;
		camera_debug_right_80 = vision_debug_right_80;
		camera_debug_mid_80 = vision_debug_mid_80;
		camera_debug_width_80 = vision_debug_width_80;

		camera_debug_left_60 = vision_debug_left_60;
		camera_debug_right_60 = vision_debug_right_60;
		camera_debug_mid_60 = vision_debug_mid_60;
		camera_debug_width_60 = vision_debug_width_60;

    EA = ea_state;
}

void my_fps_timer_callback(void)
{
    static uint16 time_ms = 0;

    time_ms++;
    if (time_ms >= 1000)
    {
        time_ms = 0;
        current_fps = fps_counter;
        fps_counter = 0;
    }
}

void camara_init(void)
{
    vision_init();

#if (IPS200_OR_WIFI == 0)
    ips200_init();
    ips200_show_string(0, 0, "mt9v03x init.");

    while (1)
    {
        system_delay_ms(80);
        if (mt9v03x_init())
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
        else
        {
            break;
        }
    }

    ips200_show_string(0, 16, "init success.");
#elif (IPS200_OR_WIFI == 1)
    wireless_uart_init();

    while (wifi_spi_init("Car", "431431431"))
    {
        system_delay_ms(100);
    }

    if (1 != WIFI_SPI_AUTO_CONNECT)
    {
        while (wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT))
        {
            system_delay_ms(100);
        }
    }

    while (mt9v03x_init())
    {
        system_delay_ms(80);
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X,
                                                 mt9v03x_image[0],
                                                 MT9V03X_W,
                                                 MT9V03X_H);
    seekfree_assistant_camera_boundary_config(X_BOUNDARY,
                                              MT9V03X_H,
                                              left_line_list,
                                              right_line_list,
                                              mid_line_list,
                                              NULL,
                                              NULL,
                                              NULL);
#else
#error "IPS200_OR_WIFI must be 0 or 1."
#endif
}

void camara_task(void)
{
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        vision_process(mt9v03x_image[0]);
				target_process(mt9v03x_image[0]);
				camera_publish_vision();
        fps_counter++;

#if (IPS200_OR_WIFI == 0)
#if CAMERA_DEBUG_DRAW_ENABLE
        ips200_show_gray_image(0, 0, mt9v03x_image[0], MT9V03X_W, MT9V03X_H, 188, 120, 0);
        ips200_show_string(0, 128, "mid:");
        ips200_show_uint8(40, 128, final_mid_line);
        ips200_show_string(0, 144, "fps:");
        ips200_show_uint16(40, 144, current_fps);
        ips200_show_string(0, 160, "conf:");
        ips200_show_uint8(48, 160, camera_confidence);
#endif
#elif (IPS200_OR_WIFI == 1)
        seekfree_assistant_camera_send();
#endif
    }
}
