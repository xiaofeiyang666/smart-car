#ifndef CAMARA_H
#define CAMARA_H

#include "zf_common_headfile.h"
#include "vision.h"

#define IPS200_OR_WIFI   0

#define jidian_search_line         VISION_IMAGE_H
#define search_start_line          VISION_IMAGE_H
#define search_end_line            VISION_STOP_ROW
#define MID_W                      VISION_MID_COL

extern uint8 img_threshold;
extern uint8 left_jidian;
extern uint8 right_jidian;
extern uint8 left_line_list[MT9V03X_H];
extern uint8 right_line_list[MT9V03X_H];
extern uint8 mid_line_list[MT9V03X_H];
extern uint8 final_mid_line;
extern volatile uint16 current_fps;

extern int16 camera_bias_raw;
extern int16 camera_preview_raw;
extern int16 camera_preview_far_raw;
extern int16 camera_curve_raw;
extern uint8 camera_route_mode;
extern uint8 camera_valid_line_cnt;
extern uint8 camera_lost_left_cnt;
extern uint8 camera_lost_right_cnt;
extern uint8 camera_confidence;
extern int8 camera_ring_dir;
extern uint8 camera_cross_state;
extern uint8 camera_cross_left_open_cnt;
extern uint8 camera_cross_right_open_cnt;
extern uint8 camera_cross_both_open_cnt;
extern uint8 camera_debug_left_control;
extern uint8 camera_debug_right_control;
extern uint8 camera_debug_mid_control;
extern uint8 camera_debug_ring_midpoint;
extern uint8 camera_debug_ring_mid_under;
extern uint8 camera_debug_ring_left115;
extern uint8 camera_debug_ring_left85;
extern uint8 camera_debug_ring_left55;
extern uint8 camera_debug_left_80;
extern uint8 camera_debug_right_80;
extern uint8 camera_debug_mid_80;
extern uint8 camera_debug_width_80;

extern uint8 camera_debug_left_60;
extern uint8 camera_debug_right_60;
extern uint8 camera_debug_mid_60;
extern uint8 camera_debug_width_60;

void camara_init(void);
void camara_task(void);
void my_fps_timer_callback(void);

#endif
