#ifndef CAMARA_H
#define CAMARA_H
#include "zf_common_headfile.h"

// 模式选择：0=IPS200显示，1=WiFi图传
#define IPS200_OR_WIFI   1
#define jidian_search_line 120
#define search_start_line 120
#define search_end_line 10
#define MID_W 94

extern uint8 img_threshold;
extern uint8 left_jidian;
extern uint8 right_jidian;
extern uint8 left_line_list[MT9V03X_H];
extern uint8 right_line_list[MT9V03X_H];
extern uint8 mid_line_list[MT9V03X_H];
extern uint8 final_mid_line;

extern int16 camera_bias_raw;
extern int16 camera_preview_raw;
extern int16 camera_preview_far_raw;
extern int16 camera_curve_raw;
extern uint8 camera_route_mode;
extern uint8 camera_valid_line_cnt;
extern uint8 camera_lost_left_cnt;
extern uint8 camera_lost_right_cnt;
extern uint8 camera_confidence;
extern volatile uint16 current_fps;

void camara_init(void);
void camara_task(void);
void my_fps_timer_callback(void);

#endif
