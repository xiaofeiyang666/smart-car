#ifndef CAMARA_H
#define CAMARA_H

#include "zf_common_headfile.h"

// 模式选择：0=IPS200显示，1=WiFi图传
#define IPS200_OR_WIFI   0

// 图像搜索范围配置
#define jidian_search_line         120
#define search_start_line          120
#define search_end_line            10
#define left_line_right_scarch     10
#define left_line_left_scarch      5
#define right_line_left_scarch     10
#define right_line_right_scarch    5
#define MID_W                      94

extern uint8 img_threshold;
extern uint8 left_jidian;
extern uint8 right_jidian;
extern uint8 left_line_list[MT9V03X_H];
extern uint8 right_line_list[MT9V03X_H];
extern uint8 mid_line_list[MT9V03X_H];
extern volatile uint8 final_mid_line;
extern volatile uint16 current_fps;

// 提供给 control 的视觉质量指标
extern volatile int16 camera_bias_raw;
extern volatile int16 camera_preview_raw;
extern volatile uint8 camera_valid_line_cnt;
extern volatile uint8 camera_lost_left_cnt;
extern volatile uint8 camera_lost_right_cnt;
extern volatile uint8 camera_confidence;
extern volatile int16 camera_preview_far_raw;
extern volatile int8 camera_ring_dir;
extern volatile uint8 camera_ring_score;
extern volatile uint8 camera_s_curve_flag;

void camara_init(void);
void camara_task(void);
void my_fps_timer_callback(void);

#endif
