#ifndef CAMARA_H
#define CAMARA_H
#include "zf_common_headfile.h"

// 模式选择：0=IPS200显示，1=WiFi图传
#define IPS200_OR_WIFI   0	
#define jidian_search_line 120
#define search_start_line 120
#define search_end_line 10
#define left_line_right_scarch 10
#define left_line_left_scarch 5
#define right_line_left_scarch 10
#define right_line_right_scarch 5
#define MID_W 90


//extern uint8 image[MT9V03X_H][MT9V03X_W];
extern uint8 img_threshold;
extern uint8 base_image[MT9V03X_H][MT9V03X_W];
extern uint8 left_jidian;
extern uint8 right_jidian;
extern uint8 left_line_list[MT9V03X_H];
extern uint8 right_line_list[MT9V03X_H];
extern uint8 mid_line_list[MT9V03X_H];

void camara_init(void);
void camara_task(void);
void my_fps_timer_callback(void);
extern uint8 final_mid_line; // 导出最终计算出的中线位置
#endif
