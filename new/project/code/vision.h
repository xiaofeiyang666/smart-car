#ifndef VISION_H
#define VISION_H

#include "zf_common_headfile.h"

/* ===================== 图像基础参数 =====================
 * VISION_MID_COL 是物理中线对应的图像列。
 *   车在直道上如果整体偏左/偏右，先确认摄像头安装，再微调它。
 * VISION_CONTROL_ROW 是舵机取中线误差的预瞄行。
 *   数值越小，看得越远，入弯更早但更容易受远处岔口/噪声影响。
 *   数值越大，看得越近，直线更稳但入弯可能变晚。
 */
#define VISION_IMAGE_W              MT9V03X_W
#define VISION_IMAGE_H              MT9V03X_H
#define VISION_MID_COL              94
#define VISION_CONTROL_ROW          80

/* ===================== 灰度搜线参数 =====================
 * VISION_BLACK_POINT 是最低黑点阈值保护。
 * VISION_WHITE_MAX_MUL / VISION_WHITE_MIN_MUL 根据底部平均亮度生成白点上下阈值。
 *   白线找不全：可适当降低 WHITE_MIN_MUL 或 BLACK_POINT。
 *   把灰噪声当白线：可适当提高 WHITE_MIN_MUL 或 BLACK_POINT。
 * VISION_SEARCH_RANGE 是上一行边线到下一行的搜索半径。
 *   弯道丢线：可略增大；噪声误跳：可略减小。
 * VISION_PIXEL_OFFSET 是隔几行/列搜索一次，越大帧率越高但细节越少。
 */
#define VISION_BLACK_POINT          100
#define VISION_WHITE_MAX_MUL        13
#define VISION_WHITE_MIN_MUL        7
#define VISION_REFERENCE_ROWS       4
#define VISION_SEARCH_RANGE         16
#define VISION_STOP_ROW             8
#define VISION_PIXEL_OFFSET         4

/* route_mode 调试值：0 普通，1~7 环岛，8 十字路口。 */
#define VISION_ROUTE_CROSS          8

extern uint8 vision_reference_point;
extern uint8 vision_white_max_point;
extern uint8 vision_white_min_point;
extern uint8 vision_reference_col;
extern uint8 vision_reference_col_line[VISION_IMAGE_H];

extern uint8 vision_left_edge_line[VISION_IMAGE_H];
extern uint8 vision_right_edge_line[VISION_IMAGE_H];
extern uint8 vision_left_control_line[VISION_IMAGE_H];
extern uint8 vision_right_control_line[VISION_IMAGE_H];
extern uint8 vision_mid_line[VISION_IMAGE_H];

extern int16 vision_error;
extern int16 vision_preview_error;
extern int16 vision_far_error;
extern int16 vision_curve_error;
extern uint8 vision_final_mid;
extern uint8 vision_valid;
extern uint8 vision_confidence;
extern uint8 vision_valid_line_count;
extern uint8 vision_lost_left_count;
extern uint8 vision_lost_right_count;
extern uint8 vision_ring_state;
extern int8 vision_ring_dir;
extern uint8 vision_cross_state;
extern uint8 vision_cross_left_open_count;
extern uint8 vision_cross_right_open_count;
extern uint8 vision_cross_both_open_count;
extern uint8 vision_debug_left_control;
extern uint8 vision_debug_right_control;
extern uint8 vision_debug_mid_control;
extern uint8 vision_debug_ring_midpoint;
extern uint8 vision_debug_ring_mid_under;
extern uint8 vision_debug_ring_left115;
extern uint8 vision_debug_ring_left85;
extern uint8 vision_debug_ring_left55;
extern uint8 vision_debug_left_80;
extern uint8 vision_debug_right_80;
extern uint8 vision_debug_mid_80;
extern uint8 vision_debug_width_80;

extern uint8 vision_debug_left_60;
extern uint8 vision_debug_right_60;
extern uint8 vision_debug_mid_60;
extern uint8 vision_debug_width_60;

void vision_init(void);
void vision_process(const uint8 *image);
uint16 vision_get_contrast(uint8 temp1, uint8 temp2);
uint8 vision_is_continue_line(uint8 *line);
uint8 vision_clamp_col(int16 val);

#endif
