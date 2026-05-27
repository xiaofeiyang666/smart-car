#ifndef TARGET_RING_H
#define TARGET_RING_H

#include "zf_common_headfile.h"

extern volatile uint8 target_enable;
extern volatile uint8 target_flag;
extern volatile uint8 target_stage;
extern volatile uint8 target_confidence;

extern volatile uint8 target_x;
extern volatile uint8 target_y;
extern volatile uint8 target_left;
extern volatile uint8 target_right;
extern volatile uint8 target_top;
extern volatile uint8 target_bottom;

extern volatile int16 target_error_x;
extern volatile int16 target_servo2_input;
extern volatile uint8 target_aim_ok;

extern uint8 target_gray_th;

void target_process(const uint8 *image);

#endif