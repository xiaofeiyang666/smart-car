#ifndef _CONTROL_H
#define _CONTROL_H
#include "zf_common_headfile.h"

extern int target_speed_base;


void control_init(void);
void control_loop(void);

extern volatile uint8 print_flag;
#endif