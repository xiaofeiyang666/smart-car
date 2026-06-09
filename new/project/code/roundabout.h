#ifndef ROUNDABOUT_H
#define ROUNDABOUT_H

#include "zf_common_headfile.h"

#define ROUNDABOUT_STATE_NORMAL       0
#define ROUNDABOUT_STATE_FIRST        1
#define ROUNDABOUT_STATE_ENTER        2
#define ROUNDABOUT_STATE_TURNING      3
#define ROUNDABOUT_STATE_INSIDE       4
#define ROUNDABOUT_STATE_OUT          5
#define ROUNDABOUT_STATE_STRAIGHTEN   6
#define ROUNDABOUT_STATE_OVER         7

#define ROUNDABOUT_DIR_NONE           0
#define ROUNDABOUT_DIR_LEFT          -1
#define ROUNDABOUT_DIR_RIGHT          1

extern uint8 roundabout_state;
extern int8 roundabout_dir;

extern uint8 ring_preMeet_flag;
extern uint8 first_meeting_flag;
extern uint8 ring_l;
extern uint8 ring_r;
extern uint8 ring_enter_flag;
extern uint8 ring_turn_flag;
extern uint8 Out_flag;
extern uint8 Straighten_flag;
extern uint8 ring_over_flag;
extern uint8 midPoint;
extern uint8 roundabout_debug_mid_under_flag;
extern uint8 roundabout_debug_left_115;
extern uint8 roundabout_debug_left_85;
extern uint8 roundabout_debug_left_55;
extern uint8 roundabout_debug_premeet;
extern uint8 roundabout_debug_candidate_count;

extern uint8 key_anlysis1;
extern uint8 key_anlysis2;
extern uint8 key_anlysis3;

void Ring(void);
void draw_line(uint8 left_point, uint8 right_point, uint8 to_flag);
void connect_point(uint8 under, uint8 top, uint8 l_r);

uint8 Ring_Pre_Meet(void);
uint8 Ring_Pre_Meet_use(void);
void Ring_First_meeting(void);
void Ring_Enter(void);
void Ring_Turing(void);
void Ring_Ring_Ring(void);
void Ring_Out(void);
void Ring_Straighten(void);
void Ring_Over(void);

uint8 isContinueLine(uint8 *arr);

void roundabout_reset(void);
void roundabout_process(void);

#endif
