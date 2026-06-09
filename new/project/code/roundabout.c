#include "roundabout.h"
#include "vision.h"

uint8 roundabout_state = ROUNDABOUT_STATE_NORMAL;
int8 roundabout_dir = ROUNDABOUT_DIR_NONE;

uint8 current_step = 0;
uint8 key_anlysis1 = 0;
uint8 key_anlysis2 = 0;
uint8 key_anlysis3 = 0;

uint8 ring_preMeet_flag = 0;
uint8 ring_l = 0;
uint8 ring_r = 0;
uint8 first_meeting_flag = 0;
uint8 ring_enter_flag = 0;
uint8 ring_turn_flag = 0;
uint8 Out_flag = 0;
uint8 Straighten_flag = 0;
uint8 ring_over_flag = 0;

static uint8 mid_under_flag = 0;
uint8 midPoint = 0;
uint8 minPoint = 0;
uint16 cnt_over = 0;

uint8 roundabout_debug_mid_under_flag = 0;
uint8 roundabout_debug_left_115 = 0;
uint8 roundabout_debug_left_85 = 0;
uint8 roundabout_debug_left_55 = 0;

uint8 roundabout_debug_premeet = 0;
uint8 roundabout_debug_candidate_count = 0;


static int abs_i(int x)
{
    return (x >= 0) ? x : -x;
}

static void sync_roundabout_status(void)
{
    roundabout_state = current_step;
    if (current_step == 0 || current_step == 7)
    {
        roundabout_dir = ROUNDABOUT_DIR_NONE;
    }
    else
    {
        roundabout_dir = ROUNDABOUT_DIR_LEFT;
    }
}

void Ring(void)
{
    switch (current_step)
    {
    case 0:
        Ring_Pre_Meet();
        if (ring_preMeet_flag)
        {
            current_step = 1;
        }
				break;
    case 1:
        Ring_First_meeting();
        if (first_meeting_flag)
        {
            current_step = 2;
        }
        break;

    case 2:
        Ring_Enter();
        if (ring_enter_flag)
        {
            current_step = 3;
        }
        break;

    case 3:
        Ring_Turing();
        if (ring_turn_flag)
        {
            current_step = 4;
        }
        break;

    case 4:
        Ring_Ring_Ring();
        if (Out_flag)
        {
            current_step = 5;
        }
        break;

    case 5:
        Ring_Out();
        if (Straighten_flag)
        {
            current_step = 6;
        }
        break;

    case 6:
        Ring_Straighten();
        if (ring_over_flag)
        {
            current_step = 7;
        }
        break;

    case 7:
        Ring_Over();
        break;
    }

    sync_roundabout_status();
    roundabout_debug_mid_under_flag = mid_under_flag;
    roundabout_debug_left_115 = vision_left_edge_line[115];
    roundabout_debug_left_85 = vision_left_edge_line[85];
    roundabout_debug_left_55 = vision_left_edge_line[55];
		
		roundabout_debug_premeet = ring_preMeet_flag;
		roundabout_debug_candidate_count = 0;
}

void draw_line(uint8 left_point, uint8 right_point, uint8 to_flag)
{
    float slope;
    uint8 i;
    uint8 start_row;
    uint8 end_row;
    uint8 start_col;
    uint8 end_col;

    if (left_point > right_point)
    {
        start_row = right_point;
        end_row = left_point;
        start_col = vision_right_control_line[right_point];
        end_col = vision_left_control_line[left_point];
    }
    else
    {
        start_row = left_point;
        end_row = right_point;
        start_col = vision_left_control_line[left_point];
        end_col = vision_right_control_line[right_point];
    }

    if (end_row == start_row) return;

    slope = (float)(end_col - start_col) / (end_row - start_row);

    if (!to_flag)
    {
        for (i = start_row + 1; i < end_row; i++)
        {
            vision_left_control_line[i] = (uint8)(start_col + slope * (i - start_row) + 0.5f);
        }
    }
    else
    {
        for (i = start_row + 1; i < end_row; i++)
        {
            vision_right_control_line[i] = (uint8)(start_col + slope * (i - start_row) + 0.5f);
        }
    }
}

void connect_point(uint8 under, uint8 top, uint8 l_r)
{
    float slope;
    uint8 i;

    if (!l_r)
    {
        slope = (float)(vision_left_control_line[under] - vision_left_control_line[top]) / (under - top);

        for (i = top + 1; i < under; i++)
        {
            vision_left_control_line[i] = vision_left_control_line[top] + slope * (i - top);
        }
    }
    else
    {
        slope = (float)(vision_right_control_line[under] - vision_right_control_line[top]) / (under - top);

        for (i = top + 1; i < under; i++)
        {
            vision_right_control_line[i] = vision_right_control_line[top] + slope * (i - top);
        }
    }
}

uint8 isContinueLine(uint8 *arr)
{
    int16 i;
    for (i = VISION_IMAGE_H - 1 - VISION_PIXEL_OFFSET; i > 7 * VISION_PIXEL_OFFSET; i -= VISION_PIXEL_OFFSET)
    {
        uint8 curr = arr[i];
        uint8 prev = arr[i - VISION_PIXEL_OFFSET];
        if (abs_i(curr - prev) > 40)
        {
            return 0;
        }
    }
    return 1;
}

uint8 Ring_Pre_Meet(void)
{
    uint8 r_con;
    uint8 l_con;

    r_con = isContinueLine(vision_right_edge_line);
    l_con = isContinueLine(vision_left_edge_line);

    if (r_con != l_con)
    {
        ring_preMeet_flag = 1;
        return 1;
    }
    else
    {
        ring_preMeet_flag = 0;
        return 0;
    }
}

uint8 Ring_Pre_Meet_use(void)
{
    uint8 r_con;
    uint8 l_con;

    r_con = isContinueLine(vision_right_edge_line);
    l_con = isContinueLine(vision_left_edge_line);

    if (r_con != l_con)
    {
        return 1;
    }
    else
    {
        return 0;
    }
}

void Ring_First_meeting(void)
{
    uint8 i;
    uint8 under;
    uint8 top;
    uint8 mid;

    uint8 turnPoint = VISION_IMAGE_H - 1;
    uint8 midPointLocal = 0;

    if (!Ring_Pre_Meet_use())
    {
        current_step = 0;
        ring_preMeet_flag = 0;
    }

    for (i = VISION_IMAGE_H - 1 - VISION_PIXEL_OFFSET; i > 30; i -= VISION_PIXEL_OFFSET)
    {
        under = vision_left_edge_line[i];
        top = vision_left_edge_line[i - VISION_PIXEL_OFFSET];

        if ((under - top) >= 15)
        {
            turnPoint = i;
            break;
        }
    }

    for (i = turnPoint - VISION_PIXEL_OFFSET; i > VISION_PIXEL_OFFSET; i -= VISION_PIXEL_OFFSET)
    {
        under = vision_left_edge_line[i + VISION_PIXEL_OFFSET];
        mid = vision_left_edge_line[i];
        top = vision_left_edge_line[i - VISION_PIXEL_OFFSET];

        if (mid >= under && mid >= top && mid > 30)
        {
            midPointLocal = i;
        }
    }

    if (turnPoint && midPointLocal)
    {
        connect_point(turnPoint, midPointLocal, 0);
    }

    if (midPointLocal &&
        vision_left_edge_line[115] <= 2 &&
        vision_left_edge_line[85] <= 2 &&
        vision_left_edge_line[55] <= 2)
    {
        first_meeting_flag = 1;
    }
}

void Ring_Enter(void)
{
    uint8 i;
    uint8 under;
    uint8 mid;
    uint8 top;
    uint8 ttop;
    uint8 uunder;

    midPoint = 0;
    for (i = VISION_IMAGE_H - 1 - 2 * VISION_PIXEL_OFFSET; i > VISION_PIXEL_OFFSET * 2; i -= VISION_PIXEL_OFFSET)
    {
        uunder = vision_left_edge_line[i + VISION_PIXEL_OFFSET + VISION_PIXEL_OFFSET];
        under = vision_left_edge_line[i + VISION_PIXEL_OFFSET];
        mid = vision_left_edge_line[i];
        top = vision_left_edge_line[i - VISION_PIXEL_OFFSET];
        ttop = vision_left_edge_line[i - VISION_PIXEL_OFFSET - VISION_PIXEL_OFFSET];

        if (mid < 8) continue;
        if (mid >= under && (mid - under) < 20 &&
            under >= uunder && (under - uunder) < 20 &&
            mid >= top && (mid - top) < 20 &&
            top >= ttop && (top - ttop) < 20 &&
            mid > 30)
        {
            midPoint = i;
            break;
        }
    }

    if (midPoint)
    {
        connect_point(VISION_IMAGE_H - 1, midPoint, 0);
    }
    else
    {
        first_meeting_flag = 0;
        current_step = 1;
    }

    if (midPoint >= 60)
    {
        mid_under_flag = 1;
    }

    if (mid_under_flag && midPoint < 45)
    {
        ring_enter_flag = 1;
    }
}
void Ring_Turing(void)
{
    uint8 i;
    uint8 under;
    uint8 top;
    uint8 leftTopPoint = 0;

    for (i = VISION_IMAGE_H - 1; i > VISION_PIXEL_OFFSET; i -= VISION_PIXEL_OFFSET)
    {
        top = vision_left_edge_line[i - VISION_PIXEL_OFFSET];
        under = vision_left_edge_line[i];

        if (abs_i(top - under) > 30)
        {
            leftTopPoint = i - VISION_PIXEL_OFFSET;
            break;
        }
    }

    draw_line(leftTopPoint, VISION_IMAGE_H - 1, 1);

    if (leftTopPoint >= VISION_CONTROL_ROW - 20)
    {
        ring_turn_flag = 1;
    }
}

void Ring_Ring_Ring(void)
{
    uint8 i;
    uint8 index;
    uint8 minn = VISION_IMAGE_W;

    minPoint = 0;
    if (vision_left_control_line[VISION_CONTROL_ROW] > VISION_MID_COL)
    {
        vision_left_control_line[VISION_CONTROL_ROW] = 0;
    }

    for (i = VISION_IMAGE_H - 1; i > VISION_PIXEL_OFFSET * 3; i -= VISION_PIXEL_OFFSET)
    {
        index = vision_right_edge_line[i];
        if (index < minn)
        {
            minn = index;
            minPoint = i;
        }
    }

    if ((vision_right_edge_line[minPoint - VISION_PIXEL_OFFSET] - vision_right_edge_line[minPoint] < 30 &&
         vision_right_edge_line[minPoint + VISION_PIXEL_OFFSET] - vision_right_edge_line[minPoint] < 30) ||
        minPoint < 40)
    {
        draw_line(0, VISION_IMAGE_H - 1, 1);
    }

    if (vision_right_edge_line[50] > 180 &&
        vision_right_edge_line[60] > 180 &&
        vision_right_edge_line[70] > 180 &&
        vision_right_edge_line[80] > 180 &&
        vision_right_edge_line[90] > 180 &&
        vision_left_edge_line[50] < 2)
    {
        Out_flag = 1;
    }
}

void Ring_Out(void)
{
    draw_line(0, VISION_IMAGE_H - 1, 1);
    if ((vision_left_edge_line[30] > 16 ||
         vision_left_edge_line[40] > 16 ||
         vision_left_edge_line[50] > 16) &&
        (vision_right_edge_line[40] - vision_left_edge_line[40] < 170) &&
        (vision_right_edge_line[50] - vision_left_edge_line[50] < 170) &&
        (vision_right_edge_line[60] - vision_left_edge_line[60] < 170))
    {
        Straighten_flag = 1;
    }
}

void Ring_Straighten(void)
{
    uint8 i;
    uint8 under;
    uint8 mid;
    uint8 top;
    uint8 midPointLocal = 0;

    for (i = VISION_IMAGE_H - 1 - VISION_PIXEL_OFFSET; i > VISION_PIXEL_OFFSET; i -= VISION_PIXEL_OFFSET)
    {
        under = vision_left_edge_line[i + VISION_PIXEL_OFFSET];
        mid = vision_left_edge_line[i];
        top = vision_left_edge_line[i - VISION_PIXEL_OFFSET];

        if (mid >= under && mid >= top && mid > 30)
        {
            midPointLocal = i;
        }
    }

    if (midPointLocal)
    {
        connect_point(VISION_IMAGE_H - 1, midPointLocal, 0);
    }

    if (isContinueLine(vision_left_edge_line) && isContinueLine(vision_right_edge_line))
    {
        ring_over_flag = 1;
    }
}

void Ring_Over(void)
{
    first_meeting_flag = 0;
    ring_l = 0;
    ring_r = 0;
    ring_enter_flag = 0;
    ring_turn_flag = 0;
    Out_flag = 0;
    Straighten_flag = 0;
    ring_over_flag = 0;
    mid_under_flag = 0;
    midPoint = 0;
    current_step = 0;
    cnt_over = 0;
    roundabout_debug_mid_under_flag = 0;
    roundabout_debug_left_115 = 0;
    roundabout_debug_left_85 = 0;
    roundabout_debug_left_55 = 0;
}

void roundabout_reset(void)
{
    Ring_Over();
    ring_preMeet_flag = 0;
    roundabout_state = ROUNDABOUT_STATE_NORMAL;
    roundabout_dir = ROUNDABOUT_DIR_NONE;
}

void roundabout_process(void)
{
    Ring();
}