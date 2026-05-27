#include "vision.h"
#include "roundabout.h"
#include <string.h>

uint8 vision_reference_point = 0;
uint8 vision_white_max_point = 0;
uint8 vision_white_min_point = 0;
uint8 vision_reference_col = VISION_MID_COL;
uint8 vision_reference_col_line[VISION_IMAGE_H];

uint8 vision_left_edge_line[VISION_IMAGE_H];
uint8 vision_right_edge_line[VISION_IMAGE_H];
uint8 vision_left_control_line[VISION_IMAGE_H];
uint8 vision_right_control_line[VISION_IMAGE_H];
uint8 vision_mid_line[VISION_IMAGE_H];

int16 vision_error = 0;
int16 vision_preview_error = 0;
int16 vision_far_error = 0;
int16 vision_curve_error = 0;
uint8 vision_final_mid = VISION_MID_COL;
uint8 vision_valid = 0;
uint8 vision_confidence = 0;
uint8 vision_valid_line_count = 0;
uint8 vision_lost_left_count = 0;
uint8 vision_lost_right_count = 0;
uint8 vision_ring_state = ROUNDABOUT_STATE_NORMAL;
int8 vision_ring_dir = ROUNDABOUT_DIR_NONE;
static uint8 vision_cross_flag = 0;
uint8 vision_cross_state = 0;
uint8 vision_cross_left_open_count = 0;
uint8 vision_cross_right_open_count = 0;
uint8 vision_cross_both_open_count = 0;
uint8 vision_debug_left_control = 0;
uint8 vision_debug_right_control = VISION_IMAGE_W - 1;
uint8 vision_debug_mid_control = VISION_MID_COL;
uint8 vision_debug_ring_midpoint = 0;
uint8 vision_debug_ring_mid_under = 0;
uint8 vision_debug_ring_left115 = 0;
uint8 vision_debug_ring_left85 = 0;
uint8 vision_debug_ring_left55 = 0;
uint8 vision_debug_left_80 = 0;
uint8 vision_debug_right_80 = 0;
uint8 vision_debug_mid_80 = VISION_MID_COL;
uint8 vision_debug_width_80 = 0;

uint8 vision_debug_left_60 = 0;
uint8 vision_debug_right_60 = 0;
uint8 vision_debug_mid_60 = VISION_MID_COL;
uint8 vision_debug_width_60 = 0;

static uint8 final_mid_init = 0;
static int16 final_mid_filtered_x8 = VISION_MID_COL * 8;
static uint8 p_cross_hold_count = 0;
static uint8 p_cross_ring_guard_count = 0;

#define ROAD_LOST_LEFT_EDGE          2
#define ROAD_LOST_RIGHT_EDGE         (VISION_IMAGE_W - 3)

#define P_CROSS_ROW_MIN             (VISION_CONTROL_ROW - 18)
#define P_CROSS_ROW_MAX             (VISION_CONTROL_ROW + 22)
#define P_CROSS_BOTH_OPEN_MIN       18
#define P_CROSS_SIDE_OPEN_MIN       30
#define P_CROSS_HOLD_FRAMES         4
#define P_CROSS_RING_GUARD_LEFT_OPEN_MIN   30
#define P_CROSS_RING_GUARD_RIGHT_OPEN_MAX  5
#define P_CROSS_RING_GUARD_BOTH_OPEN_MAX   5
#define P_CROSS_RING_GUARD_HOLD_FRAMES     24

/* P/cross detection only suppresses roundabout takeover; it does not modify control lines. */

static int abs_i(int x)
{
    return (x >= 0) ? x : -x;
}

static int clamp_i(int x, int min_v, int max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

uint8 vision_clamp_col(int16 val)
{
    if (val < 0) return 0;
    if (val >= VISION_IMAGE_W) return VISION_IMAGE_W - 1;
    return (uint8)val;
}

uint16 vision_get_contrast(uint8 temp1, uint8 temp2)
{
    int16 diff = (int16)temp1 - (int16)temp2;
    if (diff < 0) diff = -diff;
    return (uint16)((uint32)diff * 200u / ((uint16)temp1 + (uint16)temp2 + 1u));
}

uint8 vision_is_continue_line(uint8 *line)
{
    int16 i;
    for (i = VISION_IMAGE_H - 1 - VISION_PIXEL_OFFSET;
         i > 7 * VISION_PIXEL_OFFSET;
         i -= VISION_PIXEL_OFFSET)
    {
        if (abs_i((int)line[i] - (int)line[i - VISION_PIXEL_OFFSET]) > 40)
        {
            return 0;
        }
    }
    return 1;
}

static void reset_lines(void)
{
    uint8 row;
    for (row = 0; row < VISION_IMAGE_H; row++)
    {
        vision_left_edge_line[row] = 0;
        vision_right_edge_line[row] = VISION_IMAGE_W - 1;
        vision_left_control_line[row] = 0;
        vision_right_control_line[row] = VISION_IMAGE_W - 1;
        vision_mid_line[row] = VISION_MID_COL;
        vision_reference_col_line[row] = VISION_MID_COL;
    }
}

static void get_reference_point(const uint8 *image)
{
    uint16 i;
    uint16 total;
    uint32 sum = 0;
    const uint8 *p = image + (VISION_IMAGE_H - VISION_REFERENCE_ROWS) * VISION_IMAGE_W;

    total = VISION_REFERENCE_ROWS * VISION_IMAGE_W;
    for (i = 0; i < total; i++)
    {
        sum += *(p + i);
    }

    vision_reference_point = (uint8)(sum / total);
    vision_white_max_point = (uint8)clamp_i(
        ((uint16)vision_reference_point * VISION_WHITE_MAX_MUL) / 10,
        VISION_BLACK_POINT,
        230);
    vision_white_min_point = (uint8)clamp_i(
        ((uint16)vision_reference_point * VISION_WHITE_MIN_MUL) / 10,
        VISION_BLACK_POINT,
        230);
}

static void search_reference_col(const uint8 *image)
{
    uint8 col;
    uint8 row;
    uint8 i;
    uint8 temp1;
    uint8 temp2;
    uint8 globe_remote;
    uint8 globe_remote_min;
    uint16 contrast;

    globe_remote_min = VISION_IMAGE_H;
    vision_reference_col = VISION_MID_COL;

    for (col = 0; col < VISION_IMAGE_W; col += VISION_PIXEL_OFFSET)
    {
        globe_remote = VISION_IMAGE_H;

        for (row = VISION_IMAGE_H - 1; row > VISION_PIXEL_OFFSET; row -= VISION_PIXEL_OFFSET)
        {
            temp1 = *(image + row * VISION_IMAGE_W + col);
            temp2 = *(image + (row - VISION_PIXEL_OFFSET) * VISION_IMAGE_W + col);

            if (temp2 > vision_white_max_point)
            {
                continue;
            }
            else if (temp1 < vision_white_min_point)
            {
                if (globe_remote > row) globe_remote = row;
                break;
            }

            contrast = vision_get_contrast(temp1, temp2);
            if ((contrast > 32) || (row == VISION_STOP_ROW))
            {
                if (globe_remote > row) globe_remote = row;
            }
        }

        if (globe_remote < globe_remote_min)
        {
            globe_remote_min = globe_remote;
            vision_reference_col = col;
        }
    }

    if (vision_reference_col > VISION_IMAGE_W - 1)
    {
        vision_reference_col = VISION_MID_COL;
    }

    for (i = 0; i < VISION_IMAGE_H; i++)
    {
        vision_reference_col_line[i] = vision_reference_col;
    }
}

static void insert_val(void)
{
    int16 i;
    uint8 j;
    uint8 front_p;
    uint8 cur_p;
    uint8 step;

    front_p = vision_left_edge_line[VISION_IMAGE_H - 1];
    for (i = VISION_IMAGE_H - VISION_PIXEL_OFFSET - 1; i >= VISION_STOP_ROW; i -= VISION_PIXEL_OFFSET)
    {
        cur_p = vision_left_edge_line[i];
        if (cur_p >= front_p)
        {
            step = (cur_p - front_p + VISION_PIXEL_OFFSET - 1) / VISION_PIXEL_OFFSET;
            for (j = 1; j < VISION_PIXEL_OFFSET; j++)
            {
                vision_left_edge_line[i - j + VISION_PIXEL_OFFSET] = front_p + step * j;
            }
        }
        else
        {
            step = (front_p - cur_p + VISION_PIXEL_OFFSET - 1) / VISION_PIXEL_OFFSET;
            for (j = 1; j < VISION_PIXEL_OFFSET; j++)
            {
                vision_left_edge_line[i - j + VISION_PIXEL_OFFSET] = front_p - step * j;
            }
        }
        front_p = cur_p;
    }

    front_p = vision_right_edge_line[VISION_IMAGE_H - 1];
    for (i = VISION_IMAGE_H - VISION_PIXEL_OFFSET - 1; i >= VISION_STOP_ROW; i -= VISION_PIXEL_OFFSET)
    {
        cur_p = vision_right_edge_line[i];
        if (cur_p >= front_p)
        {
            step = (cur_p - front_p + VISION_PIXEL_OFFSET - 1) / VISION_PIXEL_OFFSET;
            for (j = 1; j < VISION_PIXEL_OFFSET; j++)
            {
                vision_right_edge_line[i - j + VISION_PIXEL_OFFSET] = front_p + step * j;
            }
        }
        else
        {
            step = (front_p - cur_p + VISION_PIXEL_OFFSET - 1) / VISION_PIXEL_OFFSET;
            for (j = 1; j < VISION_PIXEL_OFFSET; j++)
            {
                vision_right_edge_line[i - j + VISION_PIXEL_OFFSET] = front_p - step * j;
            }
        }
        front_p = cur_p;
    }
}

static void search_line(const uint8 *image)
{
    const uint8 *p;
    uint8 row_max = VISION_IMAGE_H - 1;
    uint8 row_min = VISION_STOP_ROW;
    uint8 col_max = VISION_IMAGE_W - 1;
    uint8 col_min = 0;
    uint8 left_start_col = vision_reference_col;
    uint8 right_start_col = (uint8)clamp_i((int)vision_reference_col + 10, 0, VISION_IMAGE_W - 1);
    uint8 left_end_col = col_min;
    uint8 right_end_col = col_max;
    uint8 search_time;
    uint8 temp1;
    uint8 temp2;
    uint16 contrast;
    uint8 left_stop = 0;
    uint8 right_stop = 0;
    uint8 stop_point;
    int16 col;
    int16 row;

    vision_lost_left_count = 0;
    vision_lost_right_count = 0;
    vision_valid_line_count = 0;

    for (row = row_max; row >= row_min; row--)
    {
        vision_left_edge_line[row] = 0;
        vision_right_edge_line[row] = VISION_IMAGE_W - 1;
    }

    for (row = row_max; row >= row_min; row -= VISION_PIXEL_OFFSET)
    {
        p = image + row * VISION_IMAGE_W;

        if (!left_stop)
        {
            search_time = 2;
            do
            {
                if (search_time == 1)
                {
                    left_start_col = vision_reference_col;
                    left_end_col = col_min;
                }
                search_time--;

                for (col = left_start_col; col > left_end_col + VISION_PIXEL_OFFSET; col -= VISION_PIXEL_OFFSET)
                {
                    temp1 = *(p + col);
                    temp2 = *(p + col - VISION_PIXEL_OFFSET);

                    if ((temp1 < vision_white_min_point) &&
                        (col == left_start_col) &&
                        (left_start_col == vision_reference_col))
                    {
                        left_stop = 1;
                        search_time = 0;
                        for (stop_point = (uint8)row; stop_point > 1; stop_point--)
                        {
                            vision_left_edge_line[stop_point] = col_min;
                        }
                        break;
                    }

                    if (temp1 < vision_white_min_point)
                    {
                        vision_left_edge_line[row] = (uint8)col;
                        break;
                    }

                    if (temp2 > vision_white_max_point)
                    {
                        continue;
                    }

                    contrast = vision_get_contrast(temp1, temp2);
                    if ((contrast > 32) || (col == col_min))
                    {
                        vision_left_edge_line[row] = (uint8)col;
                        left_start_col = (uint8)clamp_i(col + VISION_SEARCH_RANGE, col, col_max);
                        left_end_col = (uint8)clamp_i(col - VISION_SEARCH_RANGE, col_min, col);
                        search_time = 0;
                        break;
                    }
                }
            } while (search_time);
        }

        if (!right_stop)
        {
            search_time = 2;
            do
            {
                if (search_time == 1)
                {
                    right_start_col = vision_reference_col;
                    right_end_col = col_max;
                }
                search_time--;

                for (col = right_start_col; col <= right_end_col - VISION_PIXEL_OFFSET; col += VISION_PIXEL_OFFSET)
                {
                    temp1 = *(p + col);
                    temp2 = *(p + col + VISION_PIXEL_OFFSET);

                    if ((temp1 < vision_white_min_point) &&
                        (col == right_start_col) &&
                        (right_start_col == vision_reference_col))
                    {
                        right_stop = 1;
                        search_time = 0;
                        for (stop_point = (uint8)row; stop_point > row_min; stop_point--)
                        {
                            vision_right_edge_line[stop_point] = col_max;
                        }
                        break;
                    }

                    if (temp1 < vision_white_min_point)
                    {
                        vision_right_edge_line[row] = (uint8)col;
                        break;
                    }

                    if (temp2 > vision_white_max_point)
                    {
                        continue;
                    }

                    contrast = vision_get_contrast(temp1, temp2);
                    if ((contrast > 32) || (col >= col_max - VISION_PIXEL_OFFSET))
                    {
                        vision_right_edge_line[row] = (uint8)col;
                        right_start_col = (uint8)clamp_i(col - VISION_SEARCH_RANGE, col_min, col);
                        right_end_col = (uint8)clamp_i(col + VISION_SEARCH_RANGE, col, col_max);
                        search_time = 0;
                        break;
                    }
                }
            } while (search_time);
        }
    }

    insert_val();
    memcpy(vision_left_control_line, vision_left_edge_line, sizeof(vision_left_edge_line));
    memcpy(vision_right_control_line, vision_right_edge_line, sizeof(vision_right_edge_line));
}

static uint8 edge_left_lost(uint8 row)
{
    return (vision_left_edge_line[row] <= ROAD_LOST_LEFT_EDGE) ? 1 : 0;
}

static uint8 edge_right_lost(uint8 row)
{
    return (vision_right_edge_line[row] >= ROAD_LOST_RIGHT_EDGE) ? 1 : 0;
}

/*
 * P/cross detection only:
 * - Count rows near the control row where the left/right edge is stuck at image border.
 * - Hold route=8 for a few frames after detection.
 * - Do not modify edge lines, control lines, or midline.
 */
static void p_cross_detect_only(void)
{
    uint8 row;
    uint8 left_open = 0;
    uint8 right_open = 0;
    uint8 both_open = 0;
    uint8 left_lost;
    uint8 right_lost;
    uint8 ring_guard_active = 0;

    for (row = P_CROSS_ROW_MIN; row <= P_CROSS_ROW_MAX; row++)
    {
        left_lost = edge_left_lost(row);
        right_lost = edge_right_lost(row);

        if (left_lost) left_open++;
        if (right_lost) right_open++;
        if (left_lost && right_lost) both_open++;
    }

    vision_cross_left_open_count = left_open;
    vision_cross_right_open_count = right_open;
    vision_cross_both_open_count = both_open;
    vision_cross_state = 0;
    vision_cross_flag = 0;

    /* Left ring entrance often looks like left edge lost only. Do not let cross detection reset ring state there. */
    if (roundabout_state != ROUNDABOUT_STATE_NORMAL &&
        left_open >= P_CROSS_RING_GUARD_LEFT_OPEN_MIN &&
        right_open <= P_CROSS_RING_GUARD_RIGHT_OPEN_MAX &&
        both_open <= P_CROSS_RING_GUARD_BOTH_OPEN_MAX)
    {
        p_cross_ring_guard_count = P_CROSS_RING_GUARD_HOLD_FRAMES;
    }
    else if (p_cross_ring_guard_count > 0)
    {
        p_cross_ring_guard_count--;
    }

    if (p_cross_ring_guard_count > 0 && roundabout_state != ROUNDABOUT_STATE_NORMAL)
    {
        ring_guard_active = 1;
        p_cross_hold_count = 0;
    }

    if (roundabout_state >= ROUNDABOUT_STATE_TURNING &&
        roundabout_state <= ROUNDABOUT_STATE_STRAIGHTEN)
    {
        ring_guard_active = 1;
        p_cross_hold_count = 0;
    }

    if (!ring_guard_active)
    {
        if (both_open >= P_CROSS_BOTH_OPEN_MIN ||
            (right_open >= P_CROSS_SIDE_OPEN_MIN && left_open >= (P_CROSS_SIDE_OPEN_MIN / 2)))
        {
            p_cross_hold_count = P_CROSS_HOLD_FRAMES;
        }
        else if (p_cross_hold_count > 0)
        {
            p_cross_hold_count--;
        }
    }
    if (p_cross_hold_count > 0)
    {
        vision_cross_flag = 1;
        vision_cross_state = VISION_ROUTE_CROSS;
    }
}

static int avg_mid_window(uint8 center_row, uint8 radius)
{
    int16 r;
    int16 start;
    int16 end;
    int32 sum = 0;
    int16 count = 0;

    start = (int16)center_row - radius;
    end = (int16)center_row + radius;
    if (start < VISION_STOP_ROW) start = VISION_STOP_ROW;
    if (end >= VISION_IMAGE_H) end = VISION_IMAGE_H - 1;

    for (r = start; r <= end; r++)
    {
        sum += vision_mid_line[r];
        count++;
    }

    if (count == 0) return VISION_MID_COL;
    return (int)(sum / count);
}

static void fitted_midline(void)
{
    uint8 i;
    int16 width;
    int16 mid;
    int16 valid = 0;
    int16 lost_left = 0;
    int16 lost_right = 0;

    for (i = 0; i < VISION_IMAGE_H; i++)
    {
        mid = ((int16)vision_left_control_line[i] + (int16)vision_right_control_line[i]) >> 1;
        vision_mid_line[i] = vision_clamp_col(mid);

        if (i >= VISION_STOP_ROW)
        {
            width = (int16)vision_right_control_line[i] - (int16)vision_left_control_line[i];
            if (width > 12 && width < (VISION_IMAGE_W - 4))
            {
                valid++;
            }
            if (vision_left_edge_line[i] <= 2)
            {
                lost_left++;
            }
            if (vision_right_edge_line[i] >= VISION_IMAGE_W - 3)
            {
                lost_right++;
            }
        }
    }

    if (valid > 255) valid = 255;
    if (lost_left > 255) lost_left = 255;
    if (lost_right > 255) lost_right = 255;

    vision_valid_line_count = (uint8)valid;
    vision_lost_left_count = (uint8)lost_left;
    vision_lost_right_count = (uint8)lost_right;
}

static void build_outputs(void)
{
    int16 near_mid;
    int16 ctrl_mid;
    int16 far_mid;
    int16 final_mid;
    int16 conf;

    fitted_midline();

    near_mid = (int16)avg_mid_window(108, 3);
    ctrl_mid = (int16)avg_mid_window(VISION_CONTROL_ROW, 4);
    far_mid = (int16)avg_mid_window(42, 4);

    if (!final_mid_init)
    {
        final_mid_filtered_x8 = ctrl_mid * 8;
        final_mid_init = 1;
    }
    else
    {
        final_mid_filtered_x8 = (int16)((final_mid_filtered_x8 * 5 + ctrl_mid * 8 * 3) / 8);
    }

    final_mid = (final_mid_filtered_x8 + 4) / 8;
    final_mid = (int16)clamp_i(final_mid, 1, VISION_IMAGE_W - 2);

    vision_final_mid = (uint8)final_mid;
    vision_error = (int16)(final_mid - VISION_MID_COL);
    vision_preview_error = (int16)(ctrl_mid - VISION_MID_COL);
    vision_far_error = (int16)(far_mid - VISION_MID_COL);
    vision_curve_error = (int16)(far_mid - near_mid);
    vision_debug_left_control = vision_left_control_line[VISION_CONTROL_ROW];
    vision_debug_right_control = vision_right_control_line[VISION_CONTROL_ROW];
    vision_debug_mid_control = vision_mid_line[VISION_CONTROL_ROW];
    vision_debug_ring_midpoint = midPoint;
    vision_debug_ring_mid_under = roundabout_debug_mid_under_flag;
    vision_debug_ring_left115 = vision_left_edge_line[115];
    vision_debug_ring_left85 = vision_left_edge_line[85];
    vision_debug_ring_left55 = vision_left_edge_line[55];
		vision_debug_left_80 = vision_left_control_line[80];
		vision_debug_right_80 = vision_right_control_line[80];
		vision_debug_mid_80 = vision_mid_line[80];
		vision_debug_width_80 = (uint8)clamp_i(
    (int)vision_right_control_line[80] - (int)vision_left_control_line[80],
    0,
    255);

		vision_debug_left_60 = vision_left_control_line[60];
		vision_debug_right_60 = vision_right_control_line[60];
		vision_debug_mid_60 = vision_mid_line[60];
		vision_debug_width_60 = (uint8)clamp_i(
    (int)vision_right_control_line[60] - (int)vision_left_control_line[60],
    0,
    255);

    conf = (int16)vision_valid_line_count;
    if (conf > 100) conf = 100;
    conf -= (int16)(vision_lost_left_count + vision_lost_right_count) / 4;
    if (conf < 0) conf = 0;
    if (conf > 100) conf = 100;

    vision_confidence = (uint8)conf;
    vision_valid = (vision_confidence >= 20 && vision_valid_line_count >= 12) ? 1 : 0;
    if (vision_cross_flag)
    {
        vision_ring_state = VISION_ROUTE_CROSS;
        vision_ring_dir = ROUNDABOUT_DIR_NONE;
    }
    else
    {
        vision_ring_state = roundabout_state;
        vision_ring_dir = roundabout_dir;
    }
}

void vision_init(void)
{
    reset_lines();
    final_mid_init = 0;
    final_mid_filtered_x8 = VISION_MID_COL * 8;
    vision_reference_col = VISION_MID_COL;
    vision_final_mid = VISION_MID_COL;
    vision_error = 0;
    vision_preview_error = 0;
    vision_far_error = 0;
    vision_curve_error = 0;
    vision_valid = 0;
    vision_confidence = 0;
    vision_valid_line_count = 0;
    vision_lost_left_count = 0;
    vision_lost_right_count = 0;
    vision_cross_flag = 0;
    p_cross_hold_count = 0;
    p_cross_ring_guard_count = 0;
    vision_cross_state = 0;
    vision_cross_left_open_count = 0;
    vision_cross_right_open_count = 0;
    vision_cross_both_open_count = 0;
    vision_debug_left_control = 0;
    vision_debug_right_control = VISION_IMAGE_W - 1;
    vision_debug_mid_control = VISION_MID_COL;
    vision_debug_ring_midpoint = 0;
    vision_debug_ring_mid_under = 0;
    vision_debug_ring_left115 = 0;
    vision_debug_ring_left85 = 0;
    vision_debug_ring_left55 = 0;
    roundabout_reset();
}

void vision_process(const uint8 *image)
{
    get_reference_point(image);
    search_reference_col(image);
    search_line(image);
    p_cross_detect_only();
    if (vision_cross_flag)
    {
        roundabout_reset();
    }
    else
    {
        roundabout_process();
    }
    build_outputs();
}
