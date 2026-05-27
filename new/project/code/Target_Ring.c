#include "Target_Ring.h"
#include "vision.h"

#define TARGET_LOCK_ROW            92
#define TARGET_ROW_RADIUS          8
#define TARGET_COL_MARGIN          8

#define TARGET_RING_MIN_GRAY       120
#define TARGET_RING_MAX_GRAY       185
#define TARGET_GRAY_DELTA          16

#define RING_BAND_W_MIN            2
#define RING_BAND_W_MAX            16
#define RING_GAP_MIN               8
#define RING_GAP_MAX               52
#define RING_OUTER_W_MIN           18
#define RING_OUTER_W_MAX           82

#define TARGET_AIM_DEADBAND_PIX    3
#define TARGET_STABLE_NEED         2
#define TARGET_STABLE_MAX_MOVE     8
#define TARGET_HOLD_MAX            8

#define TARGET_MAX_RUNS            10

volatile uint8 target_enable = 1;
volatile uint8 target_flag = 0;
volatile uint8 target_stage = 0;
volatile uint8 target_confidence = 0;

volatile uint8 target_x = VISION_MID_COL;
volatile uint8 target_y = 0;
volatile uint8 target_left = 0;
volatile uint8 target_right = 0;
volatile uint8 target_top = 0;
volatile uint8 target_bottom = 0;

volatile int16 target_error_x = 0;
volatile int16 target_servo2_input = 0;
volatile uint8 target_aim_ok = 0;

uint8 target_gray_th = 0;

static uint8 stable_count = 0;
static uint8 last_x = VISION_MID_COL;
static uint8 last_y = 0;

static uint8 hold_count = 0;
static uint8 hold_x = VISION_MID_COL;
static uint8 hold_y = 0;
static uint8 hold_left = 0;
static uint8 hold_right = 0;
static uint8 hold_top = 0;
static uint8 hold_bottom = 0;
static int16 hold_error = 0;

static int16 clamp_i16(int16 x, int16 min_v, int16 max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

static uint8 abs_diff_u8(uint8 a, uint8 b)
{
    return (a > b) ? (a - b) : (b - a);
}

static uint8 image_pix(const uint8 *image, int16 row, int16 col)
{
    if (row < 0 || row >= VISION_IMAGE_H || col < 0 || col >= VISION_IMAGE_W)
    {
        return 255;
    }

    return *(image + row * VISION_IMAGE_W + col);
}

static uint8 is_ring_gray(uint8 pix, uint8 row_avg)
{
    if (pix < TARGET_RING_MIN_GRAY) return 0;
    if (pix > TARGET_RING_MAX_GRAY) return 0;
    if ((uint16)pix + TARGET_GRAY_DELTA >= row_avg) return 0;

    return 1;
}

static uint8 is_center_bright(const uint8 *image, int16 row, int16 col, uint8 row_avg)
{
    uint8 pix;

    pix = image_pix(image, row, col);

    if (pix > (uint8)clamp_i16((int16)row_avg - 10, 0, 255))
    {
        return 1;
    }

    return 0;
}

static uint8 calc_row_avg(const uint8 *image, int16 row, int16 left, int16 right)
{
    int16 col;
    uint16 count = 0;
    uint32 sum = 0;

    for (col = left; col <= right; col += 2)
    {
        sum += image_pix(image, row, col);
        count++;
    }

    if (count == 0) return 180;
    return (uint8)(sum / count);
}

static void target_publish_lost(uint8 stage)
{
    uint8 ea_state;

    ea_state = EA;
    EA = 0;

    target_stage = stage;
    target_aim_ok = 0;

    if (hold_count > 0)
    {
        hold_count--;

        target_flag = 1;
        target_confidence = 60;
        target_x = hold_x;
        target_y = hold_y;
        target_left = hold_left;
        target_right = hold_right;
        target_top = hold_top;
        target_bottom = hold_bottom;
        target_error_x = hold_error;
        target_servo2_input = hold_error;
    }
    else
    {
        target_flag = 0;
        target_confidence = 0;
    }

    EA = ea_state;
}

static void target_publish_valid(uint8 x, uint8 y, uint8 left, uint8 right,
                                 uint8 top, uint8 bottom, int16 error,
                                 uint8 stable_ok)
{
    uint8 ea_state;
    uint8 aim_ok = 0;

    if (error <= TARGET_AIM_DEADBAND_PIX && error >= -TARGET_AIM_DEADBAND_PIX)
    {
        aim_ok = 1;
    }

    ea_state = EA;
    EA = 0;

    target_stage = 9;
    target_x = x;
    target_y = y;
    target_left = left;
    target_right = right;
    target_top = top;
    target_bottom = bottom;
    target_error_x = error;
    target_servo2_input = error;
    target_aim_ok = aim_ok;

    if (stable_ok)
    {
        target_flag = 1;
        target_confidence = 100;

        hold_x = x;
        hold_y = y;
        hold_left = left;
        hold_right = right;
        hold_top = top;
        hold_bottom = bottom;
        hold_error = error;
        hold_count = TARGET_HOLD_MAX;
    }
    else
    {
        target_flag = 0;
        target_confidence = 60;
    }

    EA = ea_state;
}

void target_process(const uint8 *image)
{
    int16 row;
    int16 col;
    int16 row_start;
    int16 row_end;
    int16 roi_l;
    int16 roi_r;

    int16 run_start[TARGET_MAX_RUNS];
    int16 run_end[TARGET_MAX_RUNS];
    int16 run_count;

    int16 i;
    int16 j;
    int16 width1;
    int16 width2;
    int16 gap;
    int16 outer_w;
    int16 cx;
    int16 cy;
    int16 best_x = VISION_MID_COL;
    int16 best_y = TARGET_LOCK_ROW;
    int16 best_l = 0;
    int16 best_r = 0;
    int16 best_score = -1;
    int16 score;
    int16 error;

    uint8 row_avg;
    uint8 in_run;
    uint8 pix;
    uint8 stable_ok;

    if (!target_enable)
    {
        stable_count = 0;
        hold_count = 0;
        target_publish_lost(0);
        return;
    }

    row_start = TARGET_LOCK_ROW - TARGET_ROW_RADIUS;
    row_end = TARGET_LOCK_ROW + TARGET_ROW_RADIUS;

    if (row_start < VISION_STOP_ROW) row_start = VISION_STOP_ROW;
    if (row_end >= VISION_IMAGE_H) row_end = VISION_IMAGE_H - 1;

    for (row = row_start; row <= row_end; row++)
    {
        roi_l = (int16)vision_left_control_line[row] + TARGET_COL_MARGIN;
        roi_r = (int16)vision_right_control_line[row] - TARGET_COL_MARGIN;

        if (roi_l < 4) roi_l = 4;
        if (roi_r > VISION_IMAGE_W - 5) roi_r = VISION_IMAGE_W - 5;

        if (roi_r <= roi_l + RING_OUTER_W_MIN)
        {
            roi_l = 4;
            roi_r = VISION_IMAGE_W - 5;
        }

        row_avg = calc_row_avg(image, row, roi_l, roi_r);
        target_gray_th = row_avg;

        run_count = 0;
        in_run = 0;

        for (col = roi_l; col <= roi_r; col++)
        {
            pix = image_pix(image, row, col);

            if (is_ring_gray(pix, row_avg))
            {
                if (!in_run)
                {
                    in_run = 1;
                    if (run_count < TARGET_MAX_RUNS)
                    {
                        run_start[run_count] = col;
                    }
                }
            }
            else
            {
                if (in_run)
                {
                    in_run = 0;
                    if (run_count < TARGET_MAX_RUNS)
                    {
                        run_end[run_count] = col - 1;
                        run_count++;
                    }
                }
            }
        }

        if (in_run && run_count < TARGET_MAX_RUNS)
        {
            run_end[run_count] = roi_r;
            run_count++;
        }

        if (run_count < 2)
        {
            continue;
        }

        for (i = 0; i < run_count - 1; i++)
        {
            width1 = run_end[i] - run_start[i] + 1;
            if (width1 < RING_BAND_W_MIN || width1 > RING_BAND_W_MAX)
            {
                continue;
            }

            for (j = i + 1; j < run_count; j++)
            {
                width2 = run_end[j] - run_start[j] + 1;
                if (width2 < RING_BAND_W_MIN || width2 > RING_BAND_W_MAX)
                {
                    continue;
                }

                gap = run_start[j] - run_end[i] - 1;
                outer_w = run_end[j] - run_start[i] + 1;

                if (gap < RING_GAP_MIN || gap > RING_GAP_MAX)
                {
                    continue;
                }

                if (outer_w < RING_OUTER_W_MIN || outer_w > RING_OUTER_W_MAX)
                {
                    continue;
                }

                cx = (run_start[i] + run_end[j]) >> 1;
                cy = row;

                score = 0;

                score += 50;
                score -= (abs_diff_u8((uint8)cy, TARGET_LOCK_ROW) * 2);
                score -= abs_diff_u8((uint8)width1, (uint8)width2);
                score -= abs_diff_u8((uint8)cx, VISION_MID_COL) / 3;

                if (is_center_bright(image, row, cx, row_avg)) score += 15;
                if (is_center_bright(image, row, (run_end[i] + run_start[j]) >> 1, row_avg)) score += 10;

                if (score > best_score)
                {
                    best_score = score;
                    best_x = cx;
                    best_y = cy;
                    best_l = run_start[i];
                    best_r = run_end[j];
                }
            }
        }
    }

    if (best_score < 0)
    {
        stable_count = 0;
        target_publish_lost(7);
        return;
    }

    if (abs_diff_u8((uint8)best_x, last_x) <= TARGET_STABLE_MAX_MOVE &&
        abs_diff_u8((uint8)best_y, last_y) <= TARGET_STABLE_MAX_MOVE)
    {
        if (stable_count < 255) stable_count++;
    }
    else
    {
        stable_count = 1;
    }

    last_x = (uint8)best_x;
    last_y = (uint8)best_y;

    stable_ok = (stable_count >= TARGET_STABLE_NEED) ? 1 : 0;
    error = (int16)best_x - (int16)VISION_MID_COL;

    target_publish_valid((uint8)best_x,
                         (uint8)best_y,
                         (uint8)best_l,
                         (uint8)best_r,
                         (uint8)(best_y - 1),
                         (uint8)(best_y + 1),
                         error,
                         stable_ok);
}