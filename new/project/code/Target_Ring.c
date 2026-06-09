#include "Target_Ring.h"
#include "vision.h"

/* =========================================================
 *  雁过留痕圆环识别：多行扫描 + 几何比例约束 + 发射窗口
 *
 *  圆环参数：线宽约 1cm，中心线半径约 5cm
 *  外径约 11cm，内径约 9cm
 *  横向扫描圆环中心附近时：
 *      中间白色 gap / 外宽 outer_w 约 9 / 11 = 0.82
 *      两侧黑线总宽 / 外宽 outer_w 约 2 / 11 = 0.18
 *
 *  重要调参：
 *      TARGET_FIRE_OUTER_W
 *      TARGET_FIRE_ROW
 *  实车标定方法：
 *      车停在“激光刚好能打中圆环中心”的距离，
 *      从 VOFA 记录 target_outer_w 和 target_y，填到下面两个宏。
 * ========================================================= */

#define TARGET_SCAN_ROW_MIN        32
#define TARGET_SCAN_ROW_MAX        108
#define TARGET_SCAN_ROW_STEP       4

#define TARGET_ROAD_MARGIN         4
#define TARGET_ROAD_W_MIN          34
#define TARGET_ROAD_W_MAX          174
#define TARGET_LEFT_LOST_COL       3
#define TARGET_RIGHT_LOST_COL      (VISION_IMAGE_W - 4)
#define TARGET_FALLBACK_HALF_W     50

#define TARGET_RING_MIN_GRAY       115
#define TARGET_RING_MAX_GRAY       190
#define TARGET_GRAY_DELTA          6

#define RING_BAND_W_MIN            2
#define RING_BAND_W_MAX            16
#define RING_GAP_MIN               8
#define RING_GAP_MAX               58
#define RING_OUTER_W_MIN           16
#define RING_OUTER_W_MAX           86

/* 圆环几何比例过滤。单位：百分比。 */
#define RING_GAP_RATIO_MIN         65
#define RING_GAP_RATIO_MAX         92
#define RING_BAND_SUM_RATIO_MIN    8
#define RING_BAND_SUM_RATIO_MAX    38
#define RING_BAND_DIFF_MAX         5

#define TARGET_CENTER_SPREAD_MAX   16
#define TARGET_NEAR_ROW_MIN        76
#define TARGET_NEAR_ROW_MAX        108
#define TARGET_STOP_ROW_MIN        68
#define TARGET_STOP_ROW_MAX        100
#define TARGET_STOP_OUTER_MIN      18
#define TARGET_STOP_OUTER_MAX      82

#define TARGET_FAR_VOTES_MIN       4
#define TARGET_FIRE_VOTES_MIN      2
#define TARGET_RELIABLE_FAR_MIN    2
#define TARGET_RELIABLE_FIRE_MIN   1
#define TARGET_SCORE_FAR_MIN       50
#define TARGET_SCORE_FIRE_MIN      50

/* 发射窗口：二选一或同时启用。默认 outer_w 与 y 任一满足即可。 */
#define TARGET_FIRE_USE_OUTER      1
#define TARGET_FIRE_OUTER_W        48      /* 必须实车标定 */
#define TARGET_FIRE_OUTER_TOL      18

#define TARGET_FIRE_USE_ROW        1
#define TARGET_FIRE_ROW            70      /* 必须实车标定 */
#define TARGET_FIRE_ROW_TOL        14

#define TARGET_AIM_DEADBAND_PIX    5
#define TARGET_STABLE_NEED         2
#define TARGET_STABLE_MAX_MOVE     18
#define TARGET_HOLD_MAX            20      /* 20 * 5ms = 100ms，近距离短暂丢识别时保持 */
#define TARGET_PRESTOP_STABLE_NEED 1

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

volatile uint8 target_far_flag = 0;
volatile uint8 target_stop_ready = 0;
volatile uint8 target_fire_ready = 0;
volatile uint8 target_score = 0;
volatile uint8 target_votes = 0;
volatile uint8 target_reliable_votes = 0;
volatile uint8 target_outer_w = 0;
volatile uint8 target_debug_near_votes = 0;
volatile uint8 target_debug_stop_votes = 0;
volatile uint8 target_debug_reliable_stop_votes = 0;
volatile uint8 target_debug_spread = 0;
volatile uint8 target_debug_fire_window = 0;
volatile uint8 target_debug_stable_count = 0;

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
static uint8 hold_score = 0;
static uint8 hold_votes = 0;
static uint8 hold_reliable_votes = 0;
static uint8 hold_outer_w = 0;
static uint8 prestop_count = 0;

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

static int16 abs_i16(int16 x)
{
    return (x >= 0) ? x : -x;
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

    /* 圆环线条需要比本行平均灰度更暗，否则容易把亮背景噪声识别成圆环。 */
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

static uint8 calc_row_roi(int16 row, int16 *roi_l, int16 *roi_r, uint8 *reliable)
{
    int16 left;
    int16 right;
    int16 width;
    int16 fallback_l;
    int16 fallback_r;

    left = (int16)vision_left_edge_line[row];
    right = (int16)vision_right_edge_line[row];
    width = right - left;

    if (left > TARGET_LEFT_LOST_COL &&
        right < TARGET_RIGHT_LOST_COL &&
        width >= TARGET_ROAD_W_MIN &&
        width <= TARGET_ROAD_W_MAX)
    {
        left += TARGET_ROAD_MARGIN;
        right -= TARGET_ROAD_MARGIN;

        if (left < 4) left = 4;
        if (right > VISION_IMAGE_W - 5) right = VISION_IMAGE_W - 5;

        if (right > left + RING_OUTER_W_MIN)
        {
            *roi_l = left;
            *roi_r = right;
            *reliable = 1;
            return 1;
        }
    }

    /* 远处允许 fallback，提高发现率；近处会在主循环里限制 fallback 使用。 */
    fallback_l = VISION_MID_COL - TARGET_FALLBACK_HALF_W;
    fallback_r = VISION_MID_COL + TARGET_FALLBACK_HALF_W;
    if (fallback_l < 4) fallback_l = 4;
    if (fallback_r > VISION_IMAGE_W - 5) fallback_r = VISION_IMAGE_W - 5;

    if (fallback_r <= fallback_l + RING_OUTER_W_MIN) return 0;

    *roi_l = fallback_l;
    *roi_r = fallback_r;
    *reliable = 0;
    return 1;
}

static uint8 ring_pair_geometry_ok(int16 width1, int16 width2, int16 gap, int16 outer_w)
{
    int16 band_sum;

    if (gap < RING_GAP_MIN || gap > RING_GAP_MAX) return 0;
    if (outer_w < RING_OUTER_W_MIN || outer_w > RING_OUTER_W_MAX) return 0;

    /* 内白 gap / 外径 outer_w，理论约 82%。 */
    if ((gap * 100) < (outer_w * RING_GAP_RATIO_MIN)) return 0;
    if ((gap * 100) > (outer_w * RING_GAP_RATIO_MAX)) return 0;

    /* 两侧黑线总宽 / 外径 outer_w，理论约 18%。 */
    band_sum = width1 + width2;
    if ((band_sum * 100) < (outer_w * RING_BAND_SUM_RATIO_MIN)) return 0;
    if ((band_sum * 100) > (outer_w * RING_BAND_SUM_RATIO_MAX)) return 0;

    if (abs_i16(width1 - width2) > RING_BAND_DIFF_MAX) return 0;

    return 1;
}

static uint8 calc_fire_window_ok(int16 avg_outer, int16 avg_y)
{
    uint8 ok = 0;

#if TARGET_FIRE_USE_OUTER
    if (avg_outer >= (TARGET_FIRE_OUTER_W - TARGET_FIRE_OUTER_TOL) &&
        avg_outer <= (TARGET_FIRE_OUTER_W + TARGET_FIRE_OUTER_TOL))
    {
        ok = 1;
    }
#endif

#if TARGET_FIRE_USE_ROW
    if (avg_y >= (TARGET_FIRE_ROW - TARGET_FIRE_ROW_TOL) &&
        avg_y <= (TARGET_FIRE_ROW + TARGET_FIRE_ROW_TOL))
    {
        ok = 1;
    }
#endif

#if (!TARGET_FIRE_USE_OUTER && !TARGET_FIRE_USE_ROW)
    ok = 1;
#endif

    return ok;
}

static void publish_common(uint8 stage,
                           uint8 flag,
                           uint8 confidence,
                           uint8 far_flag,
                           uint8 stop_ready,
                           uint8 fire_ready,
                           uint8 x,
                           uint8 y,
                           uint8 left,
                           uint8 right,
                           uint8 top,
                           uint8 bottom,
                           int16 error,
                           uint8 score,
                           uint8 votes,
                           uint8 reliable_votes,
                           uint8 outer_w,
                           uint8 near_votes,
                           uint8 stop_votes,
                           uint8 reliable_stop_votes,
                           uint8 spread,
                           uint8 fire_window)
{
    uint8 ea_state;
    uint8 aim_ok = 0;

    if (error <= TARGET_AIM_DEADBAND_PIX && error >= -TARGET_AIM_DEADBAND_PIX)
    {
        aim_ok = 1;
    }

    ea_state = EA;
    EA = 0;

    target_stage = stage;
    target_flag = flag;
    target_confidence = confidence;
    target_far_flag = far_flag;
    target_stop_ready = stop_ready;
    target_fire_ready = fire_ready;
    target_x = x;
    target_y = y;
    target_left = left;
    target_right = right;
    target_top = top;
    target_bottom = bottom;
    target_error_x = error;
    target_servo2_input = error;
    target_aim_ok = aim_ok;
    target_score = score;
    target_votes = votes;
    target_reliable_votes = reliable_votes;
    target_outer_w = outer_w;
    target_debug_near_votes = near_votes;
    target_debug_stop_votes = stop_votes;
    target_debug_reliable_stop_votes = reliable_stop_votes;
    target_debug_spread = spread;
    target_debug_fire_window = fire_window;
    target_debug_stable_count = stable_count;

    EA = ea_state;
}

static void target_publish_lost(uint8 stage)
{
    if (hold_count > 0)
    {
        hold_count--;
        publish_common(stage,
                       1,
                       60,
                       1,
                       0,
                       0,
                       hold_x,
                       hold_y,
                       hold_left,
                       hold_right,
                       hold_top,
                       hold_bottom,
                       hold_error,
                       hold_score,
                       hold_votes,
                       hold_reliable_votes,
                       hold_outer_w,
                       0,
                       0,
                       0,
                       0,
                       0);
    }
    else
    {
        publish_common(stage,
                       0,
                       0,
                       0,
                       0,
                       0,
                       VISION_MID_COL,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0,
                       0);
    }
}

void target_process(const uint8 *image)
{
    int16 row;
    int16 col;
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
    int16 score;
    int16 best_score;
    int16 row_best_x;
    int16 row_best_l;
    int16 row_best_r;
    int16 row_best_outer;
    int16 sum_x = 0;
    int16 sum_y = 0;
    int16 sum_l = 0;
    int16 sum_r = 0;
    int16 sum_outer = 0;
    int16 first_x = -1;
    int16 min_x = 255;
    int16 max_x = 0;
    int16 avg_x;
    int16 avg_y;
    int16 avg_l;
    int16 avg_r;
    int16 avg_outer;
    int16 error;
    uint8 row_avg;
    uint8 in_run;
    uint8 pix;
    uint8 votes = 0;
    uint8 reliable_votes = 0;
    uint8 near_votes = 0;
    uint8 stop_votes = 0;
    uint8 reliable_near_votes = 0;
    uint8 reliable_stop_votes = 0;
    uint8 best_total_score = 0;
    uint8 stable_ok;
    uint8 far_ok;
    uint8 stop_ok;
    uint8 fire_ok;
    uint8 fire_window_ok;
    uint8 confidence;
    uint8 row_reliable;
    uint8 spread;

    if (!target_enable)
    {
        stable_count = 0;
        hold_count = 0;
        prestop_count = 0;
        target_publish_lost(0);
        return;
    }

    for (row = TARGET_SCAN_ROW_MIN; row <= TARGET_SCAN_ROW_MAX; row += TARGET_SCAN_ROW_STEP)
    {
        if (!calc_row_roi(row, &roi_l, &roi_r, &row_reliable))
        {
            continue;
        }

        /* 近距离 fallback ROI 容易误把赛道边缘或阴影当圆环，近处必须依赖赛道边线 ROI。 */
        if (!row_reliable && row > TARGET_NEAR_ROW_MIN)
        {
            continue;
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
            else if (in_run)
            {
                in_run = 0;
                if (run_count < TARGET_MAX_RUNS)
                {
                    run_end[run_count] = col - 1;
                    run_count++;
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

        best_score = -1;
        row_best_x = VISION_MID_COL;
        row_best_l = 0;
        row_best_r = 0;
        row_best_outer = 0;

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

                if (!ring_pair_geometry_ok(width1, width2, gap, outer_w))
                {
                    continue;
                }

                cx = (run_start[i] + run_end[j]) >> 1;

                score = 55;
                score -= abs_i16(width1 - width2) * 3;
                score -= abs_i16(cx - VISION_MID_COL) / 5;
                score -= abs_i16((gap * 100) - (outer_w * 82)) / 35;

                if (is_center_bright(image, row, cx, row_avg)) score += 12;
                if (is_center_bright(image, row, (run_end[i] + run_start[j]) >> 1, row_avg)) score += 8;
                if (row_reliable) score += 8;

                if (score > 100) score = 100;
                if (score < 0) score = 0;

                if (score > best_score)
                {
                    best_score = score;
                    row_best_x = cx;
                    row_best_l = run_start[i];
                    row_best_r = run_end[j];
                    row_best_outer = outer_w;
                }
            }
        }

        if (best_score < 0)
        {
            continue;
        }

        if (first_x < 0)
        {
            first_x = row_best_x;
        }
        else if (abs_diff_u8((uint8)row_best_x, (uint8)first_x) > TARGET_CENTER_SPREAD_MAX)
        {
            continue;
        }

        if (row_best_x < min_x) min_x = row_best_x;
        if (row_best_x > max_x) max_x = row_best_x;

        sum_x += row_best_x;
        sum_y += row;
        sum_l += row_best_l;
        sum_r += row_best_r;
        sum_outer += row_best_outer;
        votes++;

        if (row_reliable)
        {
            reliable_votes++;
        }

        if (best_score > best_total_score) best_total_score = (uint8)best_score;

        if (row >= TARGET_NEAR_ROW_MIN && row <= TARGET_NEAR_ROW_MAX)
        {
            near_votes++;
            if (row_reliable)
            {
                reliable_near_votes++;
            }
        }

        if (row >= TARGET_STOP_ROW_MIN && row <= TARGET_STOP_ROW_MAX &&
            row_best_outer >= TARGET_STOP_OUTER_MIN &&
            row_best_outer <= TARGET_STOP_OUTER_MAX)
        {
            stop_votes++;
            if (row_reliable)
            {
                reliable_stop_votes++;
            }
        }
    }

    if (votes == 0)
    {
        stable_count = 0;
        prestop_count = 0;
        target_publish_lost(7);
        return;
    }

    spread = (uint8)(max_x - min_x);
    if (spread > TARGET_CENTER_SPREAD_MAX)
    {
        stable_count = 0;
        prestop_count = 0;
        target_publish_lost(8);
        return;
    }

    avg_x = sum_x / votes;
    avg_y = sum_y / votes;
    avg_l = sum_l / votes;
    avg_r = sum_r / votes;
    avg_outer = sum_outer / votes;
    error = avg_x - VISION_MID_COL;

    if (abs_diff_u8((uint8)avg_x, last_x) <= TARGET_STABLE_MAX_MOVE &&
        abs_diff_u8((uint8)avg_y, last_y) <= TARGET_STABLE_MAX_MOVE)
    {
        if (stable_count < 255) stable_count++;
    }
    else
    {
        stable_count = 1;
    }

    last_x = (uint8)avg_x;
    last_y = (uint8)avg_y;
    stable_ok = (stable_count >= TARGET_STABLE_NEED) ? 1 : 0;

    far_ok = (votes >= TARGET_FAR_VOTES_MIN &&
              reliable_votes >= TARGET_RELIABLE_FAR_MIN &&
              best_total_score >= TARGET_SCORE_FAR_MIN) ? 1 : 0;

    if (far_ok && reliable_stop_votes >= 1 && stop_votes >= 1)
    {
        if (prestop_count < 255) prestop_count++;
    }
    else
    {
        prestop_count = 0;
    }

    /*
     * 调试版：不要再强依赖 stop_ok/prestop_count。
     * 你这组打印里 fire_window 已经偶尔出现，但 stop_ok 一直为 0，
     * 导致 fire_ready 永远为 0。先让“稳定 + 发射窗口 + 至少 2 行可靠识别”触发开火。
     */
    fire_window_ok = calc_fire_window_ok(avg_outer, avg_y);

    stop_ok = (fire_window_ok &&
               stable_ok &&
               votes >= TARGET_FIRE_VOTES_MIN) ? 1 : 0;

    fire_ok = (fire_window_ok &&
               stable_ok &&
               votes >= TARGET_FIRE_VOTES_MIN &&
               reliable_votes >= TARGET_RELIABLE_FIRE_MIN &&
               best_total_score >= TARGET_SCORE_FIRE_MIN) ? 1 : 0;

    confidence = fire_ok ? 100 : (stop_ok ? 85 : (far_ok ? 70 : 35));

    if (far_ok)
    {
        hold_x = (uint8)avg_x;
        hold_y = (uint8)avg_y;
        hold_left = (uint8)avg_l;
        hold_right = (uint8)avg_r;
        hold_top = (uint8)clamp_i16(avg_y - 2, 0, VISION_IMAGE_H - 1);
        hold_bottom = (uint8)clamp_i16(avg_y + 2, 0, VISION_IMAGE_H - 1);
        hold_error = error;
        hold_score = best_total_score;
        hold_votes = votes;
        hold_reliable_votes = reliable_votes;
        hold_outer_w = (uint8)avg_outer;
        hold_count = TARGET_HOLD_MAX;
    }

    publish_common(fire_ok ? 9 : (stop_ok ? 6 : (far_ok ? 3 : 1)),
                   (far_ok || fire_ok || stop_ok) ? 1 : 0,
                   confidence,
                   far_ok,
                   stop_ok,
                   fire_ok,
                   (uint8)avg_x,
                   (uint8)avg_y,
                   (uint8)avg_l,
                   (uint8)avg_r,
                   (uint8)clamp_i16(avg_y - 2, 0, VISION_IMAGE_H - 1),
                   (uint8)clamp_i16(avg_y + 2, 0, VISION_IMAGE_H - 1),
                   error,
                   best_total_score,
                   votes,
                   reliable_votes,
                   (uint8)avg_outer,
                   near_votes,
                   stop_votes,
                   reliable_stop_votes,
                   spread,
                   fire_window_ok);
}
