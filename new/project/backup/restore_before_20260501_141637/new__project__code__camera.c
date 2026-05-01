#include "camera.h"
#include "zf_common_headfile.h"

// ===================== 可调参数（视觉层） =====================
#define CAMERA_DEBUG_DRAW_ENABLE         0   // 0=关闭显示提帧率，1=开启显示便于调试
#define CAMERA_DEBUG_DRAW_INTERVAL       6
#define CAMERA_BINARY_SNAPSHOT_ENABLE    (CAMERA_DEBUG_DRAW_ENABLE || (IPS200_OR_WIFI == 1))

// 性能档位：0=稳健，1=均衡，2=高帧率（推荐）
#define CAMERA_PERF_PROFILE              2

#if (CAMERA_PERF_PROFILE == 0)
#define CAMERA_OTSU_SAMPLE_STEP          4
#define CAMERA_OTSU_UPDATE_INTERVAL      4
#define CAMERA_LOCAL_RANGE_MIN           14
#define CAMERA_LOCAL_RANGE_MAX           36
#define CAMERA_GLOBAL_RESCAN_NEAR_ROW    96
#define CAMERA_ROW_STEP                  2
#define CAMERA_MID_FILT_ALPHA            0.72f
#define CAMERA_FINAL_MID_STEP_MIN        4
#define CAMERA_FINAL_MID_STEP_MAX        16
#define CAMERA_RESCAN_STREAK_TH          1
#define CAMERA_RESCAN_FORCE_NEAR_ROW     114
#define CAMERA_RESCAN_CONF_TH            50
#elif (CAMERA_PERF_PROFILE == 1)
#define CAMERA_OTSU_SAMPLE_STEP          5
#define CAMERA_OTSU_UPDATE_INTERVAL      5
#define CAMERA_LOCAL_RANGE_MIN           12
#define CAMERA_LOCAL_RANGE_MAX           32
#define CAMERA_GLOBAL_RESCAN_NEAR_ROW    106
#define CAMERA_ROW_STEP                  2
#define CAMERA_MID_FILT_ALPHA            0.70f
#define CAMERA_FINAL_MID_STEP_MIN        5
#define CAMERA_FINAL_MID_STEP_MAX        17
#define CAMERA_RESCAN_STREAK_TH          2
#define CAMERA_RESCAN_FORCE_NEAR_ROW     116
#define CAMERA_RESCAN_CONF_TH            45
#else
#define CAMERA_OTSU_SAMPLE_STEP          6   // OTSU采样更稀疏，显著降算力
#define CAMERA_OTSU_UPDATE_INTERVAL      6   // 阈值更新降频，减少每秒重计算次数
#define CAMERA_LOCAL_RANGE_MIN           12  // 缩小局部搜索窗口
#define CAMERA_LOCAL_RANGE_MAX           28
#define CAMERA_GLOBAL_RESCAN_NEAR_ROW    112 // 全局补扫只保留最靠近车体的区域
#define CAMERA_ROW_STEP                  3   // 按3行处理，帧率提升最明显
#define CAMERA_MID_FILT_ALPHA            0.66f
#define CAMERA_FINAL_MID_STEP_MIN        6
#define CAMERA_FINAL_MID_STEP_MAX        18
#define CAMERA_RESCAN_STREAK_TH          2   // 连续丢线才触发全局补扫
#define CAMERA_RESCAN_FORCE_NEAR_ROW     116 // 最近场仍强制补扫，防止贴边漏检
#define CAMERA_RESCAN_CONF_TH            45
#endif

#define CAMERA_TOP_THRESHOLD_GAIN        14  // 上半区阈值补偿，抑制远处反光

#define CAMERA_WIDTH_MIN                 18
#define CAMERA_WIDTH_MAX                 (MT9V03X_W - 4)
#define CAMERA_EDGE_MIN_GAP              8

#define CAMERA_GLOBAL_RESCAN_ENABLE      1   // 1=局部丢线后启用全行补扫

#define CAMERA_WIDTH_FILT_ALPHA          0.24f
#define CAMERA_MID_SEED_BLEND_NUM        2   // mid_seed = (a*old + new)/(a+1)

#define CAMERA_NEAR_ROW                  (MT9V03X_H - 5)

#define CAMERA_RING_ROW_MIN              50
#define CAMERA_RING_ROW_MAX              108
#define CAMERA_RING_OPEN_HITS_TH         7
#define CAMERA_RING_DOMINATE_TH          5
#define CAMERA_RING_HOLD_FRAMES          28
#define CAMERA_RING_MID_BIAS_BASE        3
#define CAMERA_RING_MID_BIAS_MAX         12
#define CAMERA_RING_PREVIEW_BIAS         6

#define CAMERA_S_MID_ROW                 46
#define CAMERA_S_FAR_ROW                 28
#define CAMERA_S_CURVE_TH                7

#define WIFI_SSID_TEST                   "Car"
#define WIFI_PASSWORD_TEST               "431431431"
#define WIFI_BOUNDARY_ENABLE             1

// ===================== 全局输出 =====================
uint8 img_threshold = 120;
uint8 left_jidian = 1;
uint8 right_jidian = MT9V03X_W - 2;
uint8 left_line_list[MT9V03X_H];
uint8 right_line_list[MT9V03X_H];
uint8 mid_line_list[MT9V03X_H];
volatile uint8 final_mid_line = MID_W;

volatile int16 camera_bias_raw = 0;
volatile int16 camera_preview_raw = 0;
volatile uint8 camera_valid_line_cnt = 0;
volatile uint8 camera_lost_left_cnt = 0;
volatile uint8 camera_lost_right_cnt = 0;
volatile uint8 camera_confidence = 0;
volatile int16 camera_preview_far_raw = 0;
volatile int8 camera_ring_dir = 0;
volatile uint8 camera_ring_score = 0;
volatile uint8 camera_s_curve_flag = 0;

// ===================== 模块内部变量 =====================
#if CAMERA_BINARY_SNAPSHOT_ENABLE
static uint8 bin_image[MT9V03X_H][MT9V03X_W];
#endif
static float final_mid_filtered = (float)MID_W;
static uint8 final_mid_init = 0;
static int8 ring_hold_dir = 0;
static uint8 ring_hold_cnt = 0;

volatile uint16 current_fps = 0;
volatile uint16 fps_counter = 0;
static uint8 skip_draw = 0;
static uint8 otsu_update_cnt = 0;

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

static uint8 threshold_for_row(int row)
{
    if (row < (MT9V03X_H / 3))
    {
        return (img_threshold > (255 - CAMERA_TOP_THRESHOLD_GAIN)) ? 255 : (uint8)(img_threshold + CAMERA_TOP_THRESHOLD_GAIN);
    }
    return img_threshold;
}

static uint8 binary_pixel(int row, int x)
{
#if CAMERA_BINARY_SNAPSHOT_ENABLE
    return bin_image[row][x];
#else
    return (mt9v03x_image[row][x] >= threshold_for_row(row)) ? 255 : 0;
#endif
}

static uint8 is_left_transition(int row, int x)
{
    if (x < 2 || x > MT9V03X_W - 3) return 0;
    if ((binary_pixel(row, x - 1) == 0) && (binary_pixel(row, x) == 255) && (binary_pixel(row, x + 1) == 255))
    {
        return 1;
    }
    return 0;
}

static uint8 is_right_transition(int row, int x)
{
    if (x < 2 || x > MT9V03X_W - 3) return 0;
    if ((binary_pixel(row, x - 1) == 255) && (binary_pixel(row, x) == 255) && (binary_pixel(row, x + 1) == 0))
    {
        return 1;
    }
    return 0;
}
void mark_frame_processed(void)
{
    fps_counter++;
}

void my_fps_timer_callback(void)
{
    static uint16 time_ms = 0;
    time_ms++;
    if (time_ms >= 1000)
    {
        current_fps = fps_counter;
        fps_counter = 0;
        time_ms = 0;
    }
}

// OTSU阈值：对灰度图做步进采样，平衡精度与算力
static uint8 calc_otsu_threshold_sampling(void)
{
    uint16 hist[256];
    uint32 sum = 0;
    uint32 sum_b = 0;
    uint32 w_b = 0;
    uint32 w_f = 0;
    uint32 total = 0;
    uint16 y, x;
    uint16 i;
    float max_var = -1.0f;
    uint8 threshold = img_threshold;
    float mean_b;
    float mean_f;
    float diff;
    float var_between;
    uint8 g;

    for (i = 0; i < 256; i++) hist[i] = 0;

    for (y = 0; y < MT9V03X_H; y += CAMERA_OTSU_SAMPLE_STEP)
    {
        for (x = 0; x < MT9V03X_W; x += CAMERA_OTSU_SAMPLE_STEP)
        {
            g = mt9v03x_image[y][x];
            hist[g]++;
            total++;
        }
    }

    if (total == 0) return img_threshold;

    for (i = 0; i < 256; i++)
    {
        sum += (uint32)i * hist[i];
    }

    for (i = 0; i < 256; i++)
    {
        w_b += hist[i];
        if (w_b == 0) continue;

        w_f = total - w_b;
        if (w_f == 0) break;

        sum_b += (uint32)i * hist[i];
        mean_b = (float)sum_b / (float)w_b;
        mean_f = (float)(sum - sum_b) / (float)w_f;
        diff = mean_b - mean_f;
        var_between = (float)w_b * (float)w_f * diff * diff;

        if (var_between > max_var)
        {
            max_var = var_between;
            threshold = (uint8)i;
        }
    }

    return threshold;
}

// 灰度快照+二值快照：避免DMA更新造成的时空撕裂
#if CAMERA_BINARY_SNAPSHOT_ENABLE
static void make_binary_snapshot(void)
{
    uint16 y, x;
    uint8 th_top;
    uint8 th_bot;
    uint16 top_end;
    uint8 th;
    uint8 g;

    th_bot = img_threshold;
    th_top = (img_threshold > (255 - CAMERA_TOP_THRESHOLD_GAIN)) ? 255 : (img_threshold + CAMERA_TOP_THRESHOLD_GAIN);
    top_end = MT9V03X_H / 3;

    for (y = 0; y < MT9V03X_H; y++)
    {
        th = (y < top_end) ? th_top : th_bot;
        for (x = 0; x < MT9V03X_W; x++)
        {
            g = mt9v03x_image[y][x];
            bin_image[y][x] = (g >= th) ? 255 : 0;
        }
    }
}
#endif

// 根据行号估计赛道半宽（近处更宽，远处更窄）
static int estimate_half_width(int row)
{
    int num;
    int den;
    int val;

    den = (search_start_line - search_end_line);
    if (den <= 0) den = 1;

    num = (row - search_end_line);
    if (num < 0) num = 0;
    if (num > den) num = den;

    // 远处约44，近处约78
    val = 44 + (34 * num) / den;
    val = clamp_i(val, 42, 80);
    return val;
}

// 局部窗口找左边缘：优先贴近ref_x，减小跳变
static int find_left_edge_local(int row, int ref_x, int range)
{
    int x;
    int start_x;
    int end_x;
    int best_x;
    int best_dist;
    int dist;

    start_x = clamp_i(ref_x - range, 2, MT9V03X_W - 3);
    end_x = clamp_i(ref_x + range, 2, MT9V03X_W - 3);

    best_x = -1;
    best_dist = 10000;

    for (x = start_x; x <= end_x; x++)
    {
        if (is_left_transition(row, x))
        {
            dist = abs_i(x - ref_x);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_x = x;
            }
        }
    }

    return best_x;
}

// 局部窗口找右边缘：优先贴近ref_x，减小跳变
static int find_right_edge_local(int row, int ref_x, int range)
{
    int x;
    int start_x;
    int end_x;
    int best_x;
    int best_dist;
    int dist;

    start_x = clamp_i(ref_x - range, 2, MT9V03X_W - 3);
    end_x = clamp_i(ref_x + range, 2, MT9V03X_W - 3);

    best_x = -1;
    best_dist = 10000;

    for (x = start_x; x <= end_x; x++)
    {
        if (is_right_transition(row, x))
        {
            dist = abs_i(x - ref_x);
            if (dist < best_dist)
            {
                best_dist = dist;
                best_x = x;
            }
        }
    }

    return best_x;
}

// 全行补扫左边缘：用于大角度弯道局部搜索失败兜底
static int find_left_edge_global(int row)
{
    int x;
    for (x = 2; x <= MT9V03X_W - 3; x++)
    {
        if (is_left_transition(row, x))
        {
            return x;
        }
    }
    return -1;
}

// 全行补扫右边缘：用于大角度弯道局部搜索失败兜底
static int find_right_edge_global(int row)
{
    int x;
    for (x = MT9V03X_W - 3; x >= 2; x--)
    {
        if (is_right_transition(row, x))
        {
            return x;
        }
    }
    return -1;
}

// 整行搜索基点，保证大弯贴边时也能起步
void find_jidian(void)
{
    int y;
    int left_x;
    int right_x;

    y = jidian_search_line - 1;
    y = clamp_i(y, search_end_line + 1, MT9V03X_H - 1);

    left_x = find_left_edge_global(y);
    right_x = find_right_edge_global(y);

    if (left_x < 0) left_x = 1;
    if (right_x < 0) right_x = MT9V03X_W - 2;

    left_jidian = (uint8)left_x;
    right_jidian = (uint8)right_x;
}

// 主寻线：逐行提取左右边界，并在单边丢线时做宽度模型重建
void image_deal(void)
{
    int row;
    int left_now;
    int right_now;
    int left_prev;
    int right_prev;
    int mid_seed;
    int left_found;
    int right_found;
    int width_meas;
    int width_est;
    float width_filtered;
    int both_valid;
    int total_rows;
    int mid_now;
    int default_half;
    int local_range;
    int left_rebuilt;
    int right_rebuilt;
    int fix_mid;
    int fix_half;
    int fill_row;
    int miss_streak;
    uint8 prev_low_conf;
    int left_open_hits;
    int right_open_hits;
    int expected_width;
    int open_bias;
    int ring_score;

    for (row = 0; row < MT9V03X_H; row++)
    {
        left_line_list[row] = 1;
        right_line_list[row] = MT9V03X_W - 2;
        mid_line_list[row] = MID_W;
    }

    camera_valid_line_cnt = 0;
    camera_lost_left_cnt = 0;
    camera_lost_right_cnt = 0;
    total_rows = 0;

    left_prev = (int)left_jidian;
    right_prev = (int)right_jidian;

    if ((right_prev - left_prev) < CAMERA_EDGE_MIN_GAP)
    {
        left_prev = MID_W - 30;
        right_prev = MID_W + 30;
    }

    left_prev = clamp_i(left_prev, 1, MT9V03X_W - 8);
    right_prev = clamp_i(right_prev, left_prev + CAMERA_EDGE_MIN_GAP, MT9V03X_W - 2);
    width_filtered = (float)(right_prev - left_prev);
    mid_seed = (left_prev + right_prev) / 2;
    miss_streak = 0;
    prev_low_conf = (camera_confidence <= CAMERA_RESCAN_CONF_TH) ? 1 : 0;
    left_open_hits = 0;
    right_open_hits = 0;

    for (row = search_start_line - 1; row > search_end_line; row -= CAMERA_ROW_STEP)
    {
        total_rows += CAMERA_ROW_STEP;
        both_valid = 0;
        left_rebuilt = 0;
        right_rebuilt = 0;

        local_range = 18 + (search_start_line - row) / 5;
        local_range = clamp_i(local_range, CAMERA_LOCAL_RANGE_MIN, CAMERA_LOCAL_RANGE_MAX);

        left_found = find_left_edge_local(row, left_prev, local_range);
        right_found = find_right_edge_local(row, right_prev, local_range);

        // 局部都没找到时，用中线种子做一次扩大窗口搜索
        if ((left_found < 0) && (right_found < 0))
        {
            left_found = find_left_edge_local(row, mid_seed, local_range + 10);
            right_found = find_right_edge_local(row, mid_seed, local_range + 10);
        }

        if ((left_found < 0) || (right_found < 0))
        {
            miss_streak++;
        }
        else
        {
            miss_streak = 0;
        }

#if CAMERA_GLOBAL_RESCAN_ENABLE
        if ((row >= CAMERA_GLOBAL_RESCAN_NEAR_ROW) &&
            ((miss_streak >= CAMERA_RESCAN_STREAK_TH) ||
             (row >= CAMERA_RESCAN_FORCE_NEAR_ROW) ||
             prev_low_conf))
        {
            if (left_found < 0)
            {
                left_found = find_left_edge_global(row);
            }
            if (right_found < 0)
            {
                right_found = find_right_edge_global(row);
            }
        }
#endif

        if ((left_found >= 0) && (right_found >= 0) && ((right_found - left_found) >= CAMERA_EDGE_MIN_GAP))
        {
            left_now = left_found;
            right_now = right_found;
            both_valid = 1;
        }
        else if ((left_found >= 0) && (right_found < 0))
        {
            width_est = (int)(width_filtered + 0.5f);
            left_now = left_found;
            right_now = left_found + width_est;
            right_rebuilt = 1;
        }
        else if ((right_found >= 0) && (left_found < 0))
        {
            width_est = (int)(width_filtered + 0.5f);
            right_now = right_found;
            left_now = right_found - width_est;
            left_rebuilt = 1;
        }
        else
        {
            default_half = estimate_half_width(row);
            mid_now = mid_seed;
            left_now = mid_now - default_half;
            right_now = mid_now + default_half;
            left_rebuilt = 1;
            right_rebuilt = 1;
        }

        left_now = clamp_i(left_now, 1, MT9V03X_W - 3);
        right_now = clamp_i(right_now, 2, MT9V03X_W - 2);

        if ((right_now - left_now) < CAMERA_EDGE_MIN_GAP)
        {
            fix_mid = (left_now + right_now) / 2;
            fix_half = estimate_half_width(row);
            left_now = clamp_i(fix_mid - fix_half, 1, MT9V03X_W - 3);
            right_now = clamp_i(fix_mid + fix_half, 2, MT9V03X_W - 2);
            if (left_found < 0) left_rebuilt = 1;
            if (right_found < 0) right_rebuilt = 1;
        }

        width_meas = right_now - left_now;
        if ((width_meas >= CAMERA_WIDTH_MIN) && (width_meas <= CAMERA_WIDTH_MAX))
        {
            width_filtered = width_filtered * (1.0f - CAMERA_WIDTH_FILT_ALPHA) + (float)width_meas * CAMERA_WIDTH_FILT_ALPHA;
        }

        if ((row >= CAMERA_RING_ROW_MIN) && (row <= CAMERA_RING_ROW_MAX))
        {
            expected_width = estimate_half_width(row) * 2;
            open_bias = ((left_now + right_now) / 2) - MID_W;

            if (((left_rebuilt && !right_rebuilt) || (left_now <= 2) ||
                 ((width_meas > expected_width + 18) && (open_bias < -10))))
            {
                left_open_hits++;
            }
            if (((right_rebuilt && !left_rebuilt) || (right_now >= MT9V03X_W - 3) ||
                 ((width_meas > expected_width + 18) && (open_bias > 10))))
            {
                right_open_hits++;
            }
        }

        left_line_list[row] = (uint8)left_now;
        right_line_list[row] = (uint8)right_now;
        mid_now = (left_now + right_now) / 2;
        mid_line_list[row] = (uint8)clamp_i(mid_now, 1, MT9V03X_W - 2);

        // 行步进时，把中间未处理行用当前结果填充，保持中线连续
        if (CAMERA_ROW_STEP > 1)
        {
            for (fill_row = row + 1; (fill_row < row + CAMERA_ROW_STEP) && (fill_row < search_start_line); fill_row++)
            {
                if (fill_row > search_end_line)
                {
                    left_line_list[fill_row] = left_line_list[row];
                    right_line_list[fill_row] = right_line_list[row];
                    mid_line_list[fill_row] = mid_line_list[row];
                }
            }
        }

        if (both_valid)
        {
            if (camera_valid_line_cnt <= (uint8)(255 - CAMERA_ROW_STEP))
            {
                camera_valid_line_cnt += CAMERA_ROW_STEP;
            }
            else
            {
                camera_valid_line_cnt = 255;
            }
        }
        if (left_rebuilt)
        {
            if (camera_lost_left_cnt < 255) camera_lost_left_cnt++;
        }
        if (right_rebuilt)
        {
            if (camera_lost_right_cnt < 255) camera_lost_right_cnt++;
        }

        left_prev = left_now;
        right_prev = right_now;

        mid_seed = (CAMERA_MID_SEED_BLEND_NUM * mid_seed + mid_now) / (CAMERA_MID_SEED_BLEND_NUM + 1);
        mid_seed = clamp_i(mid_seed, 2, MT9V03X_W - 3);
    }

    if (total_rows <= 0)
    {
        camera_confidence = 0;
    }
    else
    {
        int conf;
        conf = (int)camera_valid_line_cnt * 100 / total_rows;

        // 丢线越多，置信度越低
        conf -= (int)camera_lost_left_cnt / 2;
        conf -= (int)camera_lost_right_cnt / 2;
        conf = clamp_i(conf, 0, 100);
        camera_confidence = (uint8)conf;
    }

    if ((left_open_hits >= CAMERA_RING_OPEN_HITS_TH) &&
        (left_open_hits >= right_open_hits + CAMERA_RING_DOMINATE_TH))
    {
        ring_hold_dir = -1;
        ring_hold_cnt = CAMERA_RING_HOLD_FRAMES;
        ring_score = left_open_hits;
    }
    else if ((right_open_hits >= CAMERA_RING_OPEN_HITS_TH) &&
             (right_open_hits >= left_open_hits + CAMERA_RING_DOMINATE_TH))
    {
        ring_hold_dir = 1;
        ring_hold_cnt = CAMERA_RING_HOLD_FRAMES;
        ring_score = right_open_hits;
    }
    else
    {
        ring_score = (left_open_hits > right_open_hits) ? left_open_hits : right_open_hits;
        if (ring_hold_cnt > 0)
        {
            ring_hold_cnt--;
        }
        else
        {
            ring_hold_dir = 0;
        }
    }

    camera_ring_dir = ring_hold_dir;
    camera_ring_score = (uint8)clamp_i(ring_score, 0, 100);
}

// 根据中线数组计算最终中线输出，并生成给control的视觉特征
uint8 find_mid_line_weight(void)
{
    int row;
    uint32 sum_mid = 0;
    uint32 sum_w = 0;
    int width_now;
    int row_weight;
    int near_y;
    int far_y;
    int mid_new;
    int rough_preview;
    int abs_preview;
    int step_limit;
    float delta;
    float target_mid;
    int preview_tmp;
    int preview_mid_tmp;
    int preview_far_tmp;
    int mid_y;
    int far2_y;
    int ring_bias;
    int8 ring_dir_tmp;
    uint8 ring_score_tmp;
    uint8 s_curve_tmp;
    int16 preview_far_out;
    int16 bias_tmp;
    uint8 final_mid_tmp;
    uint8 ea_state;

    near_y = clamp_i(CAMERA_NEAR_ROW, search_end_line + 1, MT9V03X_H - 1);

    // 先用固定行做一版粗前瞻，再动态决定far_y
    mid_y = clamp_i(CAMERA_S_MID_ROW, search_end_line + 1, MT9V03X_H - 1);
    far2_y = clamp_i(CAMERA_S_FAR_ROW, search_end_line + 1, MT9V03X_H - 1);
    rough_preview = (int)mid_line_list[32] - (int)mid_line_list[near_y];
    preview_mid_tmp = (int)mid_line_list[mid_y] - (int)mid_line_list[near_y];
    preview_far_tmp = (int)mid_line_list[far2_y] - (int)mid_line_list[near_y];
    abs_preview = abs_i(rough_preview);
    s_curve_tmp = 0;

    if (((preview_mid_tmp > CAMERA_S_CURVE_TH) && (preview_far_tmp < -CAMERA_S_CURVE_TH)) ||
        ((preview_mid_tmp < -CAMERA_S_CURVE_TH) && (preview_far_tmp > CAMERA_S_CURVE_TH)))
    {
        s_curve_tmp = 1;
        far_y = far2_y;
    }
    else if (abs_preview >= 12)
    {
        far_y = 38;
    }
    else if (abs_preview >= 6)
    {
        far_y = 34;
    }
    else
    {
        far_y = 28;
    }

    if ((camera_confidence < 50) && (!s_curve_tmp))
    {
        far_y += 4;
    }

    far_y = clamp_i(far_y, search_end_line + 1, MT9V03X_H - 1);

    for (row = search_start_line - 1; row > search_end_line; row -= CAMERA_ROW_STEP)
    {
        if (row >= (MT9V03X_H - 10)) row_weight = 18;
        else if (row >= (MT9V03X_H - 24)) row_weight = 14;
        else if (row >= 80) row_weight = 10;
        else if (row >= 60) row_weight = 6;
        else row_weight = 3;

        width_now = (int)right_line_list[row] - (int)left_line_list[row];
        if ((width_now < CAMERA_WIDTH_MIN) || (width_now > CAMERA_WIDTH_MAX))
        {
            row_weight = (row_weight * 5) / 10;
        }

        if ((left_line_list[row] <= 2) || (right_line_list[row] >= MT9V03X_W - 3))
        {
            row_weight = (row_weight * 7) / 10;
        }

        sum_mid += (uint32)mid_line_list[row] * (uint32)row_weight;
        sum_w += (uint32)row_weight;
    }

    if (sum_w == 0)
    {
        mid_new = MID_W;
    }
    else
    {
        mid_new = (int)(sum_mid / sum_w);
    }

    ring_dir_tmp = camera_ring_dir;
    ring_score_tmp = camera_ring_score;
    if (s_curve_tmp)
    {
        ring_dir_tmp = 0;
        ring_score_tmp = 0;
    }
    if (ring_dir_tmp != 0)
    {
        ring_bias = CAMERA_RING_MID_BIAS_BASE + (int)ring_score_tmp;
        ring_bias = clamp_i(ring_bias, CAMERA_RING_MID_BIAS_BASE, CAMERA_RING_MID_BIAS_MAX);
        mid_new += (int)ring_dir_tmp * ring_bias;
    }

    mid_new = clamp_i(mid_new, 1, MT9V03X_W - 2);

    if (!final_mid_init)
    {
        final_mid_filtered = (float)mid_new;
        final_mid_init = 1;
    }
    else
    {
        step_limit = CAMERA_FINAL_MID_STEP_MIN + abs_preview / 2;
        step_limit = clamp_i(step_limit, CAMERA_FINAL_MID_STEP_MIN, CAMERA_FINAL_MID_STEP_MAX);

        delta = (float)mid_new - final_mid_filtered;
        if (delta > (float)step_limit) delta = (float)step_limit;
        if (delta < -(float)step_limit) delta = -(float)step_limit;

        target_mid = final_mid_filtered + delta;
        final_mid_filtered = final_mid_filtered * (1.0f - CAMERA_MID_FILT_ALPHA) + target_mid * CAMERA_MID_FILT_ALPHA;
    }

    final_mid_tmp = (uint8)clamp_i((int)(final_mid_filtered + 0.5f), 1, MT9V03X_W - 2);
    bias_tmp = (int16)((int)final_mid_tmp - MID_W);
    preview_tmp = (int)mid_line_list[far_y] - (int)mid_line_list[near_y];

    if (s_curve_tmp)
    {
        preview_tmp = (preview_mid_tmp + 2 * preview_far_tmp) / 3;
    }
    if (ring_dir_tmp != 0)
    {
        preview_tmp += (int)ring_dir_tmp * CAMERA_RING_PREVIEW_BIAS;
        s_curve_tmp = 0;
    }

    preview_tmp = clamp_i(preview_tmp, -90, 90);
    preview_far_tmp = clamp_i(preview_far_tmp, -90, 90);
    preview_far_out = (int16)preview_far_tmp;

    ea_state = EA;
    EA = 0;
    final_mid_line = final_mid_tmp;
    camera_bias_raw = bias_tmp;
    camera_preview_raw = (int16)preview_tmp;
    camera_preview_far_raw = preview_far_out;
    camera_ring_dir = ring_dir_tmp;
    camera_ring_score = ring_score_tmp;
    camera_s_curve_flag = s_curve_tmp;
    EA = ea_state;

    return final_mid_tmp;
}


#if CAMERA_DEBUG_DRAW_ENABLE
static void draw_debug_overlay(void)
{
    int row;
    int px, py;

    ips200_show_gray_image(0, 0, bin_image[0], MT9V03X_W, MT9V03X_H, 211, 135, 0);

    for (row = search_start_line - 1; row > search_end_line; row -= CAMERA_ROW_STEP)
    {
        px = (left_line_list[row] * 9) >> 3;
        py = (row * 9) >> 3;
        ips200_draw_point(px, py, RGB565_BLUE);

        px = (right_line_list[row] * 9) >> 3;
        ips200_draw_point(px, py, RGB565_GREEN);

        px = (mid_line_list[row] * 9) >> 3;
        ips200_draw_point(px, py, RGB565_RED);
    }

    ips200_show_string(10, 160, "mid:");
    ips200_show_uint8(60, 160, final_mid_line);
    ips200_show_string(10, 175, "fps:");
    ips200_show_uint16(60, 175, current_fps);
    ips200_show_string(10, 190, "conf:");
    ips200_show_uint8(60, 190, camera_confidence);
}
#endif

#if (IPS200_OR_WIFI == 0)

void camara_init(void)
{
    ips200_init();
    ips200_show_string(0, 0, "mt9v03x init.");

    while (1)
    {
        system_delay_ms(80);
        if (mt9v03x_init())
        {
            ips200_show_string(0, 16, "mt9v03x reinit.");
        }
        else
        {
            break;
        }
    }

    ips200_show_string(0, 16, "init success.");
}

void camara_task(void)
{
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        if (otsu_update_cnt == 0)
        {
            img_threshold = calc_otsu_threshold_sampling();
        }
        otsu_update_cnt++;
        if (otsu_update_cnt >= CAMERA_OTSU_UPDATE_INTERVAL)
        {
            otsu_update_cnt = 0;
        }
#if CAMERA_BINARY_SNAPSHOT_ENABLE
        make_binary_snapshot();
#endif
        find_jidian();
        image_deal();
        final_mid_line = find_mid_line_weight();
        mark_frame_processed();

#if CAMERA_DEBUG_DRAW_ENABLE
        if (++skip_draw >= CAMERA_DEBUG_DRAW_INTERVAL)
        {
            skip_draw = 0;
            draw_debug_overlay();
        }
#endif
    }
}

#elif (IPS200_OR_WIFI == 1)

void camara_init(void)
{
    wireless_uart_init();

    while (wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST))
    {
        system_delay_ms(100);
    }

    if (1 != WIFI_SPI_AUTO_CONNECT)
    {
        while (wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT))
        {
            system_delay_ms(100);
        }
    }

    while (mt9v03x_init())
    {
        system_delay_ms(80);
    }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, bin_image[0], MT9V03X_W, MT9V03X_H);

#if WIFI_BOUNDARY_ENABLE
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, left_line_list, right_line_list, mid_line_list, NULL, NULL, NULL);
#endif
}

void camara_task(void)
{
    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        if (otsu_update_cnt == 0)
        {
            img_threshold = calc_otsu_threshold_sampling();
        }
        otsu_update_cnt++;
        if (otsu_update_cnt >= CAMERA_OTSU_UPDATE_INTERVAL)
        {
            otsu_update_cnt = 0;
        }
#if CAMERA_BINARY_SNAPSHOT_ENABLE
        make_binary_snapshot();
#endif
        find_jidian();
        image_deal();
        final_mid_line = find_mid_line_weight();
        mark_frame_processed();

        seekfree_assistant_camera_send();
    }
}

#else
#error "IPS200_OR_WIFI must be 0 or 1."
#endif



















