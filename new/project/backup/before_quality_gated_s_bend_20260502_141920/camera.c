#include "camera.h"
#include "zf_common_headfile.h"

// ===================== 可调参数（视觉层） =====================
#define CAMERA_DEBUG_DRAW_ENABLE         0   // 0=关闭显示提帧率，1=开启显示便于调试
#define CAMERA_DEBUG_DRAW_INTERVAL       6

// 性能档位：0=稳健，1=均衡，2=高帧率（推荐）
#define CAMERA_PERF_PROFILE              2

#if (CAMERA_PERF_PROFILE == 0)
#define CAMERA_LOCAL_RANGE_MIN           14
#define CAMERA_LOCAL_RANGE_MAX           36
#define CAMERA_GLOBAL_RESCAN_NEAR_ROW    96
#define CAMERA_ROW_STEP                  2
#define CAMERA_MID_FILT_ALPHA_PCT        72
#define CAMERA_FINAL_MID_STEP_MIN        4
#define CAMERA_FINAL_MID_STEP_MAX        16
#define CAMERA_RESCAN_STREAK_TH          1
#define CAMERA_RESCAN_FORCE_NEAR_ROW     114
#define CAMERA_RESCAN_CONF_TH            50
#elif (CAMERA_PERF_PROFILE == 1)
#define CAMERA_LOCAL_RANGE_MIN           12
#define CAMERA_LOCAL_RANGE_MAX           32
#define CAMERA_GLOBAL_RESCAN_NEAR_ROW    106
#define CAMERA_ROW_STEP                  2
#define CAMERA_MID_FILT_ALPHA_PCT        70
#define CAMERA_FINAL_MID_STEP_MIN        5
#define CAMERA_FINAL_MID_STEP_MAX        17
#define CAMERA_RESCAN_STREAK_TH          2
#define CAMERA_RESCAN_FORCE_NEAR_ROW     116
#define CAMERA_RESCAN_CONF_TH            45
#else
#define CAMERA_LOCAL_RANGE_MIN           12  // 缩小局部搜索窗口
#define CAMERA_LOCAL_RANGE_MAX           28
#define CAMERA_GLOBAL_RESCAN_NEAR_ROW    112 // 全局补扫只保留最靠近车体的区域
#define CAMERA_ROW_STEP                  3   // 按3行处理，帧率提升最明显
#define CAMERA_MID_FILT_ALPHA_PCT        66
#define CAMERA_FINAL_MID_STEP_MIN        6
#define CAMERA_FINAL_MID_STEP_MAX        18
#define CAMERA_RESCAN_STREAK_TH          2   // 连续丢线才触发全局补扫
#define CAMERA_RESCAN_FORCE_NEAR_ROW     116 // 最近场仍强制补扫，防止贴边漏检
#define CAMERA_RESCAN_CONF_TH            45
#endif

#define CAMERA_EDGE_GRAD_TH              24  // 灰度梯度阈值：越大越抗噪，越小越灵敏
#define CAMERA_EDGE_STEP_TH              10  // 邻域亮度跳变阈值：越大越保守
#define CAMERA_EDGE_SPAN_TH              20  // 局部对比度阈值：抑制低纹理区域误检
#define CAMERA_EDGE_BIAS_TH              6   // 亮暗偏置阈值：过滤伪边缘
#define CAMERA_TOP_THRESHOLD_GAIN        14  // 仅调试图传二值化使用

#define CAMERA_WIDTH_MIN                 18
#define CAMERA_WIDTH_MAX                 (MT9V03X_W - 4)
#define CAMERA_EDGE_MIN_GAP              8

#define CAMERA_GLOBAL_RESCAN_ENABLE      1   // 1=局部丢线后启用全行补扫

#define CAMERA_WIDTH_FILT_ALPHA_PCT      24
#define CAMERA_MID_SEED_BLEND_NUM        2   // mid_seed = (a*old + new)/(a+1)

#define CAMERA_NEAR_ROW                  (MT9V03X_H - 5)

// S弯/十字接圆弯预瞄融合参数：
// 第二个弯入弯晚：增大 CAMERA_ROUTE_S_CURVE_GAIN_PCT 或减小 CAMERA_ROUTE_FAR_ROW。
// 直线回正来回摆：减小 CAMERA_ROUTE_FAR_BLEND_PCT / CAMERA_ROUTE_CURVE_GAIN_PCT。
// 切内线过多：减小 CAMERA_ROUTE_LIMIT 或 CAMERA_ROUTE_S_CURVE_GAIN_PCT。
#define CAMERA_ROUTE_FAR_ROW             24
#define CAMERA_ROUTE_MID_ROW             58
#define CAMERA_ROUTE_AVG_RADIUS          4
#define CAMERA_ROUTE_FAR_BLEND_PCT       32
#define CAMERA_ROUTE_CURVE_GAIN_PCT      24
#define CAMERA_ROUTE_S_CURVE_GAIN_PCT    62
#define CAMERA_ROUTE_LIMIT               90
#define CAMERA_ROUTE_Q_MIN               32  // 窗口质量低于此值，远端/曲率不参与，防止丢线后乱打方向
#define CAMERA_ROUTE_Q_GOOD              55  // 高于此值才认为远端趋势可靠，可更新记忆
#define CAMERA_ROUTE_HOLD_DECAY_PCT      72  // 完全丢线时沿用上一可靠方向并衰减，太保守可减小

#define WIFI_SSID_TEST                   "Car"
#define WIFI_PASSWORD_TEST               "431431431"
#define WIFI_BOUNDARY_ENABLE             1
#define CAMERA_BINARY_OUTPUT_ENABLE      ((CAMERA_DEBUG_DRAW_ENABLE == 1) || (IPS200_OR_WIFI == 1))

// ===================== 全局输出 =====================
uint8 img_threshold = 120;
uint8 left_jidian = 1;
uint8 right_jidian = MT9V03X_W - 2;
uint8 left_line_list[MT9V03X_H];
uint8 right_line_list[MT9V03X_H];
uint8 mid_line_list[MT9V03X_H];
uint8 final_mid_line = MID_W;

int16 camera_bias_raw = 0;
int16 camera_preview_raw = 0;
int16 camera_preview_far_raw = 0;
int16 camera_curve_raw = 0;
uint8 camera_route_mode = 0;
uint8 camera_near_quality = 0;
uint8 camera_mid_quality = 0;
uint8 camera_far_quality = 0;
uint8 camera_near_width = 0;
uint8 camera_mid_width = 0;
uint8 camera_far_width = 0;
int16 camera_near_mid_raw = 0;
int16 camera_mid_mid_raw = 0;
int16 camera_far_mid_raw = 0;
uint8 camera_valid_line_cnt = 0;
uint8 camera_lost_left_cnt = 0;
uint8 camera_lost_right_cnt = 0;
uint8 camera_confidence = 0;

// ===================== 模块内部变量 =====================
#if CAMERA_BINARY_OUTPUT_ENABLE
static uint8 bin_image[MT9V03X_H][MT9V03X_W];
#endif
static uint8 mid_quality_list[MT9V03X_H];
static int final_mid_filtered_x8 = MID_W * 8;
static uint8 final_mid_init = 0;

volatile uint16 current_fps = 0;
volatile uint16 fps_counter = 0;
#if CAMERA_DEBUG_DRAW_ENABLE
static uint8 skip_draw = 0;
#endif

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

static int sign_i(int x)
{
    if (x > 0) return 1;
    if (x < 0) return -1;
    return 0;
}

static uint8 is_left_transition(int row, int x)
{
    int g0, g1, g2, g3, g4;
    int local_min;
    int local_max;
    int grad;
    int step;
    int grad_th;

    if (x < 2 || x > MT9V03X_W - 3) return 0;

    g0 = (int)mt9v03x_image[row][x - 2];
    g1 = (int)mt9v03x_image[row][x - 1];
    g2 = (int)mt9v03x_image[row][x];
    g3 = (int)mt9v03x_image[row][x + 1];
    g4 = (int)mt9v03x_image[row][x + 2];

    local_min = g0;
    if (g1 < local_min) local_min = g1;
    if (g2 < local_min) local_min = g2;
    if (g3 < local_min) local_min = g3;
    if (g4 < local_min) local_min = g4;

    local_max = g0;
    if (g1 > local_max) local_max = g1;
    if (g2 > local_max) local_max = g2;
    if (g3 > local_max) local_max = g3;
    if (g4 > local_max) local_max = g4;

    if ((local_max - local_min) < CAMERA_EDGE_SPAN_TH) return 0;

    grad_th = CAMERA_EDGE_GRAD_TH;
    if (row >= CAMERA_GLOBAL_RESCAN_NEAR_ROW) grad_th -= 2;
    if (row <= 32) grad_th += 2;
    if (grad_th < 12) grad_th = 12;

    grad = (g2 + g3) - (g0 + g1);
    step = g2 - g1;

    if ((grad >= grad_th) &&
        (step >= CAMERA_EDGE_STEP_TH) &&
        ((g2 - local_min) >= CAMERA_EDGE_BIAS_TH))
    {
        return 1;
    }
    return 0;
}

static uint8 is_right_transition(int row, int x)
{
    int g0, g1, g2, g3, g4;
    int local_min;
    int local_max;
    int grad;
    int step;
    int grad_th;

    if (x < 2 || x > MT9V03X_W - 3) return 0;

    g0 = (int)mt9v03x_image[row][x - 2];
    g1 = (int)mt9v03x_image[row][x - 1];
    g2 = (int)mt9v03x_image[row][x];
    g3 = (int)mt9v03x_image[row][x + 1];
    g4 = (int)mt9v03x_image[row][x + 2];

    local_min = g0;
    if (g1 < local_min) local_min = g1;
    if (g2 < local_min) local_min = g2;
    if (g3 < local_min) local_min = g3;
    if (g4 < local_min) local_min = g4;

    local_max = g0;
    if (g1 > local_max) local_max = g1;
    if (g2 > local_max) local_max = g2;
    if (g3 > local_max) local_max = g3;
    if (g4 > local_max) local_max = g4;

    if ((local_max - local_min) < CAMERA_EDGE_SPAN_TH) return 0;

    grad_th = CAMERA_EDGE_GRAD_TH;
    if (row >= CAMERA_GLOBAL_RESCAN_NEAR_ROW) grad_th -= 2;
    if (row <= 32) grad_th += 2;
    if (grad_th < 12) grad_th = 12;

    grad = (g0 + g1) - (g2 + g3);
    step = g1 - g2;

    if ((grad >= grad_th) &&
        (step >= CAMERA_EDGE_STEP_TH) &&
        ((local_max - g2) >= CAMERA_EDGE_BIAS_TH))
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

// 灰度快照+二值快照：仅用于调试显示/图传，不参与主寻线
#if CAMERA_BINARY_OUTPUT_ENABLE
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
    int width_filtered;
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
    int row_quality;
    int quality_accum;
    uint8 prev_low_conf;

    for (row = 0; row < MT9V03X_H; row++)
    {
        left_line_list[row] = 1;
        right_line_list[row] = MT9V03X_W - 2;
        mid_line_list[row] = MID_W;
        mid_quality_list[row] = 0;
    }

    camera_valid_line_cnt = 0;
    camera_lost_left_cnt = 0;
    camera_lost_right_cnt = 0;
    total_rows = 0;
    quality_accum = 0;

    left_prev = (int)left_jidian;
    right_prev = (int)right_jidian;

    if ((right_prev - left_prev) < CAMERA_EDGE_MIN_GAP)
    {
        left_prev = MID_W - 30;
        right_prev = MID_W + 30;
    }

    left_prev = clamp_i(left_prev, 1, MT9V03X_W - 8);
    right_prev = clamp_i(right_prev, left_prev + CAMERA_EDGE_MIN_GAP, MT9V03X_W - 2);
    width_filtered = right_prev - left_prev;
    mid_seed = (left_prev + right_prev) / 2;
    miss_streak = 0;
    prev_low_conf = (camera_confidence <= CAMERA_RESCAN_CONF_TH) ? 1 : 0;

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
            width_est = width_filtered;
            left_now = left_found;
            right_now = left_found + width_est;
            right_rebuilt = 1;
        }
        else if ((right_found >= 0) && (left_found < 0))
        {
            width_est = width_filtered;
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
            width_filtered = (int)(((long)width_filtered * (100 - CAMERA_WIDTH_FILT_ALPHA_PCT) + (long)width_meas * CAMERA_WIDTH_FILT_ALPHA_PCT + 50) / 100);
        }

        if (both_valid)
        {
            row_quality = 100;
        }
        else if (left_rebuilt && right_rebuilt)
        {
            row_quality = 12;
        }
        else
        {
            row_quality = 58;
        }
        if ((left_now <= 2) || (right_now >= MT9V03X_W - 3))
        {
            row_quality = (row_quality * 7) / 10;
        }
        quality_accum += row_quality * CAMERA_ROW_STEP;

        left_line_list[row] = (uint8)left_now;
        right_line_list[row] = (uint8)right_now;
        mid_now = (left_now + right_now) / 2;
        mid_line_list[row] = (uint8)clamp_i(mid_now, 1, MT9V03X_W - 2);
        mid_quality_list[row] = (uint8)clamp_i(row_quality, 0, 100);

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
                    mid_quality_list[fill_row] = mid_quality_list[row];
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
        conf = quality_accum / total_rows;

        // 丢线越多，置信度越低；单边可靠重建不会直接把conf打到0。
        conf -= (int)camera_lost_left_cnt / 4;
        conf -= (int)camera_lost_right_cnt / 4;
        conf = clamp_i(conf, 0, 100);
        camera_confidence = (uint8)conf;
    }
}

static int avg_mid_window(int center_row, int radius)
{
    int row;
    int start_row;
    int end_row;
    int sum_mid;
    int cnt;
    int width_now;

    center_row = clamp_i(center_row, search_end_line + 1, search_start_line - 1);
    start_row = clamp_i(center_row - radius, search_end_line + 1, search_start_line - 1);
    end_row = clamp_i(center_row + radius, search_end_line + 1, search_start_line - 1);

    sum_mid = 0;
    cnt = 0;
    for (row = start_row; row <= end_row; row++)
    {
        width_now = (int)right_line_list[row] - (int)left_line_list[row];
        if ((width_now >= CAMERA_WIDTH_MIN) && (width_now <= CAMERA_WIDTH_MAX))
        {
            sum_mid += (int)mid_line_list[row];
            cnt++;
        }
    }

    if (cnt <= 0)
    {
        return (int)mid_line_list[center_row];
    }
    return sum_mid / cnt;
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
    int mid_y;
    int far_y;
    int near_mid;
    int mid_mid;
    int far_mid;
    int pre_mid;
    int pre_far;
    int curve_tmp;
    int preview_tmp;
    int preview_abs;
    int route_target;
    int route_blend;
    int mid_new;
    int step_limit;
    int32 delta_x8;
    int32 target_mid_x8;
    uint8 final_mid_tmp;
    int16 bias_tmp;
    int16 preview_out;
    uint8 route_mode_tmp;
    uint8 ea_state;

    near_y = clamp_i(CAMERA_NEAR_ROW, search_end_line + 1, MT9V03X_H - 1);
    mid_y = clamp_i(CAMERA_ROUTE_MID_ROW, search_end_line + 1, MT9V03X_H - 1);
    far_y = clamp_i(CAMERA_ROUTE_FAR_ROW, search_end_line + 1, MT9V03X_H - 1);

    near_mid = avg_mid_window(near_y, 3);
    mid_mid = avg_mid_window(mid_y, CAMERA_ROUTE_AVG_RADIUS);
    far_mid = avg_mid_window(far_y, CAMERA_ROUTE_AVG_RADIUS);

    pre_mid = mid_mid - near_mid;
    pre_far = far_mid - near_mid;
    curve_tmp = far_mid - 2 * mid_mid + near_mid;
    route_mode_tmp = 0;

    // pre_mid表示当前入弯趋势，pre_far表示更远处趋势，curve_tmp表示中线弯曲变化。
    // S弯第二个弯晚：通常 pre_mid 与 pre_far 反向，此时提高curve权重让车提前准备反打。
    if ((abs_i(pre_mid) >= 8) && (abs_i(pre_far) >= 14) && (sign_i(pre_mid) != sign_i(pre_far)))
    {
        route_mode_tmp = 1;
        preview_tmp = (pre_mid * 55 + pre_far * 45) / 100;
        preview_tmp += (curve_tmp * CAMERA_ROUTE_S_CURVE_GAIN_PCT) / 100;
    }
    else
    {
        if ((abs_i(pre_far) >= 16) || (abs_i(curve_tmp) >= 18))
        {
            route_mode_tmp = 2;
        }
        preview_tmp = pre_mid;
        preview_tmp += (pre_far * CAMERA_ROUTE_FAR_BLEND_PCT) / 100;
        preview_tmp += (curve_tmp * CAMERA_ROUTE_CURVE_GAIN_PCT) / 100;

        // 十字路口后接圆弯：近处还像直线、远处已经弯时，给一点额外远端预瞄，避免入弯晚。
        if ((abs_i(pre_mid) < 8) && (abs_i(pre_far) >= 18))
        {
            preview_tmp += (pre_far * 20) / 100;
        }
    }

    if (camera_confidence < 35)
    {
        // 低置信度时远端边界可能来自补线，保留方向但减小激进程度。
        preview_tmp = (pre_mid * 70 + preview_tmp * 30) / 100;
        if (route_mode_tmp == 1) route_mode_tmp = 2;
    }

    preview_tmp = clamp_i(preview_tmp, -CAMERA_ROUTE_LIMIT, CAMERA_ROUTE_LIMIT);
    preview_abs = abs_i(preview_tmp);

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

    // 把路线目标轻微叠加到位置环：小弯仍走稳，大弯/S弯提前走内-外-内路线。
    route_target = near_mid + (preview_tmp * 30) / 100;
    route_target = clamp_i(route_target, 1, MT9V03X_W - 2);
    if (route_mode_tmp == 1)
    {
        route_blend = 32;
    }
    else if (preview_abs >= 18)
    {
        route_blend = 24;
    }
    else if (preview_abs >= 8)
    {
        route_blend = 14;
    }
    else
    {
        route_blend = 6;
    }
    if (camera_confidence < 45)
    {
        route_blend = route_blend / 2;
    }
    mid_new = (mid_new * (100 - route_blend) + route_target * route_blend + 50) / 100;
    mid_new = clamp_i(mid_new, 1, MT9V03X_W - 2);

    if (!final_mid_init)
    {
        final_mid_filtered_x8 = mid_new * 8;
        final_mid_init = 1;
    }
    else
    {
        step_limit = CAMERA_FINAL_MID_STEP_MIN + preview_abs / 2;
        step_limit = clamp_i(step_limit, CAMERA_FINAL_MID_STEP_MIN, CAMERA_FINAL_MID_STEP_MAX);

        delta_x8 = (int32)mid_new * 8 - (int32)final_mid_filtered_x8;
        if (delta_x8 > (int32)step_limit * 8) delta_x8 = (int32)step_limit * 8;
        if (delta_x8 < -((int32)step_limit * 8)) delta_x8 = -((int32)step_limit * 8);

        target_mid_x8 = (int32)final_mid_filtered_x8 + delta_x8;
        final_mid_filtered_x8 = (int)(((int32)final_mid_filtered_x8 * (100 - CAMERA_MID_FILT_ALPHA_PCT) + target_mid_x8 * CAMERA_MID_FILT_ALPHA_PCT + 50) / 100);
    }

    final_mid_tmp = (uint8)clamp_i((final_mid_filtered_x8 + 4) / 8, 1, MT9V03X_W - 2);
    bias_tmp = (int16)((int)final_mid_tmp - MID_W);
    preview_out = (int16)preview_tmp;

    ea_state = EA;
    EA = 0;
    final_mid_line = final_mid_tmp;
    camera_bias_raw = bias_tmp;
    camera_preview_raw = preview_out;
    camera_preview_far_raw = (int16)clamp_i(pre_far, -90, 90);
    camera_curve_raw = (int16)clamp_i(curve_tmp, -90, 90);
    camera_route_mode = route_mode_tmp;
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

        find_jidian();
        image_deal();
        (void)find_mid_line_weight();
        mark_frame_processed();

#if CAMERA_DEBUG_DRAW_ENABLE
        if (++skip_draw >= CAMERA_DEBUG_DRAW_INTERVAL)
        {
            skip_draw = 0;
#if CAMERA_BINARY_OUTPUT_ENABLE
            make_binary_snapshot();
#endif
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

        find_jidian();
        image_deal();
        (void)find_mid_line_weight();
        mark_frame_processed();

#if CAMERA_BINARY_OUTPUT_ENABLE
        make_binary_snapshot();
#endif
        seekfree_assistant_camera_send();
    }
}


#else
#error "IPS200_OR_WIFI must be 0 or 1."
#endif























