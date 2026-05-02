#include "camera.h"
#include "zf_common_headfile.h"

// ===================== 可调参数（视觉层） =====================
// CAMERA_DEBUG_DRAW_ENABLE：调视觉时可置1看线；正式跑置0，否则显示/图传会抢时间、降低fps。
#define CAMERA_DEBUG_DRAW_ENABLE         0
// CAMERA_DEBUG_DRAW_INTERVAL：显示打开后每隔多少帧画一次；画面卡就调大，想观察细节就调小。
#define CAMERA_DEBUG_DRAW_INTERVAL       6

// 性能档位：0=最稳但慢，1=均衡，2=高帧率。若日志fps低于35或舵机响应滞后，优先用2；若丢线严重再退到1。
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
// LOCAL_RANGE越大越不易丢线但更容易跨到错误边；S弯第二弯被抄直时不要盲目加大，先看conf和far。
#define CAMERA_LOCAL_RANGE_MIN           12
#define CAMERA_LOCAL_RANGE_MAX           28
// ROW_STEP越大fps越高、远处曲线越粗；如果S弯远处识别太跳，可改成2验证。
#define CAMERA_ROW_STEP                  3
// GLOBAL_RESCAN只在近处补扫，避免远处S弯把另一条边扫进来；大弯近处丢线多可适当减小。
#define CAMERA_GLOBAL_RESCAN_NEAR_ROW    112
// 中线滤波百分比越大响应越快但更抖；回直线抖先降，弯道响应慢再升。
#define CAMERA_MID_FILT_ALPHA_PCT        66
// 最终中线每帧限幅：小弯摆动大就降MAX；大弯跟不上就升MAX。
#define CAMERA_FINAL_MID_STEP_MIN        6
#define CAMERA_FINAL_MID_STEP_MAX        18
#define CAMERA_RESCAN_STREAK_TH          2
#define CAMERA_RESCAN_FORCE_NEAR_ROW     116
#define CAMERA_RESCAN_CONF_TH            45
#endif

// 边缘阈值：误检白色纹理/阴影就升高；看不到黑边/边线断续就降低，但每次只动2~4。
#define CAMERA_EDGE_GRAD_TH              24
#define CAMERA_EDGE_STEP_TH              10
#define CAMERA_EDGE_SPAN_TH              20
#define CAMERA_EDGE_BIAS_TH              6
#define CAMERA_TOP_THRESHOLD_GAIN        14

#define CAMERA_WIDTH_MIN                 18
#define CAMERA_WIDTH_MAX                 (MT9V03X_W - 4)
#define CAMERA_EDGE_MIN_GAP              8

#define CAMERA_GLOBAL_RESCAN_ENABLE      1

// 邻近白色赛道防串线：左下连续反向弯、白色道路很近时，某一侧边界会突然跳到旁边赛道。
// 误识别/转不过头就降低MID_JUMP或WIDTH_JUMP；正常大弯被压得太保守就升这些阈值。
#define CAMERA_CLOSE_TRACK_GUARD_ENABLE  1
#define CAMERA_CLOSE_TRACK_MID_JUMP_TH   18
#define CAMERA_CLOSE_TRACK_WIDTH_JUMP_TH 22
#define CAMERA_CLOSE_TRACK_EDGE_JUMP_TH  24
#define CAMERA_CLOSE_TRACK_EDGE_GAP_TH   7
#define CAMERA_CLOSE_TRACK_CONF_DIV      2

// 宽度和种子滤波：S弯中线突然穿过白区时，优先减小WIDTH_FILT或增大MID_SEED_BLEND让宽度/种子更稳。
#define CAMERA_WIDTH_FILT_ALPHA_PCT      24
#define CAMERA_MID_SEED_BLEND_NUM        2

#define CAMERA_NEAR_ROW                  (MT9V03X_H - 5)
#define CAMERA_PREVIEW_MID_ROW           46
#define CAMERA_PREVIEW_FAR_ROW           28

// 置信度：两边都找到给满贡献，单边找到也给一部分贡献。S弯常单边丢线，若conf长期为0就升SINGLE_LINE_CONF；误检增多就降。
#define CAMERA_SINGLE_LINE_CONF_PCT      38
#define CAMERA_LOST_CONF_PENALTY_DIV     4

// 环岛识别：直道旁环岛应表现为“某一侧连续开口 + 方向稳定 + 近/远前瞻不过分弯曲”。
// OPEN_HITS/DOMINATE越大越保守；误进环岛就升，进不去就降。CONFIRM_FRAMES越大越防误触发但进环岛更晚。
#define CAMERA_RING_ROW_MIN              50
#define CAMERA_RING_ROW_MAX              108
// 环岛按开源思路分阶段：先看远端口，不允许近端口单独触发；双侧开口先让给十字/回环逻辑。
#define CAMERA_RING_FAR_ROW_MAX          84
#define CAMERA_RING_FAR_ENTRY_HITS_TH    4
#define CAMERA_RING_FAR_ENTRY_NEAR_MAX   2
#define CAMERA_RING_FAR_ENTRY_DOMINATE_TH 3
#define CAMERA_RING_FULL_ENTRY_HITS_TH   7
#define CAMERA_RING_FULL_ENTRY_FAR_TH    3
#define CAMERA_RING_FULL_ENTRY_NEAR_GAP  1
#define CAMERA_RING_SIDE_DOMINATE_TH     5
#define CAMERA_RING_CONFIRM_FRAMES       2
#define CAMERA_RING_FAR_CONFIRM_ADD      2
#define CAMERA_RING_KEEP_SCORE_TH        4
#define CAMERA_RING_KEEP_CONF_MIN        8
#define CAMERA_RING_LOST_FRAMES          20
#define CAMERA_RING_ENTER_FRAMES         30
#define CAMERA_RING_ENTER_MIN_FRAMES     8
#define CAMERA_RING_ENTER_NEAR_HITS_TH   3
#define CAMERA_RING_INSIDE_FRAMES        52
#define CAMERA_RING_EXIT_FRAMES          26
#define CAMERA_RING_EXIT_CLEAR_SCORE_TH  2
#define CAMERA_RING_MIN_CONF             25
#define CAMERA_RING_WIDTH_OPEN_ADD       18
#define CAMERA_RING_OPEN_BIAS_TH         10
#define CAMERA_RING_STRAIGHT_PREVIEW_MAX 20
#define CAMERA_RING_FAR_PREVIEW_MAX      42
// 十字/十字回环防干扰：左右两侧同时远端开口或总开口接近时，不当作普通直道旁环岛。
#define CAMERA_RING_CROSS_FAR_HITS_TH    3
#define CAMERA_RING_CROSS_TOTAL_HITS_TH  10
#define CAMERA_RING_CROSS_DOMINATE_MAX   4
// 直道旁环岛入口顺序：先经过第一个口但保持直行，等口消失后第二个同侧口才允许入环。
#define CAMERA_RING_GATE_NONE            0
#define CAMERA_RING_GATE_PASS_EXIT       1
#define CAMERA_RING_GATE_WAIT_ENTRY      2
#define CAMERA_RING_FIRST_MOUTH_HITS_TH  3
#define CAMERA_RING_FIRST_MOUTH_DOM_TH   2
#define CAMERA_RING_FIRST_MOUTH_MIN_FRAMES 2
#define CAMERA_RING_FIRST_MOUTH_GAP_FRAMES 1
#define CAMERA_RING_ENTRY_ARM_FRAMES     105
// 环岛偏置：远端入口触发后就开始给方向；若进环后切太狠就减小MAX或PREVIEW_BIAS。
#define CAMERA_RING_MID_BIAS_MIN         11
#define CAMERA_RING_MID_BIAS_MAX         24
#define CAMERA_RING_PREVIEW_BIAS         20
#define CAMERA_RING_NONE                 0
#define CAMERA_RING_ENTER                1
#define CAMERA_RING_INSIDE               2
#define CAMERA_RING_EXIT                 3

// 普通弯道安全线：目标线向弯内侧收一点，避免整段贴外线。贴外线/冲出就升MAX或降DIV；切内线过多就降MAX或升DIV。
#define CAMERA_CURVE_INNER_BIAS_MAX      8
#define CAMERA_CURVE_INNER_BIAS_DIV      6
#define CAMERA_CURVE_INNER_CONF_MIN      35
#define CAMERA_CURVE_INNER_PREVIEW_MIN   5
#define CAMERA_CURVE_INNER_PREVIEW_MAX   34
// S弯二次入弯：far已经看到第二个弯而near还在回正时，给小偏置防止直线冲出。误把直道当S弯就升FAR_TH或降BIAS。
#define CAMERA_S_TRANS_FAR_TH            12
#define CAMERA_S_TRANS_NEAR_MAX          26
#define CAMERA_S_TRANS_CONF_MIN          15
#define CAMERA_S_TRANS_BIAS_MAX          10
#define CAMERA_S_TRANS_PREVIEW_BIAS      13
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
uint8 camera_valid_line_cnt = 0;
uint8 camera_lost_left_cnt = 0;
uint8 camera_lost_right_cnt = 0;
uint8 camera_confidence = 0;
int16 camera_preview_far_raw = 0;
int8 camera_ring_dir = 0;
uint8 camera_ring_state = CAMERA_RING_NONE;
uint8 camera_ring_score = 0;
uint8 camera_ring_far_score = 0;
uint8 camera_ring_near_score = 0;
int8 camera_ring_entry_dir_debug = 0;
uint8 camera_ring_gate_state_debug = 0;

// ===================== 模块内部变量 =====================
#if CAMERA_BINARY_OUTPUT_ENABLE
static uint8 bin_image[MT9V03X_H][MT9V03X_W];
#endif
static int final_mid_filtered_x8 = MID_W * 8;
static uint8 final_mid_init = 0;
static int8 ring_hold_dir = 0;
static uint8 ring_hold_state = CAMERA_RING_NONE;
static uint8 ring_hold_timer = 0;
static int8 ring_candidate_dir = 0;
static uint8 ring_candidate_frames = 0;
static uint8 ring_lost_frames = 0;
static int8 ring_gate_dir = 0;
static uint8 ring_gate_state = CAMERA_RING_GATE_NONE;
static uint8 ring_gate_timer = 0;
static uint8 ring_gate_lost_frames = 0;

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
    int left_open_hits;
    int right_open_hits;
    int left_open_far_hits;
    int right_open_far_hits;
    int left_open_near_hits;
    int right_open_near_hits;
    int expected_width;
    int open_bias;
    int ring_score_tmp;
    int ring_drive_score_tmp;
    int ring_far_score_tmp;
    int ring_near_score_tmp;
    int single_valid_cnt;
    int close_track_reject_cnt;
    int guard_reject;
    int left_jump;
    int right_jump;
    int mid_jump;
    int width_jump;
    int ring_entry_dir;
    int ring_raw_entry_dir;
    int side_open_dir;
    int side_open_total;
    int side_open_far;
    int side_open_near;
    int left_side_open;
    int right_side_open;
    uint8 ring_gate_allow_entry;
    int left_entry_ok;
    int right_entry_ok;
    int left_far_entry_ok;
    int right_far_entry_ok;
    int left_full_entry_ok;
    int right_full_entry_ok;
    int side_total;
    int side_far;
    int side_near;
    int ring_preview_tmp;
    int ring_far_tmp;
    uint8 ring_curve_guard;
    uint8 ring_cross_guard;
    uint8 ring_far_guard;
    uint8 ring_keep_ok;
    uint8 prev_low_conf;
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
    width_filtered = right_prev - left_prev;
    mid_seed = (left_prev + right_prev) / 2;
    miss_streak = 0;
    left_open_hits = 0;
    right_open_hits = 0;
    left_open_far_hits = 0;
    right_open_far_hits = 0;
    left_open_near_hits = 0;
    right_open_near_hits = 0;
    single_valid_cnt = 0;
    close_track_reject_cnt = 0;
    prev_low_conf = (camera_confidence <= CAMERA_RESCAN_CONF_TH) ? 1 : 0;
    for (row = search_start_line - 1; row > search_end_line; row -= CAMERA_ROW_STEP)
    {
        total_rows += CAMERA_ROW_STEP;
        both_valid = 0;
        left_rebuilt = 0;
        right_rebuilt = 0;
        guard_reject = 0;

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

#if CAMERA_CLOSE_TRACK_GUARD_ENABLE
        if ((left_found >= 0) && (right_found >= 0) && ((right_found - left_found) >= CAMERA_EDGE_MIN_GAP))
        {
            width_meas = right_found - left_found;
            mid_now = (left_found + right_found) / 2;
            left_jump = abs_i(left_found - left_prev);
            right_jump = abs_i(right_found - right_prev);
            mid_jump = abs_i(mid_now - mid_seed);
            width_jump = abs_i(width_meas - width_filtered);

            if (((mid_jump >= CAMERA_CLOSE_TRACK_MID_JUMP_TH) ||
                 (width_jump >= CAMERA_CLOSE_TRACK_WIDTH_JUMP_TH)) &&
                ((left_jump >= CAMERA_CLOSE_TRACK_EDGE_JUMP_TH) ||
                 (right_jump >= CAMERA_CLOSE_TRACK_EDGE_JUMP_TH)))
            {
                if (left_jump > right_jump + CAMERA_CLOSE_TRACK_EDGE_GAP_TH)
                {
                    left_found = -1;
                    guard_reject = 1;
                }
                else if (right_jump > left_jump + CAMERA_CLOSE_TRACK_EDGE_GAP_TH)
                {
                    right_found = -1;
                    guard_reject = 1;
                }
                else if (width_meas > width_filtered)
                {
                    if (mid_now > mid_seed)
                        right_found = -1;
                    else
                        left_found = -1;
                    guard_reject = 1;
                }
            }
        }
#endif

        if (guard_reject && (close_track_reject_cnt <= (int)(255 - CAMERA_ROW_STEP)))
        {
            close_track_reject_cnt += CAMERA_ROW_STEP;
        }

        if ((left_found >= 0) && (right_found >= 0) && ((right_found - left_found) >= CAMERA_EDGE_MIN_GAP))
        {
            left_now = left_found;
            right_now = right_found;
            both_valid = 1;
        }
        else if ((left_found >= 0) && (right_found < 0))
        {
            if (single_valid_cnt <= (int)(255 - CAMERA_ROW_STEP)) single_valid_cnt += CAMERA_ROW_STEP;
            width_est = width_filtered;
            left_now = left_found;
            right_now = left_found + width_est;
            right_rebuilt = 1;
        }
        else if ((right_found >= 0) && (left_found < 0))
        {
            if (single_valid_cnt <= (int)(255 - CAMERA_ROW_STEP)) single_valid_cnt += CAMERA_ROW_STEP;
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

        if (!guard_reject && (row >= CAMERA_RING_ROW_MIN) && (row <= CAMERA_RING_ROW_MAX))
        {
            expected_width = estimate_half_width(row) * 2;
            open_bias = ((left_now + right_now) / 2) - MID_W;

            if (((left_rebuilt && !right_rebuilt) || (left_now <= 2) ||
                 ((width_meas > expected_width + CAMERA_RING_WIDTH_OPEN_ADD) && (open_bias < -CAMERA_RING_OPEN_BIAS_TH))))
            {
                left_open_hits++;
                if (row <= CAMERA_RING_FAR_ROW_MAX)
                    left_open_far_hits++;
                else
                    left_open_near_hits++;
            }
            if (((right_rebuilt && !left_rebuilt) || (right_now >= MT9V03X_W - 3) ||
                 ((width_meas > expected_width + CAMERA_RING_WIDTH_OPEN_ADD) && (open_bias > CAMERA_RING_OPEN_BIAS_TH))))
            {
                right_open_hits++;
                if (row <= CAMERA_RING_FAR_ROW_MAX)
                    right_open_far_hits++;
                else
                    right_open_near_hits++;
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
        conf += (int)single_valid_cnt * CAMERA_SINGLE_LINE_CONF_PCT / total_rows;

        // 单边有效线在S弯仍有价值，所以惩罚不要过重；误检增多时优先调CAMERA_LOST_CONF_PENALTY_DIV。
        conf -= (int)camera_lost_left_cnt / CAMERA_LOST_CONF_PENALTY_DIV;
        conf -= (int)camera_lost_right_cnt / CAMERA_LOST_CONF_PENALTY_DIV;
        conf -= close_track_reject_cnt / CAMERA_CLOSE_TRACK_CONF_DIV;
        conf = clamp_i(conf, 0, 100);
        camera_confidence = (uint8)conf;
    }

    ring_score_tmp = (left_open_hits > right_open_hits) ? left_open_hits : right_open_hits;
    ring_far_score_tmp = (left_open_far_hits > right_open_far_hits) ? left_open_far_hits : right_open_far_hits;
    ring_near_score_tmp = (left_open_near_hits > right_open_near_hits) ? left_open_near_hits : right_open_near_hits;
    ring_drive_score_tmp = ring_score_tmp;
    if ((ring_far_score_tmp * 2) > ring_drive_score_tmp)
        ring_drive_score_tmp = ring_far_score_tmp * 2;

    ring_preview_tmp = (int)mid_line_list[CAMERA_PREVIEW_MID_ROW] - (int)mid_line_list[CAMERA_NEAR_ROW];
    ring_far_tmp = (int)mid_line_list[CAMERA_PREVIEW_FAR_ROW] - (int)mid_line_list[CAMERA_NEAR_ROW];
    ring_curve_guard = ((abs_i(ring_preview_tmp) <= CAMERA_RING_STRAIGHT_PREVIEW_MAX) &&
                        (abs_i(ring_far_tmp) <= CAMERA_RING_FAR_PREVIEW_MAX)) ? 1 : 0;

    ring_cross_guard = 0;
    if (((left_open_far_hits >= CAMERA_RING_CROSS_FAR_HITS_TH) &&
         (right_open_far_hits >= CAMERA_RING_CROSS_FAR_HITS_TH)) ||
        ((left_open_hits >= CAMERA_RING_CROSS_TOTAL_HITS_TH) &&
         (right_open_hits >= CAMERA_RING_CROSS_TOTAL_HITS_TH) &&
         (abs_i(left_open_hits - right_open_hits) <= CAMERA_RING_CROSS_DOMINATE_MAX)))
    {
        ring_cross_guard = 1;
    }

    left_far_entry_ok = ((left_open_far_hits >= CAMERA_RING_FAR_ENTRY_HITS_TH) &&
                         (left_open_near_hits <= CAMERA_RING_FAR_ENTRY_NEAR_MAX) &&
                         (left_open_far_hits >= right_open_far_hits + CAMERA_RING_FAR_ENTRY_DOMINATE_TH)) ? 1 : 0;
    right_far_entry_ok = ((right_open_far_hits >= CAMERA_RING_FAR_ENTRY_HITS_TH) &&
                          (right_open_near_hits <= CAMERA_RING_FAR_ENTRY_NEAR_MAX) &&
                          (right_open_far_hits >= left_open_far_hits + CAMERA_RING_FAR_ENTRY_DOMINATE_TH)) ? 1 : 0;

    left_full_entry_ok = ((left_open_hits >= CAMERA_RING_FULL_ENTRY_HITS_TH) &&
                          (left_open_far_hits >= CAMERA_RING_FULL_ENTRY_FAR_TH) &&
                          (left_open_far_hits + CAMERA_RING_FULL_ENTRY_NEAR_GAP >= left_open_near_hits) &&
                          (left_open_hits >= right_open_hits + CAMERA_RING_SIDE_DOMINATE_TH)) ? 1 : 0;
    right_full_entry_ok = ((right_open_hits >= CAMERA_RING_FULL_ENTRY_HITS_TH) &&
                           (right_open_far_hits >= CAMERA_RING_FULL_ENTRY_FAR_TH) &&
                           (right_open_far_hits + CAMERA_RING_FULL_ENTRY_NEAR_GAP >= right_open_near_hits) &&
                           (right_open_hits >= left_open_hits + CAMERA_RING_SIDE_DOMINATE_TH)) ? 1 : 0;

    left_entry_ok = (left_far_entry_ok || left_full_entry_ok) ? 1 : 0;
    right_entry_ok = (right_far_entry_ok || right_full_entry_ok) ? 1 : 0;
    ring_far_guard = (left_far_entry_ok || right_far_entry_ok) ? 1 : 0;

    left_side_open = ((left_open_hits >= CAMERA_RING_FIRST_MOUTH_HITS_TH) &&
                      (left_open_hits >= right_open_hits + CAMERA_RING_FIRST_MOUTH_DOM_TH)) ? 1 : 0;
    right_side_open = ((right_open_hits >= CAMERA_RING_FIRST_MOUTH_HITS_TH) &&
                       (right_open_hits >= left_open_hits + CAMERA_RING_FIRST_MOUTH_DOM_TH)) ? 1 : 0;
    side_open_dir = 0;
    side_open_total = 0;
    side_open_far = 0;
    side_open_near = 0;
    if (left_side_open && !right_side_open)
    {
        side_open_dir = -1;
        side_open_total = left_open_hits;
        side_open_far = left_open_far_hits;
        side_open_near = left_open_near_hits;
    }
    else if (right_side_open && !left_side_open)
    {
        side_open_dir = 1;
        side_open_total = right_open_hits;
        side_open_far = right_open_far_hits;
        side_open_near = right_open_near_hits;
    }

    ring_raw_entry_dir = 0;
    if (!ring_cross_guard && ring_curve_guard && (camera_confidence >= CAMERA_RING_MIN_CONF))
    {
        if (left_entry_ok && !right_entry_ok)
        {
            ring_raw_entry_dir = -1;
        }
        else if (right_entry_ok && !left_entry_ok)
        {
            ring_raw_entry_dir = 1;
        }
    }

    ring_entry_dir = 0;
    ring_gate_allow_entry = 0;
    if (ring_hold_state == CAMERA_RING_NONE)
    {
        if (ring_cross_guard)
        {
            ring_gate_state = CAMERA_RING_GATE_NONE;
            ring_gate_dir = 0;
            ring_gate_timer = 0;
            ring_gate_lost_frames = 0;
        }
        else if (ring_gate_state == CAMERA_RING_GATE_NONE)
        {
            if (side_open_dir != 0)
            {
                ring_gate_state = CAMERA_RING_GATE_PASS_EXIT;
                ring_gate_dir = (int8)side_open_dir;
                ring_gate_timer = CAMERA_RING_FIRST_MOUTH_MIN_FRAMES;
                ring_gate_lost_frames = 0;
            }
        }
        else if (ring_gate_state == CAMERA_RING_GATE_PASS_EXIT)
        {
            if (ring_gate_timer > 0) ring_gate_timer--;

            if (side_open_dir == ring_gate_dir)
            {
                ring_gate_lost_frames = 0;
            }
            else
            {
                if (ring_gate_lost_frames < 255) ring_gate_lost_frames++;
                if ((ring_gate_timer == 0) && (ring_gate_lost_frames >= CAMERA_RING_FIRST_MOUTH_GAP_FRAMES))
                {
                    ring_gate_state = CAMERA_RING_GATE_WAIT_ENTRY;
                    ring_gate_timer = CAMERA_RING_ENTRY_ARM_FRAMES;
                    ring_gate_lost_frames = 0;
                }
            }
        }
        else
        {
            if (side_open_dir == ring_gate_dir)
            {
                ring_gate_allow_entry = 1;
                ring_gate_lost_frames = 0;
            }
            else
            {
                if (ring_gate_lost_frames < 255) ring_gate_lost_frames++;
            }

            if (ring_gate_timer > 0)
                ring_gate_timer--;
            else
            {
                ring_gate_state = CAMERA_RING_GATE_NONE;
                ring_gate_dir = 0;
                ring_gate_lost_frames = 0;
            }
        }

        if ((ring_gate_allow_entry != 0) && (ring_raw_entry_dir == ring_gate_dir))
        {
            ring_entry_dir = ring_raw_entry_dir;
        }
    }
    else
    {
        ring_entry_dir = ring_raw_entry_dir;
    }

    if (ring_hold_state == CAMERA_RING_NONE)
    {
        if (ring_cross_guard)
        {
            ring_candidate_dir = 0;
            ring_candidate_frames = 0;
            ring_lost_frames = 0;
        }
        else if (ring_entry_dir != 0)
        {
            if (ring_candidate_dir == ring_entry_dir)
            {
                if (ring_far_guard && (ring_candidate_frames <= (uint8)(255 - CAMERA_RING_FAR_CONFIRM_ADD)))
                    ring_candidate_frames += CAMERA_RING_FAR_CONFIRM_ADD;
                else if (ring_candidate_frames < 255)
                    ring_candidate_frames++;
            }
            else
            {
                ring_candidate_dir = (int8)ring_entry_dir;
                ring_candidate_frames = ring_far_guard ? CAMERA_RING_FAR_CONFIRM_ADD : 1;
            }

            if (ring_candidate_frames >= CAMERA_RING_CONFIRM_FRAMES)
            {
                ring_hold_dir = ring_candidate_dir;
                ring_hold_state = CAMERA_RING_ENTER;
                ring_hold_timer = CAMERA_RING_ENTER_FRAMES;
                ring_lost_frames = 0;
                ring_gate_state = CAMERA_RING_GATE_NONE;
                ring_gate_dir = 0;
                ring_gate_timer = 0;
                ring_gate_lost_frames = 0;
            }
        }
        else
        {
            if (ring_candidate_frames > 0) ring_candidate_frames--;
            else ring_candidate_dir = 0;
            ring_lost_frames = 0;
        }
    }
    else
    {
        if (ring_hold_dir < 0)
        {
            side_total = left_open_hits;
            side_far = left_open_far_hits;
            side_near = left_open_near_hits;
        }
        else
        {
            side_total = right_open_hits;
            side_far = right_open_far_hits;
            side_near = right_open_near_hits;
        }

        ring_keep_ok = 0;
        if (ring_entry_dir == ring_hold_dir)
        {
            ring_keep_ok = 1;
        }
        else if ((side_total >= CAMERA_RING_KEEP_SCORE_TH) ||
                 (side_far >= 2) ||
                 (side_near >= 2) ||
                 ((ring_hold_timer > 0) && (camera_confidence >= CAMERA_RING_KEEP_CONF_MIN)))
        {
            ring_keep_ok = 1;
        }

        if (ring_keep_ok)
            ring_lost_frames = 0;
        else if (ring_lost_frames < 255)
            ring_lost_frames++;

        if ((ring_lost_frames >= CAMERA_RING_LOST_FRAMES) && (ring_hold_state == CAMERA_RING_ENTER))
        {
            ring_hold_state = CAMERA_RING_NONE;
            ring_hold_dir = 0;
            ring_hold_timer = 0;
            ring_candidate_dir = 0;
            ring_candidate_frames = 0;
            ring_lost_frames = 0;
        }
        else if (ring_hold_state == CAMERA_RING_ENTER)
        {
            if (ring_hold_timer > 0) ring_hold_timer--;
            if (((ring_hold_timer <= (uint8)(CAMERA_RING_ENTER_FRAMES - CAMERA_RING_ENTER_MIN_FRAMES)) &&
                 (side_near >= CAMERA_RING_ENTER_NEAR_HITS_TH)) ||
                (ring_hold_timer == 0))
            {
                ring_hold_state = CAMERA_RING_INSIDE;
                ring_hold_timer = CAMERA_RING_INSIDE_FRAMES;
                ring_lost_frames = 0;
            }
        }
        else if (ring_hold_state == CAMERA_RING_INSIDE)
        {
            if (ring_hold_timer > 0) ring_hold_timer--;
            if ((ring_hold_timer == 0) ||
                ((side_total <= CAMERA_RING_EXIT_CLEAR_SCORE_TH) &&
                 (side_far <= 1) && (side_near <= 1) &&
                 (camera_confidence >= 45)))
            {
                ring_hold_state = CAMERA_RING_EXIT;
                ring_hold_timer = CAMERA_RING_EXIT_FRAMES;
                ring_lost_frames = 0;
            }
        }
        else
        {
            if (ring_hold_timer > 0) ring_hold_timer--;
            if ((ring_hold_timer == 0) ||
                ((side_total <= CAMERA_RING_EXIT_CLEAR_SCORE_TH) &&
                 (abs_i(ring_preview_tmp) <= CAMERA_RING_STRAIGHT_PREVIEW_MAX) &&
                 (abs_i(ring_far_tmp) <= CAMERA_RING_FAR_PREVIEW_MAX) &&
                 (camera_confidence >= 45)))
            {
                ring_hold_state = CAMERA_RING_NONE;
                ring_hold_dir = 0;
                ring_hold_timer = 0;
                ring_candidate_dir = 0;
                ring_candidate_frames = 0;
                ring_lost_frames = 0;
            }
        }
    }

    camera_ring_dir = (ring_hold_state == CAMERA_RING_NONE) ? 0 : ring_hold_dir;
    camera_ring_state = ring_hold_state;
    camera_ring_score = (uint8)clamp_i(ring_drive_score_tmp, 0, 100);
    camera_ring_far_score = (uint8)clamp_i(ring_far_score_tmp, 0, 100);
    camera_ring_near_score = (uint8)clamp_i(ring_near_score_tmp, 0, 100);
    camera_ring_entry_dir_debug = (int8)ring_entry_dir;
    camera_ring_gate_state_debug = ring_gate_state;
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
    int far2_y;
    int mid_preview_y;
    int rough_y;
    int mid_new;
    int rough_preview;
    int abs_preview;
    int preview_mid_tmp;
    int preview_far_tmp;
    int racing_bias;
    int s_transition;
    int s_bias;
    int ring_bias;
    int ring_dir_tmp;
    int ring_state_tmp;
    int ring_score_tmp;
    int step_limit;
    int32 delta_x8;
    int32 target_mid_x8;
    int preview_tmp;
    int preview_far_out;
    uint8 final_mid_tmp;
    int16 bias_tmp;
    int16 preview_out;
    uint8 ea_state;

    near_y = clamp_i(CAMERA_NEAR_ROW, search_end_line + 1, MT9V03X_H - 1);
    rough_y = clamp_i(32, search_end_line + 1, MT9V03X_H - 1);
    far2_y = clamp_i(CAMERA_PREVIEW_FAR_ROW, search_end_line + 1, MT9V03X_H - 1);
    mid_preview_y = clamp_i(CAMERA_PREVIEW_MID_ROW, search_end_line + 1, MT9V03X_H - 1);

    // 先用固定行做一版粗前瞻，再动态决定far_y
    rough_preview = (int)mid_line_list[rough_y] - (int)mid_line_list[near_y];
    abs_preview = abs_i(rough_preview);
    preview_mid_tmp = (int)mid_line_list[mid_preview_y] - (int)mid_line_list[near_y];
    preview_far_tmp = (int)mid_line_list[far2_y] - (int)mid_line_list[near_y];
    preview_far_out = clamp_i(preview_far_tmp, -90, 90);

    ring_dir_tmp = (int)camera_ring_dir;
    if (ring_dir_tmp > 1) ring_dir_tmp = 1;
    if (ring_dir_tmp < -1) ring_dir_tmp = -1;
    ring_state_tmp = (int)camera_ring_state;
    ring_score_tmp = (int)camera_ring_score;

    if (abs_preview >= 12)
    {
        far_y = 38; // 急弯时略靠近，减少远处噪声误导
    }
    else if (abs_preview >= 6)
    {
        far_y = 34;
    }
    else
    {
        far_y = 28; // 直道/缓弯时看得更远，提前预判
    }

    if (camera_confidence < 50)
    {
        far_y += 4; // 低置信度时回收前瞻距离，优先稳定
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

    racing_bias = 0;
    s_transition = 0;
    if ((ring_state_tmp != CAMERA_RING_NONE) && (ring_dir_tmp != 0))
    {
        if (ring_state_tmp == CAMERA_RING_ENTER)
        {
            ring_bias = CAMERA_RING_MID_BIAS_MIN + 2 + ring_score_tmp / 3;
        }
        else if (ring_state_tmp == CAMERA_RING_INSIDE)
        {
            ring_bias = CAMERA_RING_MID_BIAS_MIN + 5 + ring_score_tmp / 4;
        }
        else
        {
            ring_bias = CAMERA_RING_MID_BIAS_MIN;
        }
        ring_bias = clamp_i(ring_bias, CAMERA_RING_MID_BIAS_MIN, CAMERA_RING_MID_BIAS_MAX);
        mid_new += ring_dir_tmp * ring_bias;
    }
    else if ((camera_confidence >= CAMERA_S_TRANS_CONF_MIN) &&
             (abs_i(preview_far_tmp) >= CAMERA_S_TRANS_FAR_TH) &&
             (abs_i(preview_mid_tmp) <= CAMERA_S_TRANS_NEAR_MAX) &&
             (((preview_mid_tmp > 0) && (preview_far_tmp < 0)) ||
              ((preview_mid_tmp < 0) && (preview_far_tmp > 0)) ||
              (abs_i(preview_mid_tmp) <= 3)))
    {
        s_transition = 1;
        s_bias = -preview_far_tmp / 6;
        s_bias = clamp_i(s_bias, -CAMERA_S_TRANS_BIAS_MAX, CAMERA_S_TRANS_BIAS_MAX);
        mid_new += s_bias;
    }
    else if ((camera_confidence >= CAMERA_CURVE_INNER_CONF_MIN) &&
             (abs_i(preview_far_tmp) >= CAMERA_CURVE_INNER_PREVIEW_MIN) &&
             (abs_preview <= CAMERA_CURVE_INNER_PREVIEW_MAX) &&
             (((preview_mid_tmp > 0) && (preview_far_tmp > 0)) ||
              ((preview_mid_tmp < 0) && (preview_far_tmp < 0))))
    {
        // 普通弯道不再把目标线推向外侧，而是向弯内侧收几像素，留出外侧安全余量。
        racing_bias = -preview_far_tmp / CAMERA_CURVE_INNER_BIAS_DIV;
        racing_bias = clamp_i(racing_bias, -CAMERA_CURVE_INNER_BIAS_MAX, CAMERA_CURVE_INNER_BIAS_MAX);
        mid_new += racing_bias;
    }

    mid_new = clamp_i(mid_new, 1, MT9V03X_W - 2);

    if (!final_mid_init)
    {
        final_mid_filtered_x8 = mid_new * 8;
        final_mid_init = 1;
    }
    else
    {
        step_limit = CAMERA_FINAL_MID_STEP_MIN + abs_preview / 2;
        step_limit = clamp_i(step_limit, CAMERA_FINAL_MID_STEP_MIN, CAMERA_FINAL_MID_STEP_MAX);

        delta_x8 = (int32)mid_new * 8 - (int32)final_mid_filtered_x8;
        if (delta_x8 > (int32)step_limit * 8) delta_x8 = (int32)step_limit * 8;
        if (delta_x8 < -((int32)step_limit * 8)) delta_x8 = -((int32)step_limit * 8);

        target_mid_x8 = (int32)final_mid_filtered_x8 + delta_x8;
        final_mid_filtered_x8 = (int)(((int32)final_mid_filtered_x8 * (100 - CAMERA_MID_FILT_ALPHA_PCT) + target_mid_x8 * CAMERA_MID_FILT_ALPHA_PCT + 50) / 100);
    }

    final_mid_tmp = (uint8)clamp_i((final_mid_filtered_x8 + 4) / 8, 1, MT9V03X_W - 2);
    bias_tmp = (int16)((int)final_mid_tmp - MID_W);
    preview_tmp = (int)mid_line_list[far_y] - (int)mid_line_list[near_y];
    if ((ring_state_tmp != CAMERA_RING_NONE) && (ring_dir_tmp != 0))
    {
        preview_tmp += ring_dir_tmp * CAMERA_RING_PREVIEW_BIAS;
    }
    else if (s_transition)
    {
        if (preview_far_tmp > 0)
            preview_tmp += CAMERA_S_TRANS_PREVIEW_BIAS;
        else
            preview_tmp -= CAMERA_S_TRANS_PREVIEW_BIAS;
    }
    preview_tmp = clamp_i(preview_tmp, -90, 90);
    preview_out = (int16)preview_tmp;

    ea_state = EA;
    EA = 0;
    final_mid_line = final_mid_tmp;
    camera_bias_raw = bias_tmp;
    camera_preview_raw = preview_out;
    camera_preview_far_raw = (int16)preview_far_out;
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























