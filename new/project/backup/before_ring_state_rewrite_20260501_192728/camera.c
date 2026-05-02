#include "camera.h"
#include "zf_common_headfile.h"
#include <math.h>

// ===================== 自定义结构体与宏 =====================
typedef struct {
    uint8 X;
    uint8 Y;
} Point;

#define CAMERA_DEBUG_DRAW_ENABLE         0
#define CAMERA_DEBUG_DRAW_INTERVAL       6
#define CAMERA_PERF_PROFILE              2

#if (CAMERA_PERF_PROFILE == 0)
// ... (保留原有的性能参数)
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
// ... (保留原有的性能参数)
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
// ... (保留原有的性能参数)
#define CAMERA_LOCAL_RANGE_MIN           12
#define CAMERA_LOCAL_RANGE_MAX           28
#define CAMERA_ROW_STEP                  3
#define CAMERA_GLOBAL_RESCAN_NEAR_ROW    112
#define CAMERA_MID_FILT_ALPHA_PCT        66
#define CAMERA_FINAL_MID_STEP_MIN        6
#define CAMERA_FINAL_MID_STEP_MAX        18
#define CAMERA_RESCAN_STREAK_TH          2
#define CAMERA_RESCAN_FORCE_NEAR_ROW     116
#define CAMERA_RESCAN_CONF_TH            45
#endif

#define CAMERA_EDGE_GRAD_TH              24
#define CAMERA_EDGE_STEP_TH              10
#define CAMERA_EDGE_SPAN_TH              20
#define CAMERA_EDGE_BIAS_TH              6
#define CAMERA_TOP_THRESHOLD_GAIN        14

#define CAMERA_WIDTH_MIN                 18
#define CAMERA_WIDTH_MAX                 (MT9V03X_W - 4)
#define CAMERA_EDGE_MIN_GAP              8
#define CAMERA_GLOBAL_RESCAN_ENABLE      1

#define CAMERA_CLOSE_TRACK_GUARD_ENABLE  1
#define CAMERA_CLOSE_TRACK_MID_JUMP_TH   18
#define CAMERA_CLOSE_TRACK_WIDTH_JUMP_TH 22
#define CAMERA_CLOSE_TRACK_EDGE_JUMP_TH  24
#define CAMERA_CLOSE_TRACK_EDGE_GAP_TH   7
#define CAMERA_CLOSE_TRACK_CONF_DIV      2

#define CAMERA_WIDTH_FILT_ALPHA_PCT      24
#define CAMERA_MID_SEED_BLEND_NUM        2

#define CAMERA_NEAR_ROW                  (MT9V03X_H - 5)
#define CAMERA_PREVIEW_MID_ROW           46
#define CAMERA_PREVIEW_FAR_ROW           28

#define CAMERA_SINGLE_LINE_CONF_PCT      38
#define CAMERA_LOST_CONF_PENALTY_DIV     4

// 原有冗余的 Ring Hits/Score 宏已废除，保留状态宏
#define CAMERA_RING_NONE                 0
#define CAMERA_RING_ENTER                1
#define CAMERA_RING_INSIDE               2
#define CAMERA_RING_EXIT                 3
#define CROSSROAD_NONE                   0
#define CROSSROAD_ENTER                  1

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

uint8 camera_ring_state = CAMERA_RING_NONE;
uint8 camera_crossroad_state = CROSSROAD_NONE;

// ===================== 模块内部变量 =====================
#if CAMERA_BINARY_OUTPUT_ENABLE
static uint8 bin_image[MT9V03X_H][MT9V03X_W];
#endif
static int final_mid_filtered_x8 = MID_W * 8;
static uint8 final_mid_init = 0;

volatile uint16 current_fps = 0;
volatile uint16 fps_counter = 0;

// ===================== 数学与基础辅助函数 =====================
static int abs_i(int x) { return (x >= 0) ? x : -x; }
static float fabs_f(float x) { return (x >= 0) ? x : -x; }
static int clamp_i(int x, int min_v, int max_v) {
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

// 在数组上两点之间画线 (物理补线)
static void FillingLine_Array(uint8* line_list, Point start, Point end) {
    if (start.Y <= end.Y || start.Y >= MT9V03X_H || end.Y >= MT9V03X_H) return;
    float slope = (float)(end.X - start.X) / (float)(start.Y - end.Y); // y递减
    for (int y = start.Y; y >= end.Y; y--) {
        int x = start.X + (int)(slope * (start.Y - y));
        x = clamp_i(x, 1, MT9V03X_W - 2);
        line_list[y] = (uint8)x;
    }
}

// 计算边线某一段的斜率 (判断是否为直道)
static float Calculate_Slope(uint8* line_list, uint8 start_y, uint8 end_y) {
    if (start_y <= end_y) return 0.0f;
    return (float)(line_list[end_y] - line_list[start_y]) / (float)(start_y - end_y);
}

// 寻找下拐点 (图像由下往上扫，边线突然向外侧剧烈扩张的位置)
static void Find_Inflection_Points(Point* left_inf, Point* right_inf) {
    left_inf->X = 0; left_inf->Y = 0;
    right_inf->X = 0; right_inf->Y = 0;
    
    // 扫左拐点
    for (int y = MT9V03X_H - 10; y > 30; y -= CAMERA_ROW_STEP) {
        if (left_line_list[y] > 5 && left_line_list[y - CAMERA_ROW_STEP] <= 3) {
            left_inf->X = left_line_list[y];
            left_inf->Y = y;
            break;
        }
    }
    // 扫右拐点
    for (int y = MT9V03X_H - 10; y > 30; y -= CAMERA_ROW_STEP) {
        if (right_line_list[y] < MT9V03X_W - 5 && right_line_list[y - CAMERA_ROW_STEP] >= MT9V03X_W - 3) {
            right_inf->X = right_line_list[y];
            right_inf->Y = y;
            break;
        }
    }
}

// ===================== 边缘提取函数 (原样保留) =====================
static uint8 is_left_transition(int row, int x) { /* 保持原样 */ return 0; }
static uint8 is_right_transition(int row, int x) { /* 保持原样 */ return 0; }
void find_jidian(void) { /* 保持原样 */ }
// ...(此处省略 is_left/right_transition 等边缘提取的实现，以你原本的代码为准)

// ===================== 特征处理与状态机 (核心替换部分) =====================
static void Feature_Processing(void) {
    Point left_inf, right_inf;
    Find_Inflection_Points(&left_inf, &right_inf);
    
    // 1. 十字路口识别防干扰：双边大量丢线，且均找到下拐点[cite: 4]
    if (camera_lost_left_cnt > 20 && camera_lost_right_cnt > 20 && left_inf.Y > 0 && right_inf.Y > 0) {
        camera_crossroad_state = CROSSROAD_ENTER;
        camera_ring_state = CAMERA_RING_NONE;
        
        // 十字路口简单补线：沿原斜率向远端补，或连接固定远点
        Point up_mid = {MID_W, 30}; 
        FillingLine_Array(left_line_list, left_inf, up_mid);
        FillingLine_Array(right_line_list, right_inf, up_mid);
        return; // 进入十字逻辑后，直接退出，防止误判为环岛
    }
    
    // 2. 左环岛识别
    if (camera_lost_left_cnt > 15 && camera_lost_right_cnt < 10 && left_inf.Y > 40) {
        // 关键防误判：右侧必须是一段近似直道
        float right_slope = Calculate_Slope(right_line_list, left_inf.Y, left_inf.Y - 15);
        if (fabs_f(right_slope) < 1.0f) {
            // 找到左侧黑洞的谷底，假定在图像远端1/4处
            Point target_valley = { MT9V03X_W / 5, 30 }; 
            FillingLine_Array(left_line_list, left_inf, target_valley);
            camera_ring_state = CAMERA_RING_ENTER;
            // TODO: 调用底层陀螺仪积分标志位 StartIntegralAngle_Z(30);[cite: 2]
            return;
        }
    }
    
    // 3. 右环岛识别[cite: 2]
    if (camera_lost_right_cnt > 15 && camera_lost_left_cnt < 10 && right_inf.Y > 40) {
        // 关键防误判：左侧必须是一段近似直道
        float left_slope = Calculate_Slope(left_line_list, right_inf.Y, right_inf.Y - 15);
        if (fabs_f(left_slope) < 1.0f) {
            Point target_valley = { MT9V03X_W * 4 / 5, 30 }; 
            FillingLine_Array(right_line_list, right_inf, target_valley);
            camera_ring_state = CAMERA_RING_ENTER;
            // TODO: 调用底层陀螺仪积分标志位 StartIntegralAngle_Z(30);[cite: 2]
            return;
        }
    }
}

// ===================== 主寻线流程 =====================
void image_deal(void) {
    int row;
    // ... 1. 初始化数组
    for (row = 0; row < MT9V03X_H; row++) {
        left_line_list[row] = 1;
        right_line_list[row] = MT9V03X_W - 2;
        mid_line_list[row] = MID_W;
    }

    camera_valid_line_cnt = 0;
    camera_lost_left_cnt = 0;
    camera_lost_right_cnt = 0;

    // ... 2. 执行原有的基于边沿检测的局部/全局寻线逻辑
    // (此处保留原代码中 for (row = search_start_line - 1; row > search_end_line; row -= CAMERA_ROW_STEP) 的核心提取逻辑)
    // 注意：删除原有所有有关 left_open_hits、right_open_hits、expected_width 等有关环岛开口统计的代码。
    
    // ... 3. 更新 conf
    camera_confidence = 100 - (camera_lost_left_cnt + camera_lost_right_cnt) / 2;
    
    // 4. 调用特征提取与补线，更新状态机
    Feature_Processing();
    
    // 5. 根据补线后的左右边线计算中线
    for (row = search_start_line - 1; row > search_end_line; row--) {
        mid_line_list[row] = (left_line_list[row] + right_line_list[row]) / 2;
    }
}

// ===================== 中线加权计算 =====================
uint8 find_mid_line_weight(void) {
    int row;
    uint32 sum_mid = 0;
    uint32 sum_w = 0;
    int mid_new;

    // 计算加权中线
    for (row = search_start_line - 1; row > search_end_line; row -= CAMERA_ROW_STEP) {
        int row_weight = (row >= MT9V03X_H - 24) ? 14 : ((row >= 60) ? 6 : 3);
        sum_mid += (uint32)mid_line_list[row] * (uint32)row_weight;
        sum_w += (uint32)row_weight;
    }
    mid_new = (sum_w == 0) ? MID_W : (int)(sum_mid / sum_w);
    mid_new = clamp_i(mid_new, 1, MT9V03X_W - 2);

    // 平滑滤波
    if (!final_mid_init) {
        final_mid_filtered_x8 = mid_new * 8;
        final_mid_init = 1;
    } else {
        final_mid_filtered_x8 = (int)(((int32)final_mid_filtered_x8 * (100 - CAMERA_MID_FILT_ALPHA_PCT) + mid_new * 8 * CAMERA_MID_FILT_ALPHA_PCT + 50) / 100);
    }

    uint8 final_mid_tmp = (uint8)clamp_i((final_mid_filtered_x8 + 4) / 8, 1, MT9V03X_W - 2);
    
    // 取消了 ring_bias！补线已经改变了赛道形态，舵机会自动追踪。
    camera_bias_raw = (int16)((int)final_mid_tmp - MID_W);
    camera_preview_raw = (int)mid_line_list[CAMERA_PREVIEW_MID_ROW] - (int)mid_line_list[CAMERA_NEAR_ROW];
    camera_preview_far_raw = (int)mid_line_list[CAMERA_PREVIEW_FAR_ROW] - (int)mid_line_list[CAMERA_NEAR_ROW];
    final_mid_line = final_mid_tmp;

    return final_mid_tmp;
}

// ... 尾部保留 camara_init() 和 camara_task() 函数