#include "camera.h"
#include "zf_common_headfile.h"
#include <math.h>

/* ===================== 全局变量 ===================== */
uint8 buff[2];
uint8 img_threshold;
uint8 left_jidian;
uint8 right_jidian;

static uint8 bin_image[MT9V03X_H][MT9V03X_W];

uint8 left_line_list[MT9V03X_H];
uint8 right_line_list[MT9V03X_H];
uint8 mid_line_list[MT9V03X_H];
uint8 left_line_raw[MT9V03X_H];

#define CAM_H     30.0f
#define CAM_F     80.0f
#define VANISH_Y  10.0f

/* ===================== 环岛/特征点专用变量 ===================== */
uint16 pt_down_x = 0, pt_down_y = 0;  /* A点(下拐点)坐标 */
uint16 pt_mid_x = 0,  pt_mid_y = 0;   /* B点(中拐点)坐标 - 预留 */
uint16 pt_up_x = 0,   pt_up_y = 0;    /* C点(上拐点)坐标 - 预留 */
uint16 exit_pt_x = 0;
uint16 exit_pt_y = 0;
uint8  island_dir = 0;                /* 0:无, 1:左环, 2:右环 */
uint8  roundabout_state = 0;          /* 环岛状态机步骤 */
uint16 delay_cnt = 0;                 /* 状态切换延时计数器 - 预留 */
int crossroad_state;
typedef enum {
    R_NONE     = 0, // 直行或普通弯道
    R_IN_PATCH = 1, // 发现 AB 点，开始入环补线
    R_A_LOST   = 2  // A 点消失，依靠 BC 斜率补线
} RState_e;

/* ===================== 八邻域专用变量 ===================== */
// 8个搜索方向: 0上, 1右上, 2右, 3右下, 4下, 5左下, 6左, 7左上
const int8 dir_x[8] = { 0,  1,  1,  1,  0, -1, -1, -1};
const int8 dir_y[8] = {-1, -1,  0,  1,  1,  1,  0, -1};

// 存储八邻域爬行得到的点集（兼容北海玄风/乾勤算法）
uint16 left_pts_x[MT9V03X_H * 3];
uint16 left_pts_y[MT9V03X_H * 3];
uint8  left_pts_dir[MT9V03X_H * 3]; // 存储生长方向
uint16 left_pts_cnt = 0;

uint16 right_pts_x[MT9V03X_H * 3];
uint16 right_pts_y[MT9V03X_H * 3];
uint8  right_pts_dir[MT9V03X_H * 3];
uint16 right_pts_cnt = 0;

/* ===================== 极速二值化宏定义 ===================== */
#define GET_THRESH(y)    ( ((y) < MT9V03X_H/3) ? ((img_threshold > 235) ? 255 : (img_threshold + 20)) : img_threshold )

/* 1. 原始判断宏（专供下面的二值化快照函数使用，提取真实灰度图） */
#define RAW_IS_WHITE(y, x)   ( mt9v03x_image[y][x] >= GET_THRESH(y) )

/* 2. 【核心修复】：全局使用的宏！
   读取被快照锁死的二值化数组 (bin_image)，彻底阻断 DMA 撕裂导致的时空错位！ */
#define IS_WHITE(y, x)       ( bin_image[y][x] == 255 )
#define IS_BLACK(y, x)       ( bin_image[y][x] == 0 )

/* 3. 宏定义级抗噪滤波 */
#define SAFE_MARGIN  2   
#define IS_WHITE_STABLE(y, x) \
    ( ((x) >= SAFE_MARGIN && (x) <= MT9V03X_W-1-SAFE_MARGIN) ? \
      ( IS_WHITE((y), (x)) && IS_WHITE((y), (x)+1) ) : 0 )

#define IS_BLACK_STABLE(y, x) \
    ( ((x) >= SAFE_MARGIN && (x) <= MT9V03X_W-1-SAFE_MARGIN) ? \
      ( IS_BLACK((y), (x)) && IS_BLACK((y), (x)-1) ) : 0 )


volatile uint16 current_fps = 0;
volatile uint16 fps_counter = 0;
static uint8 skip = 0;

// ===================== 相机模块调整记录（中文注释） =====================
// 1) 既有改动：阈值防溢出、状态机修复、调试绘制开关、环岛逻辑开关
// 2) 本次配合控制改动：保持相机主流程轻量，优先保证转向实时性

// 相机调试绘制开关：0=关闭屏幕绘制优先帧率，1=开启可视化调试
#define CAMERA_DEBUG_DRAW_ENABLE      0
#define CAMERA_DEBUG_DRAW_INTERVAL    8
// 赛道无环岛时建议关闭，可明显减少每帧计算量
#define ROUNDABOUT_LOGIC_ENABLE      1

void mark_frame_processed(void) { fps_counter++; }

void my_fps_timer_callback(void)
{
    static uint16 time_ms = 0;
    time_ms++;
    if(time_ms >= 1000)
    {
        current_fps = fps_counter;
        fps_counter = 0;
        time_ms = 0;
    }
}

/* ===================== Ostu ===================== */
uint8 Ostu(void)
{
    uint16 hist[256] = {0};
    uint32 total, sum = 0, sumB = 0, wB = 0, wF = 0;
    float mB, mF, diff, between, maxBetween = -1.0f;
    uint8 threshold = 0;
    uint16 i, x, y;

    for(y = 0; y < MT9V03X_H; y += 2) {
        for(x = 0; x < MT9V03X_W; x += 2) {
            hist[mt9v03x_image[y][x]]++;
        }
    }
    total = (MT9V03X_H / 2) * (MT9V03X_W / 2);

    for(i = 0; i < 256; i++) sum += (uint32)i * hist[i];

    for(i = 0; i < 256; i++) {
        wB += hist[i];
        if(wB == 0) continue;
        wF = total - wB;
        if(wF == 0) break;

        sumB += (uint32)i * hist[i];
        mB = (float)sumB / wB;
        mF = (float)(sum - sumB) / wF;

        diff = mB - mF;
        between = (float)wB * (float)wF * diff * diff;

        if(between > maxBetween) {
            maxBetween = between;
            threshold = (uint8)i;
        }
    }
    return threshold;
}

/* ===================== 快照生成函数 ===================== */
static void make_binary_image(void)
{
    uint16 y, x;
    for (y = 0; y < MT9V03X_H; y++) {
        for (x = 0; x < MT9V03X_W; x++) {
            // 使用原始灰度图生成二值化数组快照
            bin_image[y][x] = RAW_IS_WHITE(y, x) ? 255 : 0;
        }
    }
}

static uint8 u16_to_str(char *out, uint16 v)
{
    char tmp[6]; uint8 i = 0, len = 0;
    if(v == 0) { out[0] = '0'; return 1; }
    while(v && i < 5) { tmp[i++] = (char)('0' + (v % 10)); v /= 10; }
    while(i) { out[len++] = tmp[--i]; }
    return len;
}

uint8 Limit_uint8(int a, int b, int c)
{
    if((b>=a) && (b<=c)) return (uint8)b;
    else if(b<a) return (uint8)a;
    else if(b>c) return (uint8)c;
    return 0;
}

/* ===================== 找基点 ===================== */
void find_jidian(void)
{
    uint8 j;
    uint16 y = jidian_search_line - 1;

    left_jidian = 1;
    right_jidian = MT9V03X_W - 2;

    if(IS_WHITE(y, MT9V03X_W/2) && IS_WHITE(y, MT9V03X_W/2 + 1) && IS_WHITE(y, MT9V03X_W/2 - 1))
    {
        for(j=MT9V03X_W/2;j>0;j--) {
            if(IS_BLACK(y, j-1) && IS_WHITE(y, j) && IS_WHITE(y, j+1)) { left_jidian=j; break; }
            if(j-1==1) { left_jidian=1; break; }
        }
        for(j=MT9V03X_W/2;j<MT9V03X_W-2;j++) {
            if(IS_WHITE(y, j-1) && IS_WHITE(y, j) && IS_BLACK(y, j+1)) { right_jidian=j; break; }
            if(j+1==MT9V03X_W-2) { right_jidian=MT9V03X_W-2; break; }
        }
    }
    else if(IS_WHITE(y, MT9V03X_W/4) && IS_WHITE(y, MT9V03X_W/4 + 1) && IS_WHITE(y, MT9V03X_W/4 - 1))
    {
        for(j=MT9V03X_W/4;j>0;j--) {
            if(IS_BLACK(y, j-1) && IS_WHITE(y, j) && IS_WHITE(y, j+1)) { left_jidian=j; break; }
            if(j-1==1) { left_jidian=1; break; }
        }
        for(j=MT9V03X_W/4;j<MT9V03X_W-2;j++) {
            if(IS_WHITE(y, j-1) && IS_WHITE(y, j) && IS_BLACK(y, j+1)) { right_jidian=j; break; }
            if(j+1==MT9V03X_W-2) { right_jidian=MT9V03X_W-2; break; }
        }
    }
    else if(IS_WHITE(y, MT9V03X_W/4*3) && IS_WHITE(y, MT9V03X_W/4*3 + 1) && IS_WHITE(y, MT9V03X_W/4*3 - 1))
    {
        for(j=MT9V03X_W/4*3;j>0;j--) {
            if(IS_BLACK(y, j-1) && IS_WHITE(y, j) && IS_WHITE(y, j+1)) { left_jidian=j; break; }
            if(j-1==1) { left_jidian=1; break; }
        }
        for(j=MT9V03X_W/4*3;j<MT9V03X_W-2;j++) {
            if(IS_WHITE(y, j-1) && IS_WHITE(y, j) && IS_BLACK(y, j+1)) { right_jidian=j; break; }
            if(j+1==MT9V03X_W-2) { right_jidian=MT9V03X_W-2; break; }
        }
    }
}

/* =======================================================================
 * 八邻域边界跟踪算法 (8-Neighborhood Edge Tracking) - 防竖线修复版
 * ======================================================================= */
void image_deal(void)
{
    /* --- 1. 变量声明 (C89标准) --- */
    int i;
    int start_left_x, start_right_x, start_y;
    int curr_x, curr_y, curr_dir;
    int search_dir, found, start_search;
    int nx, ny;

    /* --- 2. 初始化数组 --- */
    for (i = 0; i < MT9V03X_H; i++) {
        left_line_list[i] = 255;
        right_line_list[i] = 255;
        mid_line_list[i] = MID_W;
    }
    left_pts_cnt = 0;
    right_pts_cnt = 0;

    /* --- 3. 获取起点 --- */
    start_left_x = left_jidian;
    start_right_x = right_jidian;
    start_y = jidian_search_line - 1;
    
    if (start_left_x < 1) start_left_x = 1;
    if (start_right_x > MT9V03X_W - 2) start_right_x = MT9V03X_W - 2;

    /* ---------------- 左边缘八邻域爬行 ---------------- */
    curr_x = start_left_x;
    curr_y = start_y;
    curr_dir = 0; /* 初始方向：向上 */

    while (left_pts_cnt < MT9V03X_H * 3) {
        /* 记录点集供高级算法使用 */
        left_pts_x[left_pts_cnt] = curr_x;
        left_pts_y[left_pts_cnt] = curr_y;
        left_pts_dir[left_pts_cnt] = curr_dir;
        left_pts_cnt++;

        /* 映射到一维数组 (左线找最小值，覆盖更新！) */
        if (curr_y >= 0 && curr_y < MT9V03X_H) {
            // 如果还没记录过，或者找到了更靠左(更小)的坐标，直接覆盖！
            if (left_line_list[curr_y] == 255 || curr_x < left_line_list[curr_y]) {
                left_line_list[curr_y] = Limit_uint8(1, curr_x, MT9V03X_W - 2);
            }
        }

        /* 只有到达图像顶端才结束爬行 */
        if (curr_y <= search_end_line) break;

        /* 寻找下一个边缘点 (向左转90度开始，扫6个方向防止倒退) */
        found = 0;
        start_search = (curr_dir + 6) % 8; 
        
        for (i = 0; i < 6; i++) {
            search_dir = (start_search + i) % 8; /* 顺时针扫 */
            nx = curr_x + dir_x[search_dir];
            ny = curr_y + dir_y[search_dir];

            /* 允许贴着屏幕边缘走，不触发 break */
            if (nx >= 1 && nx <= MT9V03X_W - 2 && ny >= search_end_line && ny < MT9V03X_H) {
                if (IS_WHITE(ny, nx)) {
                    curr_x = nx;
                    curr_y = ny;
                    curr_dir = search_dir;
                    found = 1;
                    break;
                }
            }
        }
        
        /* 【防呆机制】如果进入纯黑死胡同，强制向上走一步，防止死循环 */
        if (!found) {
            curr_y--;
            curr_dir = 0;
        }
    }

    /* ---------------- 右边缘八邻域爬行 ---------------- */
    curr_x = start_right_x;
    curr_y = start_y;
    curr_dir = 0; /* 初始方向：向上 */

    while (right_pts_cnt < MT9V03X_H * 3) {
        right_pts_x[right_pts_cnt] = curr_x;
        right_pts_y[right_pts_cnt] = curr_y;
        right_pts_dir[right_pts_cnt] = curr_dir;
        right_pts_cnt++;

        /* 映射到一维数组 (右线找最大值，覆盖更新！) */
        if (curr_y >= 0 && curr_y < MT9V03X_H) {
            // 如果还没记录过，或者找到了更靠右(更大)的坐标，直接覆盖！
            if (right_line_list[curr_y] == 255 || curr_x > right_line_list[curr_y]) {
                right_line_list[curr_y] = Limit_uint8(1, curr_x, MT9V03X_W - 2);
            }
        }

        if (curr_y <= search_end_line) break;

        found = 0;
        start_search = (curr_dir + 2) % 8; /* 向右转90度开始 */
        
        for (i = 0; i < 6; i++) {
            search_dir = (start_search + 8 - i) % 8; /* 逆时针扫 */
            nx = curr_x + dir_x[search_dir];
            ny = curr_y + dir_y[search_dir];

            if (nx >= 1 && nx <= MT9V03X_W - 2 && ny >= search_end_line && ny < MT9V03X_H) {
                if (IS_WHITE(ny, nx)) {
                    curr_x = nx;
                    curr_y = ny;
                    curr_dir = search_dir;
                    found = 1;
                    break;
                }
            }
        }
        
        if (!found) {
            curr_y--;
            curr_dir = 0;
        }
    }

			/* ---------------- 赛道空缺填补与中线计算 ---------------- */
    for (i = search_start_line - 1; i > search_end_line; i--) {
        
        /* 1. 处理完全没扫到线的空白点 (255) */
        if (left_line_list[i] == 255) {
            left_line_list[i] = (i < search_start_line - 1) ? left_line_list[i + 1] : 1;
        }
        if (right_line_list[i] == 255) {
            right_line_list[i] = (i < search_start_line - 1) ? right_line_list[i + 1] : MT9V03X_W - 2;
        }
        
        left_line_raw[i] = left_line_list[i];

        /* ==========================================
         * ??? 【新增】：S弯防串线护甲 (放在 255 填补后，中线计算前)
         * ========================================== */
        // 如果左线跑到右线脸上了，或者越界跑到右边去了
        if (left_line_list[i] >= right_line_list[i] - 5) {
            
            // 毫不犹豫，直接把这根发疯的左线“击毙”，锁死在屏幕最左边！
            left_line_list[i] = 1; 
            
            // （同理，如果你怕右线发疯跑向左边，也可以顺手加上下面这句）
//             else if (right_line_list[i] <= left_line_list[i] + 5) {
//                 right_line_list[i] = MT9V03X_W - 2;
//             }
        }

        /* ==========================================
         * ??? 【核心逻辑】：纯中线单边补偿逻辑 (保留自然边线版)
         * ========================================== */
        {
            int half_track = 50; 

            // 情况 A：左边丢线 (贴紧左边缘 <= 2)，但右边有效！
            // ?? 【精妙之处】：如果上面触发了防串线护甲，左线变成了 1，
            // 这里就会被完美捕捉，直接进入单边补偿，用健康的右线去算出完美中线！
            if (left_line_list[i] <= 2 && right_line_list[i] < MT9V03X_W - 5) {
                mid_line_list[i] = Limit_uint8(1, right_line_list[i] - half_track, MT9V03X_W - 2);
            }
            
            // 情况 B：右边丢线 (贴紧右边缘 >= MT9V03X_W - 3)，但左边有效！
            else if (right_line_list[i] >= MT9V03X_W - 3 && left_line_list[i] > 2) {
                mid_line_list[i] = Limit_uint8(1, left_line_list[i] + half_track, MT9V03X_W - 2);
            }
            
            // 情况 C：双边都有效，老老实实取平均值
            else {
                mid_line_list[i] = Limit_uint8(1, (left_line_list[i] + right_line_list[i]) / 2, MT9V03X_W - 2);
            }
        }
    }
}



/* =======================================================================
 * 极速一维中值滤波 (专治边线锯齿与飞线毛刺)
 * 放在 image_deal() 之后，所有元素预判之前调用！
 * ======================================================================= */
void median_filter_lines(void)
{
    /* ------------------------------------------------
     * ??? 严格 C89 标准：变量声明置顶
     * ------------------------------------------------ */
    int y, i, j;
    uint8 temp;
    uint8 window[5]; // 5点滑动窗口
    
    // 备份当前边线 (滤波必须依赖原始数据，不能边滤边覆盖)
    uint8 l_temp[MT9V03X_H];
    uint8 r_temp[MT9V03X_H];
    
    // 1. 拷贝当前帧的粗糙边线
    for (y = 0; y < MT9V03X_H; y++) {
        l_temp[y] = left_line_list[y];
        r_temp[y] = right_line_list[y];
    }

    /* ==========================================
     * 2. 对左线进行 5 点极速中值滤波 
     * ========================================== */
    // 首尾各留2行不处理，防止数组越界
    for (y = search_end_line + 2; y < search_start_line - 2; y++) {
        
        // 遇到丢线的废点 (贴紧屏幕边缘) 直接跳过，不参与滤波
        if (l_temp[y] <= 2 || l_temp[y] >= MT9V03X_W - 2) continue;

        // 提取上下共 5 个点的 X 坐标
        window[0] = l_temp[y - 2];
        window[1] = l_temp[y - 1];
        window[2] = l_temp[y];
        window[3] = l_temp[y + 1];
        window[4] = l_temp[y + 2];

        // 极简冒泡排序 (只有 5 个数，极其神速)
        for (i = 0; i < 4; i++) {
            for (j = 0; j < 4 - i; j++) {
                if (window[j] > window[j + 1]) {
                    temp = window[j];
                    window[j] = window[j + 1];
                    window[j + 1] = temp;
                }
            }
        }
        // 取排序后的正中间值(第3个)，覆盖回原数组
        left_line_list[y] = window[2]; 
    }

    /* ==========================================
     * 3. 对右线进行 5 点极速中值滤波 (镜像逻辑)
     * ========================================== */
    for (y = search_end_line + 2; y < search_start_line - 2; y++) {
        
        if (r_temp[y] <= 2 || r_temp[y] >= MT9V03X_W - 2) continue;

        window[0] = r_temp[y - 2];
        window[1] = r_temp[y - 1];
        window[2] = r_temp[y];
        window[3] = r_temp[y + 1];
        window[4] = r_temp[y + 2];

        for (i = 0; i < 4; i++) {
            for (j = 0; j < 4 - i; j++) {
                if (window[j] > window[j + 1]) {
                    temp = window[j];
                    window[j] = window[j + 1];
                    window[j + 1] = temp;
                }
            }
        }
        right_line_list[y] = window[2]; 
    }
    
    /* ==========================================
     * 4. 边线变平滑了，立刻重算一下中线！
     * ========================================== */
    for (y = search_end_line + 2; y < search_start_line - 2; y++) {
        if (left_line_list[y] > 2 && right_line_list[y] < MT9V03X_W - 2) {
            mid_line_list[y] = Limit_uint8(1, (left_line_list[y] + right_line_list[y]) / 2, MT9V03X_W - 2);
        }
    }
}

// ==========================================
// 赛道元素方向预判 (高敏高精十字 + 环岛双重防伪)
// ==========================================
void judge_roundabout_dir(void)
{
    /* ------------------------------------------------
     * ??? 严格 C89 标准：所有局部变量必须在最顶部声明！
     * ------------------------------------------------ */
    int r;
    uint16 left_lost_cnt = 0;
    uint16 right_lost_cnt = 0;
    uint16 wide_track_cnt = 0; // 【新增必杀】：赛道超宽计数器
    
    int half_w = MT9V03X_W / 2; 
    int is_curve = 0; 
    int left_break_y = 0;  
    int right_break_y = 0; 
    int break_diff = 0;
    int track_width = 0;
		int condition_A;
		int condition_B;
    crossroad_state = 0; // 每次重置

    // 如果已经在环岛中，不再重复判断
    if (roundabout_state > 0) return; 

    /* ==========================================
     * 1. 扫描中上部区域，统计各项特征
     * ========================================== */
    for (r = 30; r < MT9V03X_H - 15; r++) {
        
        // 【敏感升级 1】：放宽边缘判定。只要进入了屏幕边缘 10 个像素内，就算丢线！
        if (left_line_list[r] <= 10) {
            left_lost_cnt++;
            if (left_break_y < r) left_break_y = r; 
        }
        
        if (right_line_list[r] >= MT9V03X_W - 11) { 
            right_lost_cnt++;
            if (right_break_y < r) right_break_y = r;
        }
        
        // ??【绝杀特征提取】：计算当前行的赛道宽度
        track_width = right_line_list[r] - left_line_list[r];
        // 如果宽度爆炸（大于165），记录下来
        if (track_width > 165) {
            wide_track_cnt++;
        }
    }

    island_dir = 0; 

    /* ==========================================
     * ?? 【判断十字路口 (高敏感 + 高准确)】
     * ========================================== */
    break_diff = (left_break_y > right_break_y) ? (left_break_y - right_break_y) : (right_break_y - left_break_y);
    
    // 触发条件 A (传统放宽版)：双边都有明显的丢线(>=10行)，且高度差允许达到 30 像素(容忍车身倾斜)
    condition_A = (left_lost_cnt >= 10 && right_lost_cnt >= 10 && break_diff < 30);
    
    // 触发条件 B (无敌宽度版)：就算边缘没贴死，只要赛道宽度撑满屏幕的行数 >= 10 行，绝对是十字！
    condition_B = (wide_track_cnt >= 10);

    // 只要满足任意一个条件，立刻触发十字补线！
    if (condition_A || condition_B) {
        crossroad_state = 1; 
        return; // 确认十字，拦截后续的环岛判断！
    }

    /* ==========================================
     * ??? 【判断左环岛】 (以下原封不动)
     * ========================================== */
    if (left_lost_cnt > 15 && left_lost_cnt > right_lost_cnt + 10) {
        is_curve = 0; 

        for (r = 30; r <= 60; r += 5) { 
            if (right_line_list[r] < half_w + 10) {
                is_curve = 1;
                break;
            }
        }

        if (right_line_list[40] < right_line_list[90] - 40) {
            is_curve = 1;
        }

        if (is_curve) {
            island_dir = 0; 
        } 
        else {
            island_dir = 1; 
        }
    }
    
    /* ==========================================
     * ??? 【判断右环岛】
     * ========================================== */
    else if (right_lost_cnt > 15 && right_lost_cnt > left_lost_cnt + 10) {
        is_curve = 0;

        for (r = 30; r <= 60; r += 5) {
            if (left_line_list[r] > half_w - 10) {
                is_curve = 1;
                break;
            }
        }

        if (left_line_list[40] > left_line_list[90] + 40) {
            is_curve = 1;
        }

        if (is_curve) {
            island_dir = 0; 
        } 
        else {
            island_dir = 2; 
        }
    }
}

void zuohuan(void)
{
    /* --- 1. 變數聲明 --- */
    int  y, step, s, found;
    int idx_A = -1, idx_B = -1;
    int y_ref, x_ref, predict_x;
    float k_slope;
    int land_x = -1, land_y = -1;
    int curr_x, curr_y, curr_dir;
    int max_x = 0, max_y = 0;
    int isl_x[200], isl_y[200]; 
    int isl_cnt = 0;
    int trough_x, trough_y, peak_x, peak_y, keep_state6;

    /* 【關鍵新增】：記憶上一幀 B 點的座標 */
    static int last_B_x = 0;
    static int last_B_y = 60; // 默認給個中景高度

    /* 清空特徵點座標 */
    pt_down_x = 0; pt_down_y = 0;
    pt_mid_x  = 0; pt_mid_y  = 0;
    pt_up_x   = 0; pt_up_y   = 0;

    if (left_pts_cnt < 20) return;
		if (roundabout_state == R_NONE && island_dir != 1) {
			return;
		}
    /* ==========================================            
     * 步骤 1：寻找 A 点 (融合 V型特征验证 + 时域 ROI 追踪)
     * ========================================== */
    // 只有在 R_NONE (0) 和 R_IN_PATCH (1) 的状态下才允许找 A 点
    if ((roundabout_state == R_NONE || roundabout_state == R_IN_PATCH) && island_dir == 1) {
        
        /* ------------------------------------------------
         * ??? 严格 C89 标准：所有局部变量必须在最顶部声明！
         * ------------------------------------------------ */
        int a_point_y = -1;
        int search_start = MT9V03X_H - 10;
        int search_end = MT9V03X_H / 3; // 绝对下半场锁
        int x_above;
        int x_below;
        
        /* 变量声明结束，下面开始真正的执行语句 */
        max_x = 0; 
        
        // 1. 设置动态搜索窗口 (ROI 时域追踪)
        // 如果上一帧已经咬住了 A 点，启动 ROI 追踪，缩小搜索范围
        if (idx_A != -1 && pt_down_y > 0) {
            search_start = pt_down_y + 15;
            search_end   = pt_down_y - 15;
            
            // 越界保护
            if (search_start >= MT9V03X_H) search_start = MT9V03X_H - 2;
            if (search_end < MT9V03X_H / 3) search_end = MT9V03X_H / 3; 
        }

        // 2. 在锁定的窗口内寻找真正的 V 型角
        for (y = search_start; y > search_end; y--) {
            
            // 排除掉因为丢失而变成 1 的废点
            if (left_line_list[y] <= 2) continue;

            // 寻找最靠右的局部极值 (凸起)
            if (left_line_list[y] > max_x) {
                
                // 【核心绝杀：V 型夹角特征验证】
                x_above = (y - 7 > 0) ? left_line_list[y - 7] : 1;
                x_below = (y + 5 < MT9V03X_H) ? left_line_list[y + 5] : 1;
                
                // 判断：比上方突出 5 像素，比下方突出 3 像素
                if (left_line_list[y] > x_above + 5 && left_line_list[y] > x_below + 3) {
                    // 并且确保它不是在屏幕最左边蹭来蹭去的噪点
                    if (left_line_list[y] > 5) { 
                        max_x = left_line_list[y];
                        a_point_y = y;
                    }
                }
            }
        }

        // 3. 最终确认与记忆更新
        if (a_point_y != -1) {
            pt_down_x = max_x;
            pt_down_y = a_point_y;
            idx_A = a_point_y; // 标记 A 点有效，供下一帧 ROI 和步骤 2 使用
        } 
        else {
            // 容错机制：让 A 点“惯性下坠”硬抗一帧
            if (idx_A != -1 && pt_down_y < MT9V03X_H - 10) {
                pt_down_y += 5; 
            } else {
                idx_A = -1; // 彻底丢失
            }
        }
    }

    /* ==========================================
     * 步驟 2：登陸中心島 (宏定义滤波版，绝不降落在噪点上)
     * ========================================== */
    if (idx_A != -1 && roundabout_state == R_NONE) {
        // 情況 A：A 點還在，靠 A 點射線登陸
				if (pt_down_y < 0 || pt_down_y >= MT9V03X_H) {
						idx_A = -1; // 认为A无效
				} else {
						y_ref = pt_down_y + 5;
						if (y_ref < 0) y_ref = 0;
						if (y_ref >= MT9V03X_H) y_ref = MT9V03X_H - 1;

						x_ref = left_line_list[y_ref];
						if (x_ref <= 1 || x_ref >= MT9V03X_W - 2) {
								idx_A = -1; // 左线无效，避免斜率乱飞
						}
				}
        k_slope = (float)(pt_down_x - x_ref) / ((float)(pt_down_y - y_ref) + 0.001f); 
        for (y = pt_down_y - 2; y > 10; y--) {
            predict_x = pt_down_x + (int)(k_slope * (float)(y - pt_down_y));
            if (predict_x < 3) predict_x = 3;
            if (predict_x > MT9V03X_W - 2) predict_x = MT9V03X_W - 2;
            
            /* 【修改点 1】：使用 STABLE 宏，要求必须是一块实心的黑色区域才登陆 */
            if (IS_BLACK_STABLE(y, predict_x) && IS_BLACK_STABLE(y, predict_x - 2)) {
                land_x = predict_x; land_y = y; break;
            }
        }
    } 
    else if (roundabout_state >= R_IN_PATCH) {
        
        int scan_up   = last_B_y - 15;
        int scan_down = last_B_y + 15;
        if (scan_up < 5) scan_up = 5;
        if (scan_down > MT9V03X_H - 5) scan_down = MT9V03X_H - 5;

        for (y = scan_down; y >= scan_up; y--) {
            int start_x = last_B_x + 30; 
            if (start_x > MT9V03X_W - 5) start_x = MT9V03X_W - 5;

            for (predict_x = start_x; predict_x > 5; predict_x--) {
                
                /* 【修改点 2】：右边必须是坚实的白赛道，左边必须是坚实的黑岛屿。
                   彻底免疫赛道里的一粒黑灰，或者岛屿边上的一粒白反光！ */
                if (IS_WHITE_STABLE(y, predict_x) && IS_BLACK_STABLE(y, predict_x - 1)) {
                    land_x = predict_x - 1; 
                    land_y = y;
                    y = 0; break; 
                }
            }
        }
    }

    /* ==========================================
     * 【重要防护】：每次进入前，确保状态被清空，防止拿上一帧的旧数据算 C 点
     * ========================================== */
    idx_B = -1; 
    // isl_cnt = 0; // 假设你的 isl_cnt 在更外层已经清零了，这里不用动

    /* ==========================================
     * 步骤 3：爬虫寻找 B (逻辑不动)
     * ========================================== */
    if (land_x != -1  && roundabout_state<3) {
        curr_x = land_x; curr_y = land_y; curr_dir = 0;
        for (step = 0; step < 150; step++) {
            if (curr_y <= 2 || isl_cnt >= 200) break; 
            isl_x[isl_cnt] = curr_x; isl_y[isl_cnt] = curr_y; isl_cnt++;

            // 找到了 B 点，记录索引
            if (curr_y >= 30 && curr_y <= 90 && curr_x >= max_x && curr_x <= (MT9V03X_W / 2)) {
                max_x = curr_x; max_y = curr_y; idx_B = isl_cnt - 1;
            }
            
            found = 0;
            for (s = 0; s < 8; s++) {
                int search_dir = (curr_dir + 6 + s) % 8;
                int nx = curr_x + dir_x[search_dir], ny = curr_y + dir_y[search_dir];
                if (nx >= 1 && nx < MT9V03X_W-1 && ny >= 2 && ny < MT9V03X_H) {
                    if (IS_WHITE(ny, nx)) {
                        curr_x = nx; curr_y = ny; curr_dir = search_dir; found = 1; break;
                    }
                }
            }
            if (!found) curr_y--;
        }
        
        /* ==========================================
         * 【终极约束】：B 点 Y 轴单向阀保护
         * ========================================== */
        if (max_x > 10) { 
            
            /* 逻辑：如果已经确认进入环岛寻找阶段 (状态 1 或 2)，
               且当前找到的 B 点高度 (max_y) 比上一帧 (last_B_y) 变小了超过 5 个像素
               说明它诡异地往上跳了，违背了车向前开的物理规律！这是个假 B 点！*/
            if (roundabout_state >= R_IN_PATCH && max_y < last_B_y - 5) {
                
                /* ? 发现假点：直接忽略它！
                   并且强行继承上一帧正确的 B 点坐标，让状态机在这一帧安全度过 */
                pt_mid_x = last_B_x;
                pt_mid_y = last_B_y;
            } 
            else {
                /* ? 正常情况：Y 坐标是变大趋势，或者是第一次找 B 点，允许接纳 */
                pt_mid_x = max_x; 
                pt_mid_y = max_y; 
                
                /* 【合法更新記憶】 */
                last_B_x = pt_mid_x;
                last_B_y = pt_mid_y;
            }
        }
    } // <--- 步骤 3 的大括号在这里结束！！！


    /* ==========================================
     * 步骤 4：寻找 C 点 (垂直探测 + 动态记忆跟踪)
     * ========================================== */
            {
                /* 【新增】：记忆上一帧 C 点的坐标 */
                static int last_C_x = 0;
                static int last_C_y = 0;
                
                int y, x;
                int hit_y = -1;
                int hit_x = -1;
                int best_c_x = -1;
                int best_c_y = -1;
                int start_y;
                
                /* --------------------------------------------------
                 * 模式 A：常规模式 (B 点存在，以 B 为锚点往上找)
                 * -------------------------------------------------- */
                if (idx_B != -1 && isl_y[idx_B] > 20 && roundabout_state<4) {
                    
                    hit_x = isl_x[idx_B]; // 记录 B 点的横坐标作为垂直线

                    for (y = isl_y[idx_B] - 10; y > 10; y--) {
                        if (IS_BLACK(y, hit_x)) {
                            hit_y = y; 
                            break;
                        }
                    }
                }
                /* --------------------------------------------------
                 * 模式 B：盲区跟踪模式 (状态 3 不找 B 点了，以旧 C 点为锚点找新 C 点)
                 * -------------------------------------------------- */
                else if (roundabout_state == 3 && last_C_x > 0) {
                    
                    /* 逻辑：车往前开，C 点在画面中会越来越往下(Y增大)。
                       我们从旧 C 点往左平移 20 个像素，确保射线正对着黑岛底部。 */
                    hit_x = last_C_x - 20;
                    if (hit_x < 5) hit_x = 5;

                    /* 从旧 C 点稍微靠下的位置开始，垂直往上扫，寻找新的底边 */
                    start_y = last_C_y + 20; 
                    if (start_y >= MT9V03X_H) start_y = MT9V03X_H - 1;

                    for (y = start_y; y > 5; y--) {
                        if (IS_BLACK(y, hit_x)) {
                            hit_y = y;
                            break;
                        }
                    }
                }

                /* --------------------------------------------------
                 * 共有特征提取：找到了黑边 hit_y，开始横向找角点
                 * -------------------------------------------------- */
                if (hit_y != -1) {
                    
                    /* ?? 【核心高度锁】：限制 C 点高度 (Y) 必须小于 90
                       如果撞墙的这一行已经处于画面最下方 (>= 90)，说明车头已经贴近了，
                       这个角点极有可能是 A 点或车头噪点，直接放弃提取！ */
                    if (hit_y < 90) {
                        
                        /* 在撞墙这一行(hit_y)，从 hit_x 开始向右扫描找尖角 */
                        for (x = hit_x; x < MT9V03X_W - 5; x++) {
                            if (IS_BLACK(hit_y, x) && IS_WHITE(hit_y, x + 1)) {
                                best_c_x = x;
                                best_c_y = hit_y;
                                break;
                            }
                        }
                    }

                    /* 容错：尝试在稍高一点的一行找 */
                    if (best_c_x == -1 && hit_y > 5) {
                        
                        /* 同样加上高度锁 */
                        if ((hit_y - 2) < 90) {
                            for (x = hit_x; x < MT9V03X_W - 5; x++) {
                                if (IS_BLACK(hit_y - 2, x) && IS_WHITE(hit_y - 2, x + 1)) {
                                    best_c_x = x;
                                    best_c_y = hit_y - 2;
                                    break;
                                }
                            }
                        }
                    }
                }

                /* --------------------------------------------------
                 * 最终输出与记忆更新
                 * -------------------------------------------------- */
                if (best_c_x != -1) {
                    pt_up_x = best_c_x;
                    pt_up_y = best_c_y;
                    
                    /* 安全校验：向下移动 1 像素落入白区 */
                    if ((pt_up_y + 1) < MT9V03X_H && IS_WHITE(pt_up_y + 1, pt_up_x)) {
                        pt_up_y += 1;
                    }

                    /* 【关键更新】：把成功找到的新 C 点存入记忆，供下一帧使用 */
                    last_C_x = pt_up_x;
                    last_C_y = pt_up_y;
                } else {
                    /* ==========================================
                     * ??? 【核心护甲】：C点惯性下沉防闪穿
                     * ========================================== */
                    if (roundabout_state == 3) {
                        // 如果之前看到过C点，现在丢了，给它加上惯性车速往下掉！
                        if (last_C_y > 0) {
                            pt_up_x = last_C_x;
                            pt_up_y = last_C_y + 10; // 每帧强行下坠 10 像素
                            if (pt_up_y > MT9V03X_H - 1) pt_up_y = MT9V03X_H - 1; // 防溢出
                        } 
                        else {
                            // 强行在屏幕左上方伪造一个基准点，逼迫 60 度斜线画出来
                            pt_up_x = 20; 
                            pt_up_y = 10; 
                        }
                        
                        // 更新记忆，让它下一帧继续往下掉
                        last_C_x = pt_up_x;
                        last_C_y = pt_up_y;
                    } 
                    else {
                        /* 如果不是状态 3 彻底找不到了，就清零 */
                        last_C_x = 0;
                        last_C_y = 0;
                    }
                }
            }

						
						/* ==========================================
     * 步骤 5：状态 4 & 5 专属 - 绿线曲率寻拐法 (出岛点)
     * 目标：动态阈值 + 边缘防伪 + 暴力突变特权
     * ========================================== */
    exit_pt_x = 0;
    exit_pt_y = 0;

    if (roundabout_state == 4 || roundabout_state == 5) {
        int y;
        int step = 4;           /* 【敏感升级】：步长改为 4，让远处的突变更明显 */
        int min_x = MT9V03X_W;  
        int max_dx = 0;         
        int best_y = -1;
        int dx;
        int dynamic_threshold;

        /* 【范围扩大】：从屏幕最底下 H-5 开始，一路往上搜到距离顶部 15 行 */
        for (y = MT9V03X_H - 5; y >= 15 + step; y--) {
            int curr_x = right_line_list[y];
            int up_x   = right_line_list[y - step]; 
            
            /* 【准确升级：边缘防伪】：
               如果当前点或上方的点贴在了屏幕最右侧(无效默认值)，
               说明绿线在这里断了或出界了，直接跳过，绝不能当拐点！ */
            if (curr_x >= MT9V03X_W - 5 || up_x >= MT9V03X_W - 5) {
                continue; 
            }

            if (curr_x < min_x) {
                min_x = curr_x;
            }

            dx = up_x - curr_x;

            
            if (y < 40) {
                dynamic_threshold = 4;
            } else if (y < 75) {
                dynamic_threshold = 6;
            } else {
                dynamic_threshold = 8;
            }

            /* 如果超过了动态阈值，并且比之前记录的都大，则开始竞争 best_y */
            if (dx > dynamic_threshold && dx > max_dx) {
                
                /* 【敏感升级：放宽突出度束缚 + 暴力特权】
                   条件A：位于突出点附近 (容错拉宽到 min_x + 12)
                   条件B：如果 dx 突然大于 15 (极其暴力的撕裂)，无视突出条件直接认！ */
                if (curr_x <= min_x + 12 || dx > 15) {
                    max_dx = dx;
                    best_y = y;
                }
            }
        }

        /* 【核心 Bug 修复】：去掉了 max_dx > 8 的死板校验！
           只要 best_y 被赋值了，说明它一定通过了上面的动态阈值筛选！ */
        if (best_y != -1) {
            exit_pt_x = right_line_list[best_y];
            exit_pt_y = best_y;
        }

        /* --------------------------------------------------
         * 出岛切线预判
         * -------------------------------------------------- */
        if (exit_pt_x != 0) {
            /* 在状态 4 下看见点，立刻切入状态 5 准备拉线！ */
            if (roundabout_state == 4) {
                roundabout_state = 5; 
            }
        }
    }
		
		
		
		
    /* ==========================================
     * 状态切换逻辑
     * ========================================== */
    if (roundabout_state == R_NONE) {
        if (pt_down_x > 0 && pt_mid_x > 0) roundabout_state = R_IN_PATCH; 
    } 
    else if (roundabout_state == R_IN_PATCH) {
        if (pt_down_x == 0 && pt_mid_x > 0) roundabout_state = R_A_LOST;  
        if (pt_mid_x == 0) roundabout_state = R_NONE; 
    }
    else if (roundabout_state == R_A_LOST) {
        
        /* 【核心修正】：用 B 点的 Y 坐标 (pt_mid_y) 判断距离！
           当 B 点下沉到画面中下方(>50)时，说明车已经贴近环岛入口，立刻切入！ */
        if (pt_mid_y > 50 && pt_up_y > 0) {
            roundabout_state = 3; // 进入状态 3 (强制拉角入环)
        }
        /* 如果 B 点彻底丢了 (坐标为 0) */
        else if (pt_mid_y == 0) {
            roundabout_state = R_NONE;
            last_B_y = 60; // 彻底丢失后重置预测高度
        }
    }
    /* 状态 3 的退出逻辑 */
    else if (roundabout_state == 3) {
        // 进环后，C 点(上拐点)也会逐渐掉出屏幕下方而丢失
        if ( pt_up_y >= MT9V03X_H - 10) {
            roundabout_state = 4; // 切入状态 4：环内巡航
        }
    }
		/* ==========================================
     * 狀態 5 退出邏輯：出島點丟失 -> 進入狀態 6
     * ========================================== */
    else if (roundabout_state == 5) {
        /* * 當 exit_pt_x 歸 0，說明車頭已經完全甩出環島，
         * 那個右側的角點已經掉出了屏幕下方或被車身遮擋。
         */
        if (exit_pt_x == 0) {
            roundabout_state = 6; 
            
        }
    }
    
    /* ==========================================
     * 状态机跳转：状态 6 (出岛恢复) -> 状态 0 (重生)
     * 仅当左侧“波谷+上峰”特征彻底消失时退出，避免卡死
     * ========================================== */
    else if (roundabout_state == 6) {
        keep_state6 = 0;
        trough_x = 255;
        trough_y = -1;
        peak_x = 0;
        peak_y = -1;

        for (y = MT9V03X_H - 15; y >= 20; y--) {
            curr_x = left_line_list[y];
            if (curr_x > 1 && curr_x < MT9V03X_W - 5) {
                if (curr_x < trough_x) {
                    trough_x = curr_x;
                    trough_y = y;
                }
            }
        }

        if (trough_y != -1) {
            for (y = trough_y; y >= 15; y--) {
                curr_x = left_line_list[y];
                if (curr_x > peak_x && curr_x < MT9V03X_W - 5) {
                    peak_x = curr_x;
                    peak_y = y;
                }
            }

            if (peak_y != -1 && (peak_x - trough_x >= 4)) {
                keep_state6 = 1;
            }
        }

        if (!keep_state6) {
            roundabout_state = R_NONE;
            pt_down_x = 0; pt_down_y = 0;
            pt_mid_x  = 0; pt_mid_y  = 0;
            pt_up_x   = 0; pt_up_y   = 0;
            exit_pt_x = 0; exit_pt_y = 0;
            last_B_y = 60;
            island_dir = 0;
        }
    }
}

void patch_roundabout(void)
{
    int r, new_x;
    float slope;
	
    // --- 状态 1：正常的 A-B 补线 ---
    if (roundabout_state == R_IN_PATCH) {
        if (pt_down_x > 0 && pt_mid_x > 0 && pt_down_y != pt_mid_y) {
            slope = (float)((int)pt_down_x - (int)pt_mid_x) / (float)((int)pt_down_y - (int)pt_mid_y);
            for (r = pt_down_y; r >= (int)pt_mid_y; r--) {
                new_x = (int)pt_down_x - (int)(slope * (float)((int)pt_down_y - r));
                left_line_list[r] = Limit_uint8(1, new_x, MT9V03X_W - 2);
                mid_line_list[r] = (left_line_list[r] + right_line_list[r]) / 2;
            }
        }
    }

    // --- 状态 2：A 消失，从 B 点直接连到屏幕左下角 ---
    else if (roundabout_state == R_A_LOST) {
        if (pt_mid_x > 0 && pt_mid_y < MT9V03X_H - 1) { 
            slope = (float)(1 - (int)pt_mid_x) / (float)((MT9V03X_H - 1) - (int)pt_mid_y);
            for (r = pt_mid_y; r < MT9V03X_H; r++) {
                new_x = (int)pt_mid_x + (int)(slope * (float)(r - (int)pt_mid_y));
                left_line_list[r] = Limit_uint8(1, new_x, MT9V03X_W - 2);
                mid_line_list[r] = (left_line_list[r] + right_line_list[r]) / 2;
            }
        }
    }
    
    // --- 【修正】状态 3：快进环岛，从 C 点向右下角呈 60 度猛烈拉线 ---
    else if (roundabout_state == 3 || roundabout_state == 2) {
        /* 确保 C 点有效 */
        if (pt_up_x > 0 && pt_up_y < MT9V03X_H - 1) { 
            
            /* 从 C 点位置向下，一路强行画到屏幕底部 */
            for (r = pt_up_y; r < MT9V03X_H; r++) {
                
                /* 计算 60 度角拉线的理论 X 坐标 */
                new_x = (int)pt_up_x + ((r - (int)pt_up_y) * 7) / 4;
                
                /* ==========================================
                 * 【核心新增】：相交截断保护 (防底层干扰)
                 * ========================================== */
                /* 如果我们算出来的拉线 X 坐标，已经撞上（大于等于）了原本存在的真实右边线，
                   说明引导线已经完美切入了真实赛道边界！
                   立刻 break 打断循环，保留下方所有真实有效的边线和中线数据！*/
                if (new_x >= right_line_list[r]) {
                    break; 
                }
                
                /* Limit_uint8 保护，防止越界 */
                right_line_list[r] = Limit_uint8(1, new_x, MT9V03X_W - 2);
                
                /* 重新计算这一行的中线，引导舵机暴躁入环 */
                mid_line_list[r] = (left_line_list[r] + right_line_list[r]) / 2;
            }
        }
    }
		
		// --- 【极简暴力版】状态 5：强行出岛，右线左上60度拉线，蓝线死锁屏幕左边缘 ---
    else if (roundabout_state == 5) {
        /* 确保出岛点有效 */
        if (exit_pt_x > 0 && exit_pt_y > 0) { 
            
            for (r = exit_pt_y; r >= 0; r--) {
                
                /* ==========================================
                 * 1. 绿线：60度角左上强拉 
                 * ========================================== */
                new_x = (int)exit_pt_x - (((int)exit_pt_y - r) * 7) / 4;
                right_line_list[r] = Limit_uint8(1, new_x, MT9V03X_W - 2);
                
                /* ==========================================
                 * 2. 蓝线：放弃寻线，直接锁死在屏幕最左侧！
                 * (假设 1 是你屏幕最左边的有效坐标)
                 * ========================================== */
                left_line_list[r] = 1; 
                
                /* ==========================================
                 * 3. 极简截断保护
                 * ========================================== */
                /* 如果右边切进来的绿线，已经撞到了屏幕最左侧(和蓝线重合了)，
                   说明这个出岛的“漏斗”已经闭合了，上面的部分没必要再算了！ */
                if (right_line_list[r] <= 2) {
                    break; 
                }

                /* ==========================================
                 * 4. 融合中线
                 * ========================================== */
                /* 此时中线完全是由右侧 60 度的斜率主导的，
                   没有任何外界图像噪点能干扰舵机的动作，平滑到极致！ */
                mid_line_list[r] = (left_line_list[r] + right_line_list[r]) / 2;
            }
        }
    }
		
		// --- 状态 6：出岛恢复期 (双峰波谷搭桥 + 单峰左下兜底) ---
    else if (roundabout_state == 6) {
        int y;
        int pt_down_x = 0, pt_down_y = -1; // 下方波峰
        int trough_x = 255, trough_y = -1; // 波谷最深处
        int pt_up_x = 0, pt_up_y = -1;     // 上方波峰 (出岛 C 点)
				int draw_flag;
        /* ==========================================
         * 步骤 1 & 2：扫描最深波谷，并向上下寻找波峰 (逻辑不变)
         * ========================================== */
        for (y = MT9V03X_H - 15; y >= 20; y--) {
            int curr_x = left_line_list[y];
            if (curr_x > 1 && curr_x < MT9V03X_W - 5) {
                if (curr_x < trough_x) {
                    trough_x = curr_x;
                    trough_y = y;
                }
            }
        }

        if (trough_y != -1) {
            // 往下找下波峰
            for (y = trough_y; y <= MT9V03X_H - 5; y++) {
                int curr_x = left_line_list[y];
                if (curr_x > pt_down_x && curr_x < MT9V03X_W - 5) {
                    pt_down_x = curr_x;
                    pt_down_y = y;
                }
            }

            // 往上找上波峰
            for (y = trough_y; y >= 15; y--) {
                int curr_x = left_line_list[y];
                if (curr_x > pt_up_x && curr_x < MT9V03X_W - 5) {
                    pt_up_x = curr_x;
                    pt_up_y = y;
                }
            }

            /* ==========================================
             * 步骤 3：多级验证与搭桥连线 (增加左下兜底逻辑)
             * ========================================== */
            draw_flag = 0; // 是否执行连线的标志：0=不画，1=画

            // 【情况 A：完美的双山丘波谷】
            // 上下波峰都存在，且都比波谷向右凸出至少 4 个像素
            if (pt_down_y != -1 && pt_up_y != -1 && pt_down_y > pt_up_y) {
                if ((pt_down_x - trough_x >= 4) && (pt_up_x - trough_x >= 4)) {
                    draw_flag = 1; // 满足双峰条件
                }
            }

            // 【情况 B：单山丘兜底 (用户新增逻辑)】
            // 如果情况 A 没触发 (下山丘不存在或不明显)，但上山丘(C点)依然坚挺地存在着！
            if (draw_flag == 0 && pt_up_y != -1 && (pt_up_x - trough_x >= 4)) {
                
                // 【核心兜底】：强行把下方波峰的锚点，拉到屏幕的最左下角！
                pt_down_x = 1;                  // 屏幕最左边缘 (防止越界用 1)
                pt_down_y = MT9V03X_H - 1;      // 屏幕最底端
                
                draw_flag = 1; // 满足单峰兜底条件，准备强行画线
            }

            /* ==========================================
             * 步骤 4：统一执行插值连线
             * ========================================== */
            if (draw_flag == 1) {
                // 计算两点之间的差值
                int dx = pt_down_x - pt_up_x;
                int dy = pt_down_y - pt_up_y;

                // 从上波峰一路画到下方的目标点 (下波峰 或 左下角)
                for (y = pt_up_y; y <= pt_down_y; y++) {
                    
                    // 极速整数插值算法
                    int new_x = pt_up_x + (dx * (y - pt_up_y)) / dy;
                    
                    // 刷新蓝线，并做好安全限制
                    left_line_list[y] = Limit_uint8(1, new_x, MT9V03X_W - 2);
                    
                    // 重新计算被刷新这一段的中线 (红线)
                    mid_line_list[y] = (left_line_list[y] + right_line_list[y]) / 2;
                }
            }
        }
    }
}

/* =======================================================================
 * 十字路口补线：断点强连法 + 下端点向量衍生约束追踪 (纯整数无浮点)
 * ======================================================================= */
void patch_crossroad(void)
{
    /* ------------------------------------------------
     * ??? 严格 C89 标准：变量声明置顶
     * ------------------------------------------------ */
    int y, new_x;
    
    // 记录上下撕裂点
    int l_dn_y = -1, l_up_y = -1;
    int r_dn_y = -1, r_up_y = -1;
    
    // 向量计算与预测专用变量
    int dx,dy, slope_int, pred_x, diff_x;
    
    // 允许上断点偏离预测方向的最大误差 (像素)
    int valid_margin = 15; 
    
    int patch_start_y = 0, patch_end_y = MT9V03X_H;

    if (crossroad_state == 0) return;

    /* ==========================================
     * 1. 左边线：找下断点 -> 算向量 -> 顺着向量找上断点
     * ========================================== */
    // A. 找下断点 (掉入深渊前的那一步)
    for (y = MT9V03X_H - 5; y >= 20; y--) {
        if (left_line_list[y] <= 5 && left_line_list[y+1] > 5) {
            l_dn_y = y + 1;
            
            /* ??【核心修复】：切除爬虫的“内弯钩子”！
               强行把基准点往下退 5 行，取笔直赛道上的健康点算斜率。
               不仅斜率会完美垂直，画线时也会直接把这个钩子覆盖掉！ */
            if (l_dn_y + 5 < MT9V03X_H - 5) {
                l_dn_y += 5; 
            }
            break;
        }
    }

    // B. 如果找到了下断点，且下方有足够长的健康赛道来计算向量
    if (l_dn_y != -1 && l_dn_y + 10 < MT9V03X_H) {
        
        // 【核心】：利用下断点及其下方 10 行的坐标，计算赛道的切线向量
        dx = left_line_list[l_dn_y] - left_line_list[l_dn_y + 10];
        // 左移 8 位相当于放大 256 倍，dy 是 10，所以除以 10
        slope_int = (dx << 8) / 10; 

        // 向上寻找上断点 (爬出深渊的第一步)
        for (y = l_dn_y - 5; y >= 5; y--) {
            if (left_line_list[y] > 5 && left_line_list[y+1] <= 5) {
                
                // ??【向量衍生预判】：算一算顺着下方的走势，这个高度的 X 理论上应该在哪？
                pred_x = left_line_list[l_dn_y] + ((slope_int * (l_dn_y - y)) >> 8);
                
                // 校验实际看到的点，是否落在预测方向的允许误差内
                diff_x = left_line_list[y] - pred_x;
                if (diff_x < 0) diff_x = -diff_x; // 取绝对值
                
                // 如果误差在允许范围内，承认它是真正的上断点！
                if (diff_x < valid_margin) {
                    l_up_y = y;
                    break; 
                }
                // 否则说明是噪点，无视它，继续往上找！
            }
        }
        
        // 【智能兜底】：如果上面出画了，没找到。直接顺着向量延长线射到天际！
        if (l_up_y == -1) {
            l_up_y = 10;
            left_line_list[l_up_y] = left_line_list[l_dn_y] + ((slope_int * (l_dn_y - 10)) >> 8);
            left_line_list[l_up_y] = Limit_uint8(1, left_line_list[l_up_y], MT9V03X_W - 2);
        }
    }

    /* ==========================================
     * 2. 右边线：镜像处理
     * ========================================== */
    for (y = MT9V03X_H - 5; y >= 20; y--) {
        if (right_line_list[y] >= MT9V03X_W - 5 && right_line_list[y+1] < MT9V03X_W - 5) {
            r_dn_y = y + 1;
            
            /* ??【核心修复】：切除右侧的内弯钩子 */
            if (r_dn_y + 5 < MT9V03X_H - 5) {
                r_dn_y += 5;
            }
            break;
        }
    }

    if (r_dn_y != -1 && r_dn_y + 10 < MT9V03X_H) {
        
        dx = right_line_list[r_dn_y] - right_line_list[r_dn_y + 10];
        slope_int = (dx << 8) / 10; 

        for (y = r_dn_y - 5; y >= 5; y--) {
            if (right_line_list[y] < MT9V03X_W - 5 && right_line_list[y+1] >= MT9V03X_W - 5) {
                
                // ?? 右侧向量衍生预判
                pred_x = right_line_list[r_dn_y] + ((slope_int * (r_dn_y - y)) >> 8);
                
                diff_x = right_line_list[y] - pred_x;
                if (diff_x < 0) diff_x = -diff_x;

                if (diff_x < valid_margin) {
                    r_up_y = y;
                    break;
                }
            }
        }
        
        if (r_up_y == -1) {
            r_up_y = 10;
            right_line_list[r_up_y] = right_line_list[r_dn_y] + ((slope_int * (r_dn_y - 10)) >> 8);
            right_line_list[r_up_y] = Limit_uint8(1, right_line_list[r_up_y], MT9V03X_W - 2);
        }
    }

    /* ==========================================
     * 3. 搭桥连线与重算中线
     * ========================================== */
    // --- 执行左侧搭桥 ---
    if (l_dn_y != -1 && l_up_y != -1 && l_dn_y > l_up_y) {
        dx = left_line_list[l_up_y] - left_line_list[l_dn_y];
        dy = l_dn_y - l_up_y;
        slope_int = (dx << 8) / dy;
        
        for (y = l_dn_y - 1; y > l_up_y; y--) {
            new_x = left_line_list[l_dn_y] + ((slope_int * (l_dn_y - y)) >> 8);
            left_line_list[y] = Limit_uint8(1, new_x, MT9V03X_W - 2);
        }
        patch_start_y = (patch_start_y > l_dn_y) ? patch_start_y : l_dn_y;
        patch_end_y   = (patch_end_y < l_up_y) ? patch_end_y : l_up_y;
    }

    // --- 执行右侧搭桥 ---
    if (r_dn_y != -1 && r_up_y != -1 && r_dn_y > r_up_y) {
        dx = right_line_list[r_up_y] - right_line_list[r_dn_y];
        dy = r_dn_y - r_up_y;
        slope_int = (dx << 8) / dy;
        
        for (y = r_dn_y - 1; y > r_up_y; y--) {
            new_x = right_line_list[r_dn_y] + ((slope_int * (r_dn_y - y)) >> 8);
            right_line_list[y] = Limit_uint8(1, new_x, MT9V03X_W - 2);
        }
        patch_start_y = (patch_start_y > r_dn_y) ? patch_start_y : r_dn_y;
        patch_end_y   = (patch_end_y < r_up_y) ? patch_end_y : r_up_y;
    }

    // --- 重新融合被覆盖区段的中线 ---
    if (patch_start_y > 0 && patch_start_y > patch_end_y) {
        for (y = patch_start_y; y >= patch_end_y; y--) {
            mid_line_list[y] = Limit_uint8(1, (left_line_list[y] + right_line_list[y]) / 2, MT9V03X_W - 2);
        }
    }
}



/* ===================== 中线加权 ===================== */
uint8 mid_weight_list[120]=
{
    1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1,
    1,1,1,1,1,1,1,1,1,1, 6,6,6,6,6,6,6,6,6,6,
    7,8,9,10,11,12,13,14,15,16,17,18,19,20,20,20,20,19,18,17,
    16,15,14,13,12,11,10,9,8,7,6,6,6,6,6,6,6,6,6,6,
    1,1,1,1,1,1,1,1,1,1, 1,1,1,1,1,1,1,1,1,1
};

 uint8 final_mid_line = MID_W;
uint8 last_mid_line = MID_W;

uint8 find_mid_line_weight(void)
{
    uint8 mid_line_value = MID_W;
    uint8 mid_line = MID_W;
    uint32 weight_midline_sum = 0;
    uint32 weight_sum = 0;
    uint8 i;

    for(i=MT9V03X_H-1; i>search_end_line; i--) {
        weight_midline_sum += mid_line_list[i] * mid_weight_list[i];
        weight_sum += mid_weight_list[i];
    }
    mid_line=(uint8)(weight_midline_sum/weight_sum);
    mid_line_value=last_mid_line*0.05+mid_line*0.95;
    last_mid_line=mid_line_value;
    return mid_line_value;
}

/* ===================== 画线 ===================== */
void draw_line(void)
{
    uint8 i;
    int px, py,ax,ay;
		int d;

    for(i = MT9V03X_H - 1; i>search_end_line; i--) {
        px = (Limit_uint8(1,left_line_list[i],MT9V03X_W-2) * 9) >> 3; py = (i * 9) >> 3;
        ips200_draw_point(px, py, RGB565_BLUE);
        ips200_draw_point(px, py+1, RGB565_BLUE);

        px = (Limit_uint8(1,right_line_list[i],MT9V03X_W-2) * 9) >> 3; py = (i * 9) >> 3;
        ips200_draw_point(px, py, RGB565_GREEN);
        ips200_draw_point(px, py+1, RGB565_GREEN);

        px = (Limit_uint8(1,mid_line_list[i],MT9V03X_W-2) * 9) >> 3; py = (i * 9) >> 3;
        ips200_draw_point(px, py, RGB565_RED);
        ips200_draw_point(px, py+1, RGB565_RED);
    }
		
		// 新增：画出 A 点红色大十字 (上下左右各 3 像素)
    // ==========================================
    if (pt_down_y > 0 && pt_down_y < MT9V03X_H) {
        // 1. 把相机坐标映射到 IPS 屏幕坐标
        ax = ((int)pt_down_x * 9) >> 3;
        ay = ((int)pt_down_y * 9) >> 3;
        
        
        // 2. 以 ax, ay 为中心，画正负 3 像素的红十字
        for (d = -3; d <= 3; d++) {
            // 安全限制，防止画到屏幕外导致死机
            if (ax + d >= 0 && ax + d < 240 && ay >= 0 && ay < 240) {
                ips200_draw_point(ax + d, ay, RGB565_RED); // 横线
            }
            if (ax >= 0 && ax < 240 && ay + d >= 0 && ay + d < 240) {
                ips200_draw_point(ax, ay + d, RGB565_RED); // 竖线
            }
        }
    }
		// 新增：画出 B 点红色大十字 (上下左右各 3 像素)
    // ==========================================
    if (pt_mid_y > 0 && pt_mid_y < MT9V03X_H) {
        // 1. 把相机坐标映射到 IPS 屏幕坐标
        ax = ((int)pt_mid_x * 9) >> 3;
        ay = ((int)pt_mid_y * 9) >> 3;
        
        
        // 2. 以 ax, ay 为中心，画正负 3 像素的红十字
        for (d = -3; d <= 3; d++) {
            // 安全限制，防止画到屏幕外导致死机
            if (ax + d >= 0 && ax + d < 240 && ay >= 0 && ay < 240) {
                ips200_draw_point(ax + d, ay, RGB565_RED); // 横线
            }
            if (ax >= 0 && ax < 240 && ay + d >= 0 && ay + d < 240) {
                ips200_draw_point(ax, ay + d, RGB565_RED); // 竖线
            }
        }
    }
		// 新增：画出 C 点红色大十字 (上下左右各 3 像素)
    // ==========================================
    if (pt_up_y > 0 && pt_up_y < MT9V03X_H) {
        // 1. 把相机坐标映射到 IPS 屏幕坐标
        ax = ((int)pt_up_x * 9) >> 3;
        ay = ((int)pt_up_y * 9) >> 3;
        
        
        // 2. 以 ax, ay 为中心，画正负 3 像素的红十字
        for (d = -3; d <= 3; d++) {
            // 安全限制，防止画到屏幕外导致死机
            if (ax + d >= 0 && ax + d < 240 && ay >= 0 && ay < 240) {
                ips200_draw_point(ax + d, ay, RGB565_PURPLE); // 横线
            }
            if (ax >= 0 && ax < 240 && ay + d >= 0 && ay + d < 240) {
                ips200_draw_point(ax, ay + d, RGB565_PURPLE); // 竖线
            }
        }
    }
		// ==========================================
    if (exit_pt_x > 0 && exit_pt_y < MT9V03X_H) {
        // 1. 把相机坐标映射到 IPS 屏幕坐标
        ax = ((int)exit_pt_x * 9) >> 3;
        ay = ((int)exit_pt_y * 9) >> 3;
        
        
        // 2. 以 ax, ay 为中心，画正负 3 像素的红十字
        for (d = -3; d <= 3; d++) {
            // 安全限制，防止画到屏幕外导致死机
            if (ax + d >= 0 && ax + d < 240 && ay >= 0 && ay < 240) {
                ips200_draw_point(ax + d, ay, RGB565_BROWN); // 横线
            }
            if (ax >= 0 && ax < 240 && ay + d >= 0 && ay + d < 240) {
                ips200_draw_point(ax, ay + d, RGB565_BROWN); // 竖线
            }
        }
    }
}

#if (IPS200_OR_WIFI == 0)
void camara_init(void)
{
    ips200_init();
    ips200_show_string(0, 0, "mt9v03x init.");
    while(1) {
        system_delay_ms(100);
        if(mt9v03x_init()) ips200_show_string(0, 16, "mt9v03x reinit.");
        else break;
    }
    ips200_show_string(0, 16, "init success.");
}

void camara_task(void)
{
    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        img_threshold = Ostu();
        find_jidian();
        image_deal(); /* 基础扫线 */

#if ROUNDABOUT_LOGIC_ENABLE
        judge_roundabout_dir(); /* 预判环岛方向 */
        zuohuan();
        patch_roundabout();
#endif
				
        final_mid_line = find_mid_line_weight();
        mark_frame_processed();

#if CAMERA_DEBUG_DRAW_ENABLE
        if(++skip >= CAMERA_DEBUG_DRAW_INTERVAL) {
            skip = 0;

            make_binary_image();
            ips200_show_gray_image(0, 0, bin_image[0], MT9V03X_W, MT9V03X_H, 211, 135, 0);
            draw_line();

            ips200_show_string(10, 160, "mid_value:");
            ips200_show_uint8(80,160,final_mid_line);
            ips200_show_string(10, 175, "fps:");
            ips200_show_uint16(50,175,current_fps);
            ips200_show_string(10, 190, "state:");
            ips200_show_uint8(80,190,roundabout_state);
            ips200_show_string(10, 280, "island_dir:");
            ips200_show_uint8(100, 280, island_dir);
            ips200_show_uint8(35,210,exit_pt_x);
            ips200_show_uint8(80,210,exit_pt_y);
            ips200_show_uint8(35,235,pt_mid_x);
            ips200_show_uint8(80,235,pt_mid_y);
            ips200_show_uint8(35,260,pt_up_x);
            ips200_show_uint8(80,260,pt_up_y);
        }
#endif
    }
}

#elif (IPS200_OR_WIFI == 1)

// 【核心修改 1】：把 INCLUDE_BOUNDARY_TYPE 改为 1，开启边线叠加传输！
#define INCLUDE_BOUNDARY_TYPE   1
#define WIFI_SSID_TEST          "Car"
#define WIFI_PASSWORD_TEST      "431431431"

/* ==========================================
 * 专用函数：通过 WiFi 虚拟串口发送调试文本
 * 替代原本屏幕上的 ips200_show_string
 * ========================================== */


void wifi_send_debug_text(void)
{
    char tx[128]; 
    uint8 p = 0;
    
    // 拼接基础状态，这三个数据最关键！
    tx[p++] = 'D'; tx[p++] = 'i'; tx[p++] = 'r'; tx[p++] = ':'; p += u16_to_str(&tx[p], island_dir); tx[p++] = ' ';
    tx[p++] = 'S'; tx[p++] = 't'; tx[p++] = 'a'; tx[p++] = 't'; tx[p++] = 'e'; tx[p++] = ':'; p += u16_to_str(&tx[p], roundabout_state); tx[p++] = ' ';
    
    // 拼接 A 和 B 点坐标，看看到底找到没有
    tx[p++] = 'A'; tx[p++] = '('; p += u16_to_str(&tx[p], pt_down_x); tx[p++] = ','; p += u16_to_str(&tx[p], pt_down_y); tx[p++] = ')'; tx[p++] = ' ';
    tx[p++] = 'B'; tx[p++] = '('; p += u16_to_str(&tx[p], pt_mid_x); tx[p++] = ','; p += u16_to_str(&tx[p], pt_mid_y); tx[p++] = ')'; 
    
    tx[p++] = '\r'; tx[p++] = '\n';
    
    // 发送给逐飞助手右侧的文本框
    wireless_uart_send_buffer((uint8*)tx, p);
}

void camara_init(void)
{
    wireless_uart_init();
    while(wifi_spi_init(WIFI_SSID_TEST, WIFI_PASSWORD_TEST)) { system_delay_ms(100); }
    if(1 != WIFI_SPI_AUTO_CONNECT) {
        while(wifi_spi_socket_connect("TCP", WIFI_SPI_TARGET_IP, WIFI_SPI_TARGET_PORT, WIFI_SPI_LOCAL_PORT)) { system_delay_ms(100); }
    }
    while(mt9v03x_init()) { system_delay_ms(100); }

    seekfree_assistant_interface_init(SEEKFREE_ASSISTANT_WIFI_SPI);
    
    // 【核心修改 2】：配置图像源
    // 这里默认发送灰度原图。如果你想看二值化图，把 mt9v03x_image[0] 换成 bin_image[0] 即可
    seekfree_assistant_camera_information_config(SEEKFREE_ASSISTANT_MT9V03X, bin_image[0], MT9V03X_W, MT9V03X_H);

#if(1 == INCLUDE_BOUNDARY_TYPE)
    // 【核心修改 3】：配置三根边线！
    // 因为我们的数组是按行(Y)存放横坐标(X)的，所以采用 X_BOUNDARY 类型。
    // 按顺序把 左线、右线、中线 喂给助手，助手会自动把它们按不同颜色画在图传上。
    seekfree_assistant_camera_boundary_config(X_BOUNDARY, MT9V03X_H, left_line_list, right_line_list, mid_line_list, NULL, NULL, NULL);
#endif
}

void camara_task(void)
{
    if(mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;  

        img_threshold = Ostu();
				make_binary_image();
        find_jidian();
        image_deal(); /* 基础扫线 */
				median_filter_lines();
#if ROUNDABOUT_LOGIC_ENABLE
        judge_roundabout_dir(); /* 预判环岛方向 */
        zuohuan();
        patch_roundabout();
				patch_crossroad();
#endif
        
        final_mid_line = find_mid_line_weight();
        mark_frame_processed();
					
				  // 如果你在初始化里改成了发 bin_image[0]，这里记得取消注释生成二值化图
				
				
				// 触发发送：打包图像 + 左/右/中三条线，一并推给上位机
				seekfree_assistant_camera_send();
        // 【核心修改 4】：加上降频发送保护！
        // WiFi 带宽有限，如果每帧都发，会造成严重拥堵和控制延迟。
        // 这里复用 CAMERA_DEBUG_DRAW_INTERVAL (比如设置成8，即每8帧发一次)
        if(++skip >= CAMERA_DEBUG_DRAW_INTERVAL) {
            skip = 0;
            
            
						wifi_send_debug_text();
           
        }
    }
}
#else
#error "IPS200_OR_WIFI must be 0 (IPS200) or 1 (WiFi)."
#endif