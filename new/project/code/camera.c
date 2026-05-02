#include "camera.h"
#include "zf_common_headfile.h"

/* 开源风格视觉流程：
 * 阈值二值判断 -> 自底向上扫左右边界 -> 取中线 -> 固定行段算偏差。
 */
#define CAMERA_DEBUG_DRAW_ENABLE      0
#define CAMERA_DEBUG_DRAW_INTERVAL    6

#define IMAGE_BLACK                   0
#define IMAGE_WHITE                   255
#define BORDER_BIAS                   1

#define CAMERA_ROW_STEP               3
#define CAMERA_THRESHOLD_UPDATE_DIV   3
#define CAMERA_THRESHOLD_ROW_STEP     4
#define CAMERA_THRESHOLD_COL_STEP     4
#define CAMERA_BIAS_START_LINE        70
#define CAMERA_BIAS_END_LINE          50
#define CAMERA_PREVIEW_NEAR_ROW       (MT9V03X_H - 15)
#define CAMERA_PREVIEW_MID_ROW        55
#define CAMERA_PREVIEW_FAR_ROW        30
#define CAMERA_AVG_RADIUS             2
#define CAMERA_MIN_BIAS_ROWS          5

#define WIFI_SSID_TEST                "Car"
#define WIFI_PASSWORD_TEST            "431431431"
#define WIFI_BOUNDARY_ENABLE          1
#define CAMERA_BINARY_OUTPUT_ENABLE   ((CAMERA_DEBUG_DRAW_ENABLE == 1) || (IPS200_OR_WIFI == 1))
#define PIXEL_IS_WHITE_FAST(row, col) (mt9v03x_image[(row)][(col)] > img_threshold)

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
uint8 camera_valid_line_cnt = 0;
uint8 camera_lost_left_cnt = 0;
uint8 camera_lost_right_cnt = 0;
uint8 camera_confidence = 0;

volatile uint16 current_fps = 0;
volatile uint16 fps_counter = 0;

#if CAMERA_BINARY_OUTPUT_ENABLE
static uint8 bin_image[MT9V03X_H][MT9V03X_W];
#endif

static uint8 scan_mid = MID_W;
static int last_bias_mid = MID_W;
static int16 last_slope_x10 = 0;

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

static uint8 pixel_is_white(int row, int col)
{
    row = clamp_i(row, 0, MT9V03X_H - 1);
    col = clamp_i(col, 0, MT9V03X_W - 1);
    return PIXEL_IS_WHITE_FAST(row, col) ? 1 : 0;
}

static uint8 otsu_threshold(void)
{
    static uint16 hist[256];
    uint16 row;
    uint16 col;
    uint16 i;
    uint32 total;
    uint32 sum_all;
    uint32 sum_back;
    uint32 weight_back;
    uint32 weight_fore;
    uint8 threshold;
    float mean_back;
    float mean_fore;
    float diff;
    float variance;
    float max_variance;

    for (i = 0; i < 256; i++)
    {
        hist[i] = 0;
    }

    total = 0;
    for (row = 0; row < MT9V03X_H; row += CAMERA_THRESHOLD_ROW_STEP)
    {
        for (col = 0; col < MT9V03X_W; col += CAMERA_THRESHOLD_COL_STEP)
        {
            hist[mt9v03x_image[row][col]]++;
            total++;
        }
    }

    if (total == 0)
    {
        return img_threshold;
    }

    sum_all = 0;
    for (i = 0; i < 256; i++)
    {
        sum_all += (uint32)i * (uint32)hist[i];
    }

    threshold = img_threshold;
    sum_back = 0;
    weight_back = 0;
    max_variance = 0.0f;

    for (i = 0; i < 256; i++)
    {
        weight_back += hist[i];
        if (weight_back == 0)
        {
            continue;
        }

        weight_fore = total - weight_back;
        if (weight_fore == 0)
        {
            break;
        }

        sum_back += (uint32)i * (uint32)hist[i];
        mean_back = (float)sum_back / (float)weight_back;
        mean_fore = (float)(sum_all - sum_back) / (float)weight_fore;
        diff = mean_back - mean_fore;
        variance = (float)weight_back * (float)weight_fore * diff * diff;

        if (variance > max_variance)
        {
            max_variance = variance;
            threshold = (uint8)i;
        }
    }

    return threshold;
}

#if CAMERA_BINARY_OUTPUT_ENABLE
static void make_binary_snapshot(void)
{
    uint16 row;
    uint16 col;

    for (row = 0; row < MT9V03X_H; row++)
    {
        for (col = 0; col < MT9V03X_W; col++)
        {
            bin_image[row][col] = PIXEL_IS_WHITE_FAST(row, col) ? IMAGE_WHITE : IMAGE_BLACK;
        }
    }
}
#endif

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

static void scan_one_row(int row, int *mid_io, int *left_out, int *right_out,
                         uint8 *left_ok, uint8 *right_ok)
{
    int col;
    int mid;
    uint8 flag_l;
    uint8 flag_r;

    mid = clamp_i(*mid_io, 1, MT9V03X_W - 2);
    flag_l = 0;
    flag_r = 0;
    *left_out = 0;
    *right_out = MT9V03X_W - 1;

    if (!PIXEL_IS_WHITE_FAST(row, mid))
    {
        for (col = mid; col - BORDER_BIAS > 0; col--)
        {
            if (PIXEL_IS_WHITE_FAST(row, col) && PIXEL_IS_WHITE_FAST(row, col - BORDER_BIAS))
            {
                *right_out = col;
                flag_r = 1;
                break;
            }
        }

        if (flag_r)
        {
            for (; col - BORDER_BIAS > 0; col--)
            {
                if (!PIXEL_IS_WHITE_FAST(row, col) && !PIXEL_IS_WHITE_FAST(row, col - BORDER_BIAS))
                {
                    *left_out = col;
                    flag_l = 1;
                    break;
                }
            }
        }
        else
        {
            for (col = mid; col + BORDER_BIAS < MT9V03X_W - 1; col++)
            {
                if (PIXEL_IS_WHITE_FAST(row, col) && PIXEL_IS_WHITE_FAST(row, col + BORDER_BIAS))
                {
                    *left_out = col;
                    flag_l = 1;
                    break;
                }
            }

            if (flag_l)
            {
                for (; col + BORDER_BIAS < MT9V03X_W - 1; col++)
                {
                    if (!PIXEL_IS_WHITE_FAST(row, col) && !PIXEL_IS_WHITE_FAST(row, col + BORDER_BIAS))
                    {
                        *right_out = col;
                        flag_r = 1;
                        break;
                    }
                }
            }
        }
    }
    else
    {
        for (col = mid; col - BORDER_BIAS > 0; col--)
        {
            if (!PIXEL_IS_WHITE_FAST(row, col) && !PIXEL_IS_WHITE_FAST(row, col - BORDER_BIAS))
            {
                *left_out = col;
                flag_l = 1;
                break;
            }
        }

        for (col = mid; col + BORDER_BIAS < MT9V03X_W - 1; col++)
        {
            if (!PIXEL_IS_WHITE_FAST(row, col) && !PIXEL_IS_WHITE_FAST(row, col + BORDER_BIAS))
            {
                *right_out = col;
                flag_r = 1;
                break;
            }
        }
    }

    if (!flag_l)
    {
        *left_out = 0;
    }
    if (!flag_r)
    {
        *right_out = MT9V03X_W - 1;
    }
    if (*left_out >= *right_out)
    {
        *left_out = 0;
        *right_out = MT9V03X_W - 1;
        flag_l = 0;
        flag_r = 0;
    }

    *mid_io = (*left_out + *right_out) / 2;
    *left_ok = flag_l;
    *right_ok = flag_r;
}

void find_jidian(void)
{
    int row;
    int mid;
    int left;
    int right;
    uint8 left_ok;
    uint8 right_ok;

    row = clamp_i(jidian_search_line - 1, search_end_line + 1, MT9V03X_H - 1);
    mid = MID_W;
    scan_one_row(row, &mid, &left, &right, &left_ok, &right_ok);

    left_jidian = (uint8)clamp_i(left, 0, MT9V03X_W - 1);
    right_jidian = (uint8)clamp_i(right, 0, MT9V03X_W - 1);
    scan_mid = (uint8)clamp_i(mid, 1, MT9V03X_W - 2);
}

void image_deal(void)
{
    int row;
    int mid;
    int left;
    int right;
    int total_rows;
    int valid_rows;
    int fill_row;
    uint8 left_ok;
    uint8 right_ok;

    for (row = 0; row < MT9V03X_H; row++)
    {
        left_line_list[row] = 0;
        right_line_list[row] = MT9V03X_W - 1;
        mid_line_list[row] = MID_W;
    }

    camera_lost_left_cnt = 0;
    camera_lost_right_cnt = 0;
    total_rows = 0;
    valid_rows = 0;
    mid = scan_mid;

    for (row = search_start_line - 1; row > search_end_line; row -= CAMERA_ROW_STEP)
    {
        scan_one_row(row, &mid, &left, &right, &left_ok, &right_ok);

        left_line_list[row] = (uint8)left;
        right_line_list[row] = (uint8)right;
        mid_line_list[row] = (uint8)mid;

        for (fill_row = row - 1; (fill_row > row - CAMERA_ROW_STEP) && (fill_row > search_end_line); fill_row--)
        {
            left_line_list[fill_row] = (uint8)left;
            right_line_list[fill_row] = (uint8)right;
            mid_line_list[fill_row] = (uint8)mid;
        }

        total_rows += CAMERA_ROW_STEP;
        if (left_ok && right_ok)
        {
            valid_rows += CAMERA_ROW_STEP;
        }
        if (!left_ok)
        {
            camera_lost_left_cnt = (uint8)clamp_i((int)camera_lost_left_cnt + CAMERA_ROW_STEP, 0, 255);
        }
        if (!right_ok)
        {
            camera_lost_right_cnt = (uint8)clamp_i((int)camera_lost_right_cnt + CAMERA_ROW_STEP, 0, 255);
        }
    }

    scan_mid = (uint8)clamp_i(mid, 1, MT9V03X_W - 2);
    camera_valid_line_cnt = (uint8)clamp_i(valid_rows, 0, 255);
    if (total_rows > 0)
    {
        camera_confidence = (uint8)clamp_i((valid_rows * 100) / total_rows, 0, 100);
    }
    else
    {
        camera_confidence = 0;
    }
}

static int avg_mid_window(int center_row, int radius)
{
    int row;
    int start_row;
    int end_row;
    int sum_mid;
    int cnt;

    center_row = clamp_i(center_row, search_end_line + 1, search_start_line - 1);
    start_row = clamp_i(center_row - radius, search_end_line + 1, search_start_line - 1);
    end_row = clamp_i(center_row + radius, search_end_line + 1, search_start_line - 1);

    sum_mid = 0;
    cnt = 0;
    for (row = start_row; row <= end_row; row++)
    {
        sum_mid += mid_line_list[row];
        cnt++;
    }

    if (cnt <= 0)
    {
        return mid_line_list[center_row];
    }
    return sum_mid / cnt;
}

static int fixed_row_bias_mid(int startline, int endline)
{
    int row;
    int sum_mid;
    int cnt;
    int mid;

    startline = clamp_i(startline, search_end_line + 1, search_start_line - 1);
    endline = clamp_i(endline, search_end_line, startline - 1);
    sum_mid = 0;
    cnt = 0;

    for (row = startline; row > endline; row--)
    {
        mid = mid_line_list[row];
        if (!PIXEL_IS_WHITE_FAST(row, mid))
        {
            break;
        }
        if ((row + 1 < MT9V03X_H) && (abs_i(mid_line_list[row] - mid_line_list[row + 1]) > MT9V03X_W / 3))
        {
            break;
        }

        sum_mid += mid;
        cnt++;
    }

    if (cnt >= CAMERA_MIN_BIAS_ROWS)
    {
        last_bias_mid = sum_mid / cnt;
    }
    return last_bias_mid;
}

static int16 regression_slope_x10(int startline, int endline)
{
    int row;
    int actual_endline;
    int sum_x;
    int sum_y;
    int cnt;
    float avg_x;
    float avg_y;
    float sum_up;
    float sum_down;
    float slope;

    startline = clamp_i(startline, search_end_line + 1, search_start_line - 1);
    endline = clamp_i(endline, search_end_line, startline - 1);
    actual_endline = endline;
    sum_x = 0;
    sum_y = 0;
    cnt = 0;

    for (row = startline; row > endline; row--)
    {
        if (!PIXEL_IS_WHITE_FAST(row, mid_line_list[row]))
        {
            actual_endline = row;
            break;
        }
        if ((row + 1 < MT9V03X_H) && (abs_i(mid_line_list[row] - mid_line_list[row + 1]) > MT9V03X_W / 3))
        {
            actual_endline = row;
            break;
        }

        sum_x += row;
        sum_y += mid_line_list[row];
        cnt++;
    }

    if (cnt <= CAMERA_MIN_BIAS_ROWS)
    {
        return last_slope_x10;
    }

    avg_x = (float)sum_x / (float)cnt;
    avg_y = (float)sum_y / (float)cnt;
    sum_up = 0.0f;
    sum_down = 0.0f;

    for (row = startline; row > actual_endline; row--)
    {
        sum_up += ((float)mid_line_list[row] - avg_y) * ((float)row - avg_x);
        sum_down += ((float)row - avg_x) * ((float)row - avg_x);
    }

    if (sum_down == 0.0f)
    {
        slope = 0.0f;
    }
    else
    {
        slope = sum_up / sum_down;
    }

    last_slope_x10 = (int16)clamp_i((int)(slope * 10.0f), -90, 90);
    return last_slope_x10;
}

uint8 find_mid_line_weight(void)
{
    int bias_mid;
    int near_mid;
    int preview_mid;
    int far_mid;
    int preview;
    int preview_far;
    int curve;
    int slope_x10;
    uint8 route_mode;
    uint8 ea_state;

    bias_mid = fixed_row_bias_mid(CAMERA_BIAS_START_LINE, CAMERA_BIAS_END_LINE);
    near_mid = avg_mid_window(CAMERA_PREVIEW_NEAR_ROW, CAMERA_AVG_RADIUS);
    preview_mid = avg_mid_window(CAMERA_PREVIEW_MID_ROW, CAMERA_AVG_RADIUS);
    far_mid = avg_mid_window(CAMERA_PREVIEW_FAR_ROW, CAMERA_AVG_RADIUS);

    preview = preview_mid - near_mid;
    preview_far = far_mid - near_mid;
    curve = far_mid - 2 * preview_mid + near_mid;
    slope_x10 = regression_slope_x10(CAMERA_BIAS_START_LINE, CAMERA_BIAS_END_LINE);

    route_mode = 0;
    if ((abs_i(preview) >= 8) || (abs_i(preview_far) >= 12) || (abs_i(curve) >= 10))
    {
        route_mode = 2;
    }
    if ((abs_i(preview) >= 8) && (abs_i(preview_far) >= 12) &&
        (sign_i(preview) != 0) && (sign_i(preview_far) != 0) &&
        (sign_i(preview) != sign_i(preview_far)))
    {
        route_mode = 1;
    }

    ea_state = EA;
    EA = 0;
    final_mid_line = (uint8)clamp_i(bias_mid, 1, MT9V03X_W - 2);
    camera_bias_raw = (int16)clamp_i(bias_mid - MID_W, -90, 90);
    camera_preview_raw = (int16)clamp_i(preview, -90, 90);
    camera_preview_far_raw = (int16)clamp_i(preview_far, -90, 90);
    camera_curve_raw = (int16)clamp_i(curve + slope_x10, -90, 90);
    camera_route_mode = route_mode;
    EA = ea_state;

    return final_mid_line;
}

#if CAMERA_DEBUG_DRAW_ENABLE
static void draw_debug_overlay(void)
{
    int row;
    int px;
    int py;

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
    static uint8 threshold_div = 0;

    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        if (threshold_div == 0)
        {
            img_threshold = otsu_threshold();
        }
        threshold_div++;
        if (threshold_div >= CAMERA_THRESHOLD_UPDATE_DIV)
        {
            threshold_div = 0;
        }

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
    static uint8 threshold_div = 0;

    if (mt9v03x_finish_flag)
    {
        mt9v03x_finish_flag = 0;

        if (threshold_div == 0)
        {
            img_threshold = otsu_threshold();
        }
        threshold_div++;
        if (threshold_div >= CAMERA_THRESHOLD_UPDATE_DIV)
        {
            threshold_div = 0;
        }

        find_jidian();
        image_deal();
        (void)find_mid_line_weight();
        mark_frame_processed();

        make_binary_snapshot();
        seekfree_assistant_camera_send();
    }
}

#else
#error "IPS200_OR_WIFI must be 0 or 1."
#endif
