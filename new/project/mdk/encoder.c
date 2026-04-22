#include "encoder.h"
#include <stdlib.h> // 需要用到 abs() 绝对值函数

//用于给摄像头算法累计行驶距离的全局变量
volatile int32 accumulated_distance = 0;

// 编码器引脚定义
#define ENCODER_QUAD_1      ( PWMA_ENCODER )            
#define ENCODER_QUAD_1_CHA  ( PWMA_ENCODER_CH1P_P60 )   
#define ENCODER_QUAD_1_CHB  ( PWMA_ENCODER_CH2P_P62 )   

#define ENCODER_QUAD_2      ( PWMC_ENCODER )            
#define ENCODER_QUAD_2_CHA  ( PWMC_ENCODER_CH1P_P40 )   
#define ENCODER_QUAD_2_CHB  ( PWMC_ENCODER_CH2P_P42 )   

// 真正定义这两个速度变量的地方
int16 left_speed = 0;
int16 right_speed = 0;
int16 left_speed_raw = 0;
int16 right_speed_raw = 0;

// 编码器初始化函数
void encoder_init(void)
{
    encoder_quad_init(ENCODER_QUAD_1, ENCODER_QUAD_1_CHA, ENCODER_QUAD_1_CHB); 
    encoder_quad_init(ENCODER_QUAD_2, ENCODER_QUAD_2_CHA, ENCODER_QUAD_2_CHB); 
}

// 速度更新函数（应该在定时器中断中被调用）
void encoder_update(void)
{
    left_speed_raw  = encoder_get_count(ENCODER_QUAD_1); // 读取并修正左侧方向
    right_speed_raw = -encoder_get_count(ENCODER_QUAD_2);
		left_speed = 	left_speed_raw;
		right_speed = right_speed_raw;
	
		accumulated_distance += (abs(left_speed_raw) + abs(right_speed_raw)) / 2;

    encoder_clear_count(ENCODER_QUAD_1); // 清空计数
    encoder_clear_count(ENCODER_QUAD_2); 
}

int16 Distance_Measure(void)
{
    int16 dist = accumulated_distance;
    accumulated_distance = 0; // 读完后清零，相当于测量两次调用之间的距离
    return dist;
}

//#include "encoder.h"
//#include <stdlib.h> // 需要用到 abs()

//volatile int32 accumulated_distance = 0;

//// 【新增】根据你实测的数据，定义最大脉冲数
//#define MAX_SPEED_PULSES 2000  

//// 编码器引脚定义 (保持你的原样)
//#define ENCODER_QUAD_1      ( PWMA_ENCODER )            
//#define ENCODER_QUAD_1_CHA  ( PWMA_ENCODER_CH1P_P60 )   
//#define ENCODER_QUAD_1_CHB  ( PWMA_ENCODER_CH2P_P62 )   

//#define ENCODER_QUAD_2      ( PWMC_ENCODER )            
//#define ENCODER_QUAD_2_CHA  ( PWMC_ENCODER_CH1P_P40 )   
//#define ENCODER_QUAD_2_CHB  ( PWMC_ENCODER_CH2P_P42 )   

//int16 left_speed = 0;
//int16 right_speed = 0;
//int16 left_speed_raw = 0;
//int16 right_speed_raw = 0;

//void encoder_init(void)
//{
//    encoder_quad_init(ENCODER_QUAD_1, ENCODER_QUAD_1_CHA, ENCODER_QUAD_1_CHB); 
//    encoder_quad_init(ENCODER_QUAD_2, ENCODER_QUAD_2_CHA, ENCODER_QUAD_2_CHB); 
//}

//void encoder_update(void)
//{
//    left_speed_raw  = encoder_get_count(ENCODER_QUAD_1); 
//    right_speed_raw = -encoder_get_count(ENCODER_QUAD_2);
//    
//    // 【核心修改】将原始脉冲映射到 0~90！
//    // 强制转换为 long 进行运算，防止乘法溢出，算完再转回 int16
//    left_speed = (int16)(((long)left_speed_raw * 90) / MAX_SPEED_PULSES);
//    right_speed = (int16)(((long)right_speed_raw * 90) / MAX_SPEED_PULSES);
//    
//    // 累加距离还是用原始脉冲比较好，精度高
//    accumulated_distance += (abs(left_speed_raw) + abs(right_speed_raw)) / 2;

//    encoder_clear_count(ENCODER_QUAD_1); 
//    encoder_clear_count(ENCODER_QUAD_2); 
//}

//int16 Distance_Measure(void)
//{
//    int16 dist = accumulated_distance;
//    accumulated_distance = 0; 
//    return dist;
//}