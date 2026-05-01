///*
// * pid.h
// *
// *  Created on: 2025年1月15日
// *      Author: j1912
// */

//#ifndef CODE_PID_H_
//#define CODE_PID_H_
//#include "zf_common_headfile.h"



//typedef struct
//{
//    float kp, ki, kd; //三个系数
//    float error, lastError; //误差、上次误差
//    float integral, maxIntegral; //积分、积分限幅
//    float output, maxOutput; //输出、输出限幅
//    float output_integral;//输出累加
//}PID;

//extern PID left_motor_speedpid ; //创建一个PID结构体变量
//extern PID right_motor_speedpid ; //创建一个PID结构体变量

////extern int temp;
////extern uint8 flag_pid_finish;
//void left_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut);
//void right_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut);
//void left_motor_speed_pid_calc( int speed, int speed_fd);
//void right_motor_speed_pid_calc( int speed, int speed_fd);
////void position_pid_init(float p, float i, float d, int maxI, int maxOut);
////void position_pid_calc(void);

////extern PID positionpid; //创建一个PID结构体变量





//#endif /* CODE_PID_H_ */
#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

typedef struct
{
    float kp, ki, kd;            // 三个系数
    float error, lastError;      // 误差、上次误差
    float integral, maxIntegral; // 积分、积分限幅
    float output, maxOutput;     // 输出、输出限幅
    float output_integral;       // 输出累加
} PID;

// 暴露出两个电机的 PID 结构体变量
extern PID left_motor_speedpid; 
extern PID right_motor_speedpid;

void PID_Init(PID *pid, float p, float i, float d, int maxI, int maxOut);
void PID_Calc(PID *pid, int reference, int feedback);

// 速度环初始化函数
void left_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut);
void right_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut);
void left_motor_speed_pid_calc(int speed, int speed_fd);
void right_motor_speed_pid_calc(int speed, int speed_fd);


#endif /* CODE_PID_H_ */

