/*
 * pid.c
 *
 *  Created on: 2025年1月15日
 *      Author: j1912
 */
#include "pid.h"
#include <stdio.h>
#include <math.h>
//首先定义PID结构体用于存放一个PID的数据
#define BASE_SPEED  20
PID left_motor_speedpid = {0};             //创建一个PID结构体变量
PID right_motor_speedpid = {0};            //创建一个PID结构体变量
PID positionpid = {0};                     //创建一个PID结构体变量
//用于初始化pid参数的函数
void PID_Init(PID *pid, float p, float i, float d, int maxI, int maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->maxIntegral = maxI;
    pid->maxOutput = maxOut;
}

//进行一次pid计算
//参数为(pid结构体,目标值,反馈值)，计算结果放在pid结构体的output成员中
void PID_Calc(PID *pid, int reference, int feedback)
{
    float dout;
    float pout;
    float integral_next;
    float output_next;

    //更新数据
    pid->lastError = pid->error; //将旧error存起来
    pid->error = (float)(reference - feedback); //计算新error

    dout = (pid->error - pid->lastError) * pid->kd;
    pout = pid->error * pid->kp;

    integral_next = pid->integral + pid->error * pid->ki;
    if(integral_next > pid->maxIntegral) integral_next = pid->maxIntegral;
    else if(integral_next < -pid->maxIntegral) integral_next = -pid->maxIntegral;

    output_next = pout + dout + integral_next;

    if(output_next > pid->maxOutput)
    {
        pid->output = pid->maxOutput;
        if(pid->error < 0.0f)
        {
            pid->integral = integral_next;
        }
    }
    else if(output_next < -pid->maxOutput)
    {
        pid->output = -pid->maxOutput;
        if(pid->error > 0.0f)
        {
            pid->integral = integral_next;
        }
    }
    else
    {
        pid->integral = integral_next;
        pid->output = output_next;
    }
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     左电机pid初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------

void left_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut)
{
    PID_Init(&left_motor_speedpid,p,i,d,maxI,maxOut);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     右电机pid初始化
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------

void right_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut)
{
    PID_Init(&right_motor_speedpid,p,i,d,maxI,maxOut);
}

//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算左电机pid
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------

void left_motor_speed_pid_calc( int speed, int speed_fd)
{
    PID_Calc(&left_motor_speedpid,speed,speed_fd);
    //left_motor_control(left_motor_speedpid.output);
    //printf("%d",left_motor_speedpid.output_integral);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     计算右电机pid
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------

void right_motor_speed_pid_calc( int speed, int speed_fd)
{
    PID_Calc(&right_motor_speedpid,speed,speed_fd);
    //right_motor_control(right_motor_speedpid.output);
}
//-------------------------------------------------------------------------------------------------------------------
// 函数简介     初始化位置pid
// 参数说明     void
// 返回参数     void
//-------------------------------------------------------------------------------------------------------------------

void position_pid_init(float p, float i, float d, int maxI, int maxOut)
{
    PID_Init(&positionpid,p,i,d,maxI,maxOut);
}

//int error ,last_error;
//extern volatile int16_t my_Mid_Line[MT9V03X_H];
//volatile int16_t count_mid_point;
////-------------------------------------------------------------------------------------------------------------------
//// 函数简介     位置pid,通过中线偏差得到error,并给出差速，最终这个函数在encoder.c中调用
//// 参数说明     void
//// 返回参数     void
////-------------------------------------------------------------------------------------------------------------------

//void position_pid_calc(void)
//{
//    error = 0;
//    if(count_mid_point>60) count_mid_point=60;
//    if(count_mid_point >=10)
//    {
//        for(int i=0;i<count_mid_point;i++)
//        {
//          error += (94-my_Mid_Line[i]);
//        }
//        error = error/count_mid_point;
//        last_error = error;
//    }
//    else
//    {
//        error = last_error;
//    }
//    position_pid_init((pow(abs(error)/10.0,3)),0,pow(abs(error)/10.0,4),0,BASE_SPEED);                                   //位置pid初始化
//    PID_Calc(&positionpid,error,0);
//    if(positionpid.output < 0)                  //误差小于0右转
//    {
//        left_motor_speed_pid_calc(BASE_SPEED+abs(positionpid.output),encoder_data_2);
//        right_motor_speed_pid_calc(BASE_SPEED-abs(positionpid.output),encoder_data_4);
////        right_motor_speed_pid_calc(200,encoder_data_4);
//    }
//    else if(positionpid.output > 0)                  //误差大于0左转
//    {
////        left_motor_speed_pid_calc(200,encoder_data_2);
//        left_motor_speed_pid_calc(BASE_SPEED-abs(positionpid.output),encoder_data_2);
//        right_motor_speed_pid_calc(BASE_SPEED+abs(positionpid.output),encoder_data_4);
//    }
//    else
//    {
//        left_motor_speed_pid_calc(BASE_SPEED,encoder_data_2);
//        right_motor_speed_pid_calc(BASE_SPEED,encoder_data_4);
//    }

//}