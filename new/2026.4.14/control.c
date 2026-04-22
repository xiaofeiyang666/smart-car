#include "control.h"
#include "pid.h"
#include "encoder.h"
#include "motor.h"
#include "servo.h"   
#include "camera.h"
#include "brushless.h"
#include "camera_new.h"

// 在这里定义最大脉冲数，让 control.c 认识它
#define MAX_SPEED_PULSES 4000  

volatile uint8 print_flag = 0;

float servo_kp = 1.2;  // 摄像头偏差比例系数，2026.4.14添加这里
float servo_kd = 0.5;  // 陀螺仪阻尼系数 ，2026.4.14添加这里

// 目标速度
int target_speed_base = 3;   

PID servo_pid = {0};          

void control_init(void)
{
	
    left_motor_speed_pid_init(0.10, 0.002, 0.0, 90, 90); 
    right_motor_speed_pid_init(0.10, 0.002, 0.0, 90, 90);
    
    PID_Init(&servo_pid, 1.5, 0, 1.0, 0, 40); 
}

//void control_loop(void)
//{
//    int target_pulses;
//    static uint8 loop_cnt = 0;
//    
//    float current_angle;

//		int offset = final_mid_line - MID_W;

//    // 变量声明完毕后，再执行这句乘除法映射计算
//    target_pulses = (int)(((long)target_speed_base * MAX_SPEED_PULSES) / 90);

//     //1. 舵机转向控制 
//    PID_Calc(&servo_pid, 0, offset); 
//    current_angle = SERVO_CENTER + servo_pid.output;
//    servo_set_angle(current_angle);

//    // 2. 读编码器 (此时 left_speed 是最原始、最高精度的真实脉冲)
//    encoder_update(); 
//    
//    // 3. 将放大后的高精度目标脉冲 (target_pulses) 喂给 PID
//    left_motor_speed_pid_calc(target_pulses, left_speed);
//    right_motor_speed_pid_calc(target_pulses, right_speed);
//    
//    // 4. 将 PID 算出来的占空比给电机
//    set_motor_speed((int)left_motor_speedpid.output, (int)right_motor_speedpid.output);
//        
//    // set_fan_speed(600,600); 
//    
//    loop_cnt++;
//    if(loop_cnt >= 5) 
//    {
//        loop_cnt = 0;
//        print_flag = 1;
//    }
//}

//void control_loop(void)
//{
//    int target_pulses;
//    static uint8 loop_cnt = 0;
//    float current_angle;
//    
//    // 变量声明
//    int offset = final_mid_line - MID_W;
//    int diff_speed;         // 差速大小
//    float diff_k = 1.1;     // 差速系数（数值越大，两轮转速差越大）

//    // 计算基础目标脉冲
//    target_pulses = (int)(((long)target_speed_base * MAX_SPEED_PULSES) / 90);

//    // 舵机转向控制 
//    PID_Calc(&servo_pid, 0, offset); 
//    current_angle = SERVO_CENTER + servo_pid.output;
//    servo_set_angle(current_angle);

//    // ================== 【差速】 ==================
//    // 舵机的输出 (servo_pid.output) 直接反映了当前弯道的急缓。
//    // 弯越急，servo_pid.output 越大，计算出的 diff_speed 也越大。
//    diff_speed = (int)(servo_pid.output * diff_k);
//    // ==========================================================

//    // 读编码器真实脉冲
//    encoder_update(); 
//    
//    // 5. 将带有差速的目标脉冲喂给左右电机的 PID
//    // 假设左转时 servo_pid.output 为负，那么左轮（内侧）减速，右轮（外侧）加速。
//    // 如果加上这段代码后，发现车子过弯更僵硬了（推头），
//    // 说明加减号和你的硬件方向反了，只需要把下面的 + 和 - 对调一下即可！
//    left_motor_speed_pid_calc(target_pulses + diff_speed, left_speed);
//    right_motor_speed_pid_calc(target_pulses - diff_speed, right_speed);
//    
//    // 将 PID 算出来的占空比给电机
//    set_motor_speed((int)left_motor_speedpid.output, (int)right_motor_speedpid.output);
//        
//    // 打印逻辑
//    loop_cnt++;
//    if(loop_cnt >= 5) 
//    {
//        loop_cnt = 0;
//        print_flag = 1;
//    }
//}

void control_loop(void)//2026.4.14添加这里
{
    int target_pulses;
    static uint8 loop_cnt = 0;
    float current_angle;
    float servo_out;
    float gyro_z_actual;
    
    int offset = final_mid_line - MID_W; // 摄像头偏差
    int diff_speed;         
    float diff_k = 1.1;     

    // 1. 获取最新陀螺仪数据
    imu660ra_get_gyro();
    // 转换为实际物理数据 (°/s)，提高直观性
    gyro_z_actual = imu660ra_gyro_transition(imu660ra_gyro_z);

    // ================== 【核心：串级/融合 PD 转向控制】 ==================
    // 逻辑：偏差越大，舵机打角越大；但车身自旋越快，越要阻止舵机继续打角
    // 注意：这里的正负号 (+ 或 -) 极其重要，必须根据你陀螺仪的安装方向实际测试！
    // 验证方法：用手左右快速摆动车头，如果舵机跟着摆动方向抵抗（反打），说明符号对了。
    servo_out = (servo_kp * offset) - (servo_kd * gyro_z_actual);
    
    current_angle = SERVO_CENTER + servo_out;
    servo_set_angle(current_angle);

    // ================== 【差速补偿配合】 ==================
    // 差速仍然跟随舵机的最终输出，保证了转向意图和动力的一致性
    diff_speed = (int)(servo_out * diff_k);

    // 基础目标脉冲计算
    target_pulses = (int)(((long)target_speed_base * MAX_SPEED_PULSES) / 90);

    // 编码器更新
    encoder_update(); 
    
    // 给左右电机加上差速 (同样需要注意实际转向时的内外轮关系)
    left_motor_speed_pid_calc(target_pulses + diff_speed, left_speed);
    right_motor_speed_pid_calc(target_pulses - diff_speed, right_speed);
    
    set_motor_speed((int)left_motor_speedpid.output, (int)right_motor_speedpid.output);
        
    loop_cnt++;
    if(loop_cnt >= 5) 
    {
        loop_cnt = 0;
        print_flag = 1;
    }
}
