#ifndef CODE_PID_H_
#define CODE_PID_H_

#include "zf_common_headfile.h"

typedef struct
{
    float kp;
    float ki;
    float kd;
    float error;
    float lastError;
    float prevError;
    float maxIntegral;
    float output;
    float maxOutput;
    float output_integral;
} PID;

extern PID left_motor_speedpid;
extern PID right_motor_speedpid;

void PID_Init(PID *pid, float p, float i, float d, int maxI, int maxOut);
void PID_Calc(PID *pid, int reference, int feedback);

void left_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut);
void right_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut);

void left_motor_speed_pid_calc(int speed, int speed_fd);
void right_motor_speed_pid_calc(int speed, int speed_fd);

#endif /* CODE_PID_H_ */
