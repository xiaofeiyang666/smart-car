#include "pid.h"

PID left_motor_speedpid = {0};
PID right_motor_speedpid = {0};

static float pid_limit(float x, float min_v, float max_v)
{
    if (x < min_v) return min_v;
    if (x > max_v) return max_v;
    return x;
}

void PID_Init(PID *pid, float p, float i, float d, int maxI, int maxOut)
{
    pid->kp = p;
    pid->ki = i;
    pid->kd = d;
    pid->error = 0.0f;
    pid->lastError = 0.0f;
    pid->prevError = 0.0f;
    pid->maxIntegral = (float)maxI;
    pid->output = 0.0f;
    pid->maxOutput = (float)maxOut;
    pid->output_integral = 0.0f;
}

void PID_Calc(PID *pid, int reference, int feedback)
{
    float delta;

    pid->error = (float)(reference - feedback);

    delta = pid->kp * (pid->error - pid->lastError) +
            pid->ki * pid->error +
            pid->kd * (pid->error - 2.0f * pid->lastError + pid->prevError);

    pid->output += delta;
    pid->output = pid_limit(pid->output, -50.0f, pid->maxOutput);

    pid->prevError = pid->lastError;
    pid->lastError = pid->error;
}

void left_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut)
{
    PID_Init(&left_motor_speedpid, p, i, d, maxI, maxOut);
}

void right_motor_speed_pid_init(float p, float i, float d, int maxI, int maxOut)
{
    PID_Init(&right_motor_speedpid, p, i, d, maxI, maxOut);
}

void left_motor_speed_pid_calc(int speed, int speed_fd)
{
    PID_Calc(&left_motor_speedpid, speed, speed_fd);
}

void right_motor_speed_pid_calc(int speed, int speed_fd)
{
    PID_Calc(&right_motor_speedpid, speed, speed_fd);
}
