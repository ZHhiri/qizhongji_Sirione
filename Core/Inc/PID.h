#ifndef __PID_H__
#define __PID_H__

#include<stdio.h>

typedef struct 
{
    float KP;        // PID参数P
    float KI;        // PID参数I
    float KD;        // PID参数D
    float fdb;       // PID反馈值
    float ref;       // PID目标值
    float cur_error; // 当前误差
    float error[2];  // 前两次误差
    float integral;  // 积分
    float integralMax;  // 积分上限
    float integralMin;  // 积分下限 用于积分饱和
    float output;    // 输出值
    float outputMax; // 最大输出值的绝对值
    float outputMin; // 最小输出值的绝对值用于防抖
}PID_t;
void P_Calc(PID_t *pid);
void IncrPID_Calc(PID_t *pid);
void PosePID_Calc(PID_t *pid);
void PD_Calc(PID_t *pid);
//void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle);
#endif
