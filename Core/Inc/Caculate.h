#ifndef _CACULATE_H__
#define _CACULATE_H__

#include "DJI.h"

void positionServo(float ref, DJI_t * motor);

void speedServo(float ref, DJI_t * motor);

void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle);

typedef struct {
    float Position[4];     // 四个位置点：起点、加速结束点、匀速结束点（减速开始点）、终点
    float Time[3];         // 三个时间点：加速结束时间、匀速结束时间（减速开始时间）、总时间
    float MaxVelocity;
    float MaxAccelerate;
    float DeltaPosition;
    char type;             // 是否有匀速段：0-无，1-有
    char direction;        // 运动方向：1-正向，-1-反向
    float CurrPosition;    // 当前位置
} VelocityPlan_T;

void VelocityPlan_Init(VelocityPlan_T* plan, float StartPosition, float StopPosition, float MaxAccelerate, float MaxVelocity);
void RS485_positionServo(float ref, DJI_t * motor,float fdb);
float CalcPos(VelocityPlan_T* plan, float NowTime);
void positionServo_2(float ref, DJI_t * motor,DJI_t * motor_2);
#endif
