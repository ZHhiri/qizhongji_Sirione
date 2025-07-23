#ifndef _CACULATE_H__
#define _CACULATE_H__

#include "DJI.h"

void positionServo(float ref, DJI_t * motor);

void speedServo(float ref, DJI_t * motor);

void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle);

typedef struct {
    float Position[4];     // �ĸ�λ�õ㣺��㡢���ٽ����㡢���ٽ����㣨���ٿ�ʼ�㣩���յ�
    float Time[3];         // ����ʱ��㣺���ٽ���ʱ�䡢���ٽ���ʱ�䣨���ٿ�ʼʱ�䣩����ʱ��
    float MaxVelocity;
    float MaxAccelerate;
    float DeltaPosition;
    char type;             // �Ƿ������ٶΣ�0-�ޣ�1-��
    char direction;        // �˶�����1-����-1-����
    float CurrPosition;    // ��ǰλ��
} VelocityPlan_T;

void VelocityPlan_Init(VelocityPlan_T* plan, float StartPosition, float StopPosition, float MaxAccelerate, float MaxVelocity);
void RS485_positionServo(float ref, DJI_t * motor,float fdb);
float CalcPos(VelocityPlan_T* plan, float NowTime);
void positionServo_2(float ref, DJI_t * motor,DJI_t * motor_2);
#endif
