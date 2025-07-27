
#ifndef __PARAM_H__
#define __PARAM_H__
#include "decode.h"
#include "DJI.h"
#include "cmsis_os.h"
typedef struct 
{
    double x;
    double y;
    double z;
    double yaw;
}gantryposition;

typedef struct
{
    DJI_t *Motor_X;
    DJI_t *Motor_Y;
    DJI_t *Motor_Y2;
    DJI_t *Motor_Z;
    DJI_t *Motor_yaw;
    gantryposition gantrypos;
}gantrystate;

extern  uint16_t runflag;
extern float Wheel_StartPos[5];
extern gantrystate mygantry; 
extern TickType_t WheelCorrect_NowTick ;
extern TickType_t WheelCorrect_StartTick;
extern  TickType_t WheelCorrect_StartTick_2;
extern  TickType_t WheelCorrect_NowTick_2;
//extern uint8_t usart1_rx[1];
extern uint8_t usart3_rx[1];

//extern uint8_t Rxbuffer_1[195];
extern uint8_t Rxbuffer_2[195];

extern LidarPointTypedef Lidar1;
extern LidarPointTypedef Lidar2;


#endif // !__PARAM_H__PARAM_H__