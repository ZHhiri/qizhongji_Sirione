#include "upperservo.h"
#include "Caculate.h"
#include "param.h"
#include "DJI.h"
#include "decode.h"
#include "cmsis_os.h"
#include "wtr_can.h"
#include "math.h"
#include <stdbool.h>
#define maxspeed        100000//300000200000
#define maxAcceleration 45000//7000050000
#define maxspeed_2        300000
#define maxAcceleration_2 100000//130000
gantrystate mygantry;
float Wheel_StartPos[5]={0};
TickType_t WheelCorrect_StartTick;
TickType_t WheelCorrect_NowTick;
TickType_t WheelCorrect_StartTick_2;
TickType_t WheelCorrect_NowTick_2;
void upperservotask(void const *argument)
{
    /* USER CODE BEGIN upperservotask */
    mygantry.gantrypos.x = 706;//706
    mygantry.gantrypos.y = 0;
    osDelay(300);
     WheelCorrect_StartTick = xTaskGetTickCount();
     WheelCorrect_StartTick_2 = xTaskGetTickCount();
    Wheel_StartPos[1]=hDJI[1].posPID.fdb;
    Wheel_StartPos[2]=hDJI[2].posPID.fdb;
    Wheel_StartPos[4]=hDJI[4].posPID.fdb;
    /* Infinite loop */
    for (;;) {
         WheelCorrect_NowTick     = xTaskGetTickCount();
         WheelCorrect_NowTick_2     = xTaskGetTickCount();
        TickType_t WheelCorrect_ElapsedTick = WheelCorrect_NowTick - WheelCorrect_StartTick;
        TickType_t WheelCorrect_ElapsedTick_2 = WheelCorrect_NowTick_2 - WheelCorrect_StartTick_2;
        float timeSec                       = (WheelCorrect_ElapsedTick / (1000.0)); // 获取当前时间/s
        float timeSec_2                     = (WheelCorrect_ElapsedTick_2/ (1000.0)); // 获取当前时间/s
         VelocityPlanning( Wheel_StartPos[1],
                             maxspeed ,
                             maxAcceleration,
                             mygantry.gantrypos.y*8191, timeSec,
                             &hDJI[1].posPID.ref);
         VelocityPlanning( Wheel_StartPos[2] ,
                             maxspeed ,
                             maxAcceleration,
                             - mygantry.gantrypos.y * 8191, timeSec,
                             &hDJI[2].posPID.ref);
        VelocityPlanning( Wheel_StartPos[4],
                             maxspeed_2 ,
                             maxAcceleration_2,
                             mygantry.gantrypos.z*8191, timeSec_2,
                             &hDJI[4].posPID.ref);   
        // STP_23L_Decode(Rxbuffer_1, &Lidar1);//激光是长轴的a
        // STP_23L_Decode(Rxbuffer_2, &Lidar2);//激光是短轴的
        
		RS485_positionServo(mygantry.gantrypos.x/0.037, mygantry.Motor_X,Encoder_value/0.037);
        positionServo(hDJI[1].posPID.ref/8191, mygantry.Motor_Y);
        positionServo(hDJI[2].posPID.ref/8191.0f, mygantry.Motor_Y2);
        positionServo(hDJI[4].posPID.ref/8191, mygantry.Motor_Z);
        positionServo(mygantry.gantrypos.yaw, mygantry.Motor_yaw);
 

         CanTransmit_DJI_1234(&hcan1, mygantry.Motor_yaw->speedPID.output,
                             mygantry.Motor_Y->speedPID.output,
                             mygantry.Motor_Y2->speedPID.output,
                             mygantry.Motor_X->speedPID.output);

        CanTransmit_DJI_5678(&hcan1,
                             mygantry.Motor_Z->speedPID.output,
                             0,
                             0,
                             0);
        osDelay(1);
    }
    /* USER CODE END upperservotask */
}

void gantry_Motor_init() // 电机初始化
{
    hDJI[0].motorType = M2006; // yaw
    hDJI[1].motorType = M2006; // y
    hDJI[2].motorType = M2006; // y
    hDJI[3].motorType = M3508; // x
    hDJI[4].motorType = M3508; // z

    mygantry.Motor_X   = &hDJI[3];
    mygantry.Motor_Y   = &hDJI[1];
    mygantry.Motor_Y2   = &hDJI[2];
    mygantry.Motor_Z   = &hDJI[4];
    mygantry.Motor_yaw = &hDJI[0];

    DJI_Init();
    osDelay(1000);
    mygantry.Motor_X->speedPID.outputMax   = hDJI[3].speedPID.outputMax;
    mygantry.Motor_Y->speedPID.outputMax   = hDJI[1].speedPID.outputMax;
     mygantry.Motor_Y2->speedPID.outputMax   = hDJI[1].speedPID.outputMax;
    mygantry.Motor_Z->speedPID.outputMax   = hDJI[4].speedPID.outputMax;
    mygantry.Motor_yaw->speedPID.outputMax = hDJI[0].speedPID.outputMax;

    mygantry.Motor_X->posPID.outputMax   = hDJI[3].posPID.outputMax;
    mygantry.Motor_Y->posPID.outputMax   = hDJI[1].posPID.outputMax;
     mygantry.Motor_Y2->posPID.outputMax   = hDJI[1].posPID.outputMax;
    mygantry.Motor_Z->posPID.outputMax   = hDJI[4].posPID.outputMax;
    mygantry.Motor_yaw->posPID.outputMax = hDJI[0].posPID.outputMax;

    CANFilterInit(&hcan1);
    CANFilterInit(&hcan2);
}

// Function to reset PID parameters
void pid_reset(PID_t *pid, float kp, float ki, float kd)
{
    pid->KP = kp;
    pid->KI = ki;
    pid->KD = kd;
}
