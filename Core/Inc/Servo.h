#ifndef SERVO_H
#define SERVO_H
#include "stm32f4xx_hal.h"
#include "tim.h"

extern TIM_HandleTypeDef htim1;

//舵机控制的角度范围
//CCR = 1000 + (角度 + 90) × (4000 / 180)

#define SERVO_ON_ANGLE 3800
#define SERVO_OFF_ANGLE 4300
#define SERVO_MIN_ANGLE 2000 
#define SERVO_MAX_ANGLE 4500 

//采用的是TIM1的通道3:PSC=83 ARR=39999

void Servo_Init(TIM_HandleTypeDef *htim , uint32_t Channel);//初始化舵机
// Servo_Init函数需要传入TIM_HandleTypeDef和通道号
// htim1是TIM1的句柄，Channel是TIM通道号（如TIM_CHANNEL_3）

void Servo_SetAngle(uint16_t angle);//设置舵机角度
void Servo_On(void);//舵机收紧：使爪子夹紧牛奶盒
void Servo_Off(void);//舵机放松：使爪子松开牛奶盒

#endif // SERVO_H