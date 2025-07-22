#ifndef WTR_CAN_H
#define WTR_CAN_H


#include "DJI.h"
#include "can.h"

extern int32_t Encoder_value;
extern int32_t Encoder_value_last;
extern int32_t Encoder_value_diff;
extern int64_t Encoder_value_sum;
extern float Encoder_speed;
extern int8_t first_flag;
HAL_StatusTypeDef CANFilterInit(CAN_HandleTypeDef* hcan);


#endif
