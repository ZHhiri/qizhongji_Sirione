#ifndef WTR_CAN_H
#define WTR_CAN_H


#include "DJI.h"
#include "can.h"

extern float Encoder_value;
extern int32_t Encoder_value_last;
extern int32_t Encoder_value_diff;
extern int64_t Encoder_value_sum;
extern float Encoder_speed;
extern int8_t first_flag;
extern float Encoder_value_y      ;
extern int32_t Encoder_value_last_y ;
extern int32_t Encoder_value_diff_y ;
extern int64_t Encoder_value_sum_y  ;
extern float Encoder_speed_y      ;
extern int8_t first_flag_y          ;
HAL_StatusTypeDef CANFilterInit(CAN_HandleTypeDef* hcan);


#endif
