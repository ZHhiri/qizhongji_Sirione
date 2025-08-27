/*过滤器配置，can的fifo0接收*/

#include "wtr_can.h"
#include "mi_motor.h"

uint8_t CanReceiveData[8];
CAN_RxHeaderTypeDef rxMsg; // 发送接收结构体
uint8_t rx_data[8];        // 接收数据
uint32_t Motor_Can_ID;     // 接收数据电机ID
float Encoder_value          = 0;
int32_t Encoder_value_last   = 0;
int32_t Encoder_value_diff   = 0;
int64_t Encoder_value_sum    = 0;
float Encoder_speed          = 0.0;
int8_t first_flag            = 0;
float Encoder_value_y        = 0;
int32_t Encoder_value_last_y = 0;
int32_t Encoder_value_diff_y = 0;
int64_t Encoder_value_sum_y  = 0;
float Encoder_speed_y        = 0.0;
int8_t first_flag_y          = 0;
HAL_StatusTypeDef CANFilterInit(CAN_HandleTypeDef *hcan)
{
    /*新增内容：启用时钟*/
    CAN_FilterTypeDef sFilterConfig;

    sFilterConfig.FilterBank           = 0;
    sFilterConfig.FilterMode           = CAN_FILTERMODE_IDMASK;
    sFilterConfig.FilterScale          = CAN_FILTERSCALE_32BIT;
    sFilterConfig.FilterIdHigh         = 0x0000;
    sFilterConfig.FilterIdLow          = 0x0000;
    sFilterConfig.FilterMaskIdHigh     = 0x0000;
    sFilterConfig.FilterMaskIdLow      = 0x0000;
    sFilterConfig.FilterFIFOAssignment = CAN_RX_FIFO0;
    sFilterConfig.FilterActivation     = ENABLE;
    if (hcan->Instance == CAN1) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        sFilterConfig.FilterBank           = 0;
        sFilterConfig.SlaveStartFilterBank = 0;
    } else if (hcan->Instance == CAN2) {
        __HAL_RCC_CAN1_CLK_ENABLE();
        __HAL_RCC_CAN2_CLK_ENABLE();
        sFilterConfig.FilterBank           = 14;
        sFilterConfig.SlaveStartFilterBank = 14;
    }

    /*战队祖传内容*/

    if (HAL_CAN_ConfigFilter(hcan, &sFilterConfig) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_Start(hcan) != HAL_OK) {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK) {
        Error_Handler();
    }

    return HAL_OK;
}

void CanDataDecode(CAN_RxHeaderTypeDef RxHeader)
{
    /* Can message Decode */
    if (RxHeader.IDE == CAN_ID_STD) {
        DJI_CanMsgDecode(RxHeader.StdId, CanReceiveData);
        if (RxHeader.StdId == 0x08) {
            if (RxHeader.DLC == 0x07 && CanReceiveData[2] == 0x01) {
                // 原有编码器数据处理逻辑
                Encoder_value = CanReceiveData[3] | (CanReceiveData[4] << 8) |
                                (CanReceiveData[5] << 16) | (CanReceiveData[6] << 24);

                Encoder_value *= 0.037;
                Encoder_value_last        = 0;
                Encoder_value_diff        = (Encoder_value - Encoder_value_last);
                int64_t Encoder_value_sum = 0;
                Encoder_speed             = Encoder_value_diff / 0.001f;
                first_flag                = 0;

                if (first_flag != 0) {
                    if (Encoder_value_diff < -12000) {
                        Encoder_value_sum += Encoder_value_diff + 24576;
                    } else if (Encoder_value_diff > 12000) {
                        Encoder_value_sum += Encoder_value_diff - 24576;
                    } else {
                        Encoder_value_sum += Encoder_value_diff;
                    }
                }
                Encoder_value_last = Encoder_value;
                first_flag         = 1;
            } else {
            }
        }
        if (RxHeader.StdId == 0x07) {
            if (RxHeader.DLC == 0x07 && CanReceiveData[2] == 0x01) {
                // 原有编码器数据处理逻辑
                Encoder_value_y = CanReceiveData[3] | (CanReceiveData[4] << 8) |
                                  (CanReceiveData[5] << 16) | (CanReceiveData[6] << 24);

                Encoder_value_y *= 0.049;
                Encoder_value_last_y        = 0;
                Encoder_value_diff_y        = (Encoder_value_y - Encoder_value_last_y);
                int64_t Encoder_value_sum_y = 0;
                Encoder_speed_y             = Encoder_value_diff_y / 0.001f;
                first_flag_y                = 0;

                if (first_flag_y != 0) {
                    if (Encoder_value_diff_y < -12000) {
                        Encoder_value_sum_y += Encoder_value_diff_y + 24576;
                    } else if (Encoder_value_diff_y > 12000) {
                        Encoder_value_sum_y += Encoder_value_diff_y - 24576;
                    } else {
                        Encoder_value_sum_y += Encoder_value_diff_y;
                    }
                }
                Encoder_value_last_y = Encoder_value_y;
                first_flag_y         = 1;
            } else {
            }
        }
    }
    if (RxHeader.IDE == CAN_ID_EXT) {
        // vesc反馈关掉这里就不会有消息
        ;
        ;
    }
}
void CanDataDecode2(CAN_RxHeaderTypeDef RxHeader)
{
    /* Can message Decode */
    if (RxHeader.IDE == CAN_ID_STD) {
        DJI_CanMsgDecode2(RxHeader.StdId, CanReceiveData);
    }
    if (RxHeader.IDE == CAN_ID_EXT) {
        // vesc反馈关掉这里就不会有消息
        ;
        ;
    }
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef RxHeader;
    if (hcan->Instance == hcan1.Instance) {
        if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader, CanReceiveData) != HAL_OK) {
            Error_Handler();
        }
        CanDataDecode(RxHeader);
    }
    if (hcan->Instance == hcan2.Instance) {
        HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rxMsg, rx_data); // 接收数据
        Motor_Can_ID = Get_Motor_ID(rxMsg.ExtId);                  // 首先获取回传电机ID信息
        switch (Motor_Can_ID)                                      // 将对应ID电机信息提取至对应结构体
        {
            case 0X7F:
                if (rxMsg.ExtId >> 24 != 0) // 检查是否为广播模式
                    Motor_Data_Handler(&mi_motor[0], rx_data, rxMsg.ExtId);
                else
                    mi_motor[0].MCU_ID = rx_data[0];
                break;
            default:
                break;
        }
    }
}
