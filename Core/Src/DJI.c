/*包含DJI的初始化，DJI的Transmit，DJI的解码算法*/

#include "DJI.h"
#include "Caculate.h"
#include "PID.h"
DJI_t hDJI[8];
PIDController miPIDController = {0.6f, 0.6f, 0.013f, 0.0f, 0.0f};
// 使用DJI init前需要指定motorType
void DJI_Init()
{

    // hDJI.0 = 云台电机
    hDJI[0].speedPID.KP        = 0.3;
    hDJI[0].speedPID.KI        = 0.000025;
    hDJI[0].speedPID.KD        = 0.001;
    hDJI[0].speedPID.outputMax = 1200;

    hDJI[0].posPID.KP        = 0.3f;
    hDJI[0].posPID.KI        = 0.003f;
    hDJI[0].posPID.KD        = 0.003f;
    hDJI[0].posPID.outputMax = 3000;

    // hDJI.1 = 前后电机；hDJI.2 = 前后电机
    hDJI[1].speedPID.KP          = 15;
    hDJI[1].speedPID.KI          = 0.00015;
    hDJI[1].speedPID.KD          = 0.0;
    hDJI[1].speedPID.outputMax   = 16000;
    hDJI[1].speedPID.integralMax = 100000;
    hDJI[1].speedPID.integralMin = -100000;

    hDJI[1].posPID.KP          = 3.0f;
    hDJI[1].posPID.KI          = 0.0001f;
    hDJI[1].posPID.KD          = 0.0f;
    hDJI[1].posPID.outputMax   = 130000;
    hDJI[1].posPID.integralMax = 100000;
    hDJI[1].posPID.integralMin = -100000;

    // hDJI.1 = 前后电机；hDJI.2 = 前后电机
    hDJI[2].speedPID.KP          = 15;
    hDJI[2].speedPID.KI          = 0.00015;
    hDJI[2].speedPID.KD          = 0.0;
    hDJI[2].speedPID.outputMax   = 16000;
    hDJI[2].speedPID.integralMax = 100000;
    hDJI[2].speedPID.integralMin = -100000;

    hDJI[2].posPID.KP          = 3.0f;
    hDJI[2].posPID.KI          = 0.0001f;
    hDJI[2].posPID.KD          = 0.0f;
    hDJI[2].posPID.outputMax   = 130000;//140000
    hDJI[2].posPID.integralMax = 100000;
    hDJI[2].posPID.integralMin = -100000;

    // hDJI.3 = 左右电机
    hDJI[3].speedPID.KP          = 5;
    hDJI[3].speedPID.KI          = 2;
    hDJI[3].speedPID.KD          = 0.001;
    hDJI[3].speedPID.integralMax = 100000;
    hDJI[3].speedPID.outputMax   = 16000;

    hDJI[3].posPID.KP          = 5.0f;
    hDJI[3].posPID.KI          =1.0f;
    hDJI[3].posPID.KD          = 0.00006f;
    hDJI[3].posPID.integralMax = 100000;
    hDJI[3].posPID.outputMax   = 5300;

    // hDJI.4 = 升降电机
    hDJI[4].speedPID.KP        = 5;
    hDJI[4].speedPID.KI        = 4; // 4
    hDJI[4].speedPID.KD        = 0.00001;
    hDJI[4].speedPID.outputMax = 16000;

    hDJI[4].posPID.KP        = 1.7f; // 1.7
    hDJI[4].posPID.KI        = 1.2f; // 1
    hDJI[4].posPID.KD        = 0.0005f;
    hDJI[4].posPID.outputMax = 4600;

    laxian_pid.KP        = 0.6f;//2.4/4.3
    laxian_pid.KI        = 0.6f;
    laxian_pid.KD        = 0.013f;
    laxian_pid.outputMax = 500;
    laxian_pid.integralMax = 100;//500
    laxian_pid.integralMin = -100;

    miPIDController.kp = 0.0f;//0.002
    miPIDController.ki = 0.0f;//0.0001
    miPIDController.kd = 0.0f;//0.013
    miPIDController.integral = 2.0f;
    for (int i = 0; i < 8; i++) {
        /*        hDJI[i].speedPID.KP = 12;
                hDJI[i].speedPID.KI = 0.2;
                hDJI[i].speedPID.KD = 5;l
                hDJI[i].speedPID.outputMax = 8000;

                hDJI[i].posPID.KP = 80.0f;
                hDJI[i].posPID.KI = 1.0f;
                hDJI[i].posPID.KD =0.0f;
                hDJI[i].posPID.outputMax = 5000;
        //			  hDJI[i].posPID.outputMin = 1500;
        */

        if (hDJI[i].motorType == M3508) {
            hDJI[i].reductionRate = 3591.0f / 187.0f; // 2006减速比为36 3508减速比约为19
        } else if (hDJI[i].motorType == M2006) {
            hDJI[i].reductionRate = 36.0f;
        }

        hDJI[i].encoder_resolution = 8192.0f;
    }
}

static uint32_t TxMailbox;

HAL_StatusTypeDef CanTransmit_SetBaudRate(CAN_HandleTypeDef *hcanx, uint8_t sensorId, uint8_t baudRate) {
    CAN_TxHeaderTypeDef TxHeader;
    uint8_t TxData[8];
    uint32_t TxMailbox;

    // 配置CAN头部
    TxHeader.StdId = sensorId;
    TxHeader.IDE = CAN_ID_STD;
    TxHeader.RTR = CAN_RTR_DATA;
    TxHeader.DLC = 0x04;
    TxHeader.TransmitGlobalTime = DISABLE;

    // 填充数据域
    TxData[0] = 0x04;
    TxData[1] = sensorId;
    TxData[2] = 0x04;  // 指令码：设置波特率
    TxData[3] = baudRate;
	TxData[4] = 0x03;

    // 发送数据
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
    return HAL_CAN_AddTxMessage(hcanx, &TxHeader, TxData, &TxMailbox);
}
void CanTransmit_DJI_1234(CAN_HandleTypeDef *hcanx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq)
{
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.DLC   = 0x08;
    TxMessage.StdId = 0x200;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;

    uint8_t TxData[8];
    TxData[0] = (uint8_t)(cm1_iq >> 8);
    TxData[1] = (uint8_t)cm1_iq;
    TxData[2] = (uint8_t)(cm2_iq >> 8);
    TxData[3] = (uint8_t)cm2_iq;
    TxData[4] = (uint8_t)(cm3_iq >> 8);
    TxData[5] = (uint8_t)cm3_iq;
    TxData[6] = (uint8_t)(cm4_iq >> 8);
    TxData[7] = (uint8_t)cm4_iq;
    while (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
    if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, TxData, &TxMailbox) != HAL_OK) {
        Error_Handler(); // 如果CAN信息发送失败则进入死循环
    }
}

void CanTransmit_DJI_5678(CAN_HandleTypeDef *hcanx, int16_t cm5_iq, int16_t cm6_iq, int16_t cm7_iq, int16_t cm8_iq)
{
    CAN_TxHeaderTypeDef TxMessage;

    TxMessage.DLC   = 0x08;
    TxMessage.StdId = 0x1FF;
    TxMessage.IDE   = CAN_ID_STD;
    TxMessage.RTR   = CAN_RTR_DATA;

    uint8_t TxData[8];
    TxData[0] = (uint8_t)(cm5_iq >> 8);
    TxData[1] = (uint8_t)cm5_iq;
    TxData[2] = (uint8_t)(cm6_iq >> 8);
    TxData[3] = (uint8_t)cm6_iq;
    TxData[4] = (uint8_t)(cm7_iq >> 8);
    TxData[5] = (uint8_t)cm7_iq;
    TxData[6] = (uint8_t)(cm8_iq >> 8);
    TxData[7] = (uint8_t)cm8_iq;

    while (HAL_CAN_GetTxMailboxesFreeLevel(hcanx) == 0);
    if (HAL_CAN_AddTxMessage(hcanx, &TxMessage, TxData, &TxMailbox) != HAL_OK) {
        Error_Handler(); // 如果CAN信息发送失败则进入死循环
    }
}

void DJI_Update(DJI_t *motor, uint8_t *fdbData)
{
    /*  反馈信息计算  */
    motor->FdbData.RotorAngle_0_360 = (fdbData[0] << 8 | fdbData[1]) * 360.0f / motor->encoder_resolution; /* unit:degree*/
    motor->FdbData.rpm              = (int16_t)(fdbData[2] << 8 | fdbData[3]);                             /* unit:rom   */
    motor->FdbData.current          = (int16_t)(fdbData[4] << 8 | fdbData[5]);
    /*  计算数据处理  */
    /*  更新反馈速度/位置  */
    motor->Calculate.RotorAngle_0_360_Log[LAST] = motor->Calculate.RotorAngle_0_360_Log[NOW];
    motor->Calculate.RotorAngle_0_360_Log[NOW]  = motor->FdbData.RotorAngle_0_360;
    /* 电机圈数更新        */
    if (motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_Log[LAST] > (180.0f))
        motor->Calculate.RotorRound--;
    else if (motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_Log[LAST] < -(180.0))
        motor->Calculate.RotorRound++;
    /* 电机输出轴角度      */
    motor->AxisData.AxisAngle_inDegree = motor->Calculate.RotorRound * 360.0f;
    motor->AxisData.AxisAngle_inDegree += motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_OffSet;
    motor->AxisData.AxisAngle_inDegree /= motor->reductionRate;
    motor->AxisData.AxisAngle_inDegree = motor->AxisData.AxisAngle_inDegree / 360 * 8191;

    motor->AxisData.AxisVelocity    = motor->FdbData.rpm / motor->reductionRate;
    motor->Calculate.RotorAngle_all = motor->Calculate.RotorRound * 360 + motor->Calculate.RotorAngle_0_360_Log[NOW] - motor->Calculate.RotorAngle_0_360_OffSet;
}

void get_dji_offset(DJI_t *motor, uint8_t *fdbData)
{
    motor->FdbData.RotorAngle_0_360             = (fdbData[0] << 8 | fdbData[1]) * 360.0f / motor->encoder_resolution;
    motor->Calculate.RotorAngle_0_360_Log[LAST] = motor->FdbData.RotorAngle_0_360;
    motor->Calculate.RotorAngle_0_360_Log[NOW]  = motor->Calculate.RotorAngle_0_360_Log[LAST];

    motor->Calculate.RotorAngle_0_360_OffSet = motor->FdbData.RotorAngle_0_360;
}

HAL_StatusTypeDef DJI_CanMsgDecode(uint32_t Stdid, uint8_t *fdbData)
{
    int i = Stdid - 0x201;
    
    if (i >= 0 && i < 8) {
        if (hDJI[i].FdbData.msg_cnt < 50) {
            get_dji_offset(&hDJI[i], fdbData);
            hDJI[i].FdbData.msg_cnt++;
        } else {
            DJI_Update(&hDJI[i], fdbData);
        }
        return HAL_OK;
    }

    return HAL_ERROR;
}
