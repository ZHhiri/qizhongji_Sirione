#ifndef __UPEERSTART_H__
#define __UPEERSTART_H__
#include "upperservo.h"
#include "retarget.h"
#include "param.h"
#include "cmsis_os.h"
#include "tim.h"
#include "mi_motor.h"
#include "wtr_can.h"

void StartDefaultTask(void const *argument)
{
    /* USER CODE BEGIN StartDefaultTask */

    /* initialize motors incluing motortypes、xyz of motors、PID of motors*/

    gantry_Motor_init();
    osDelay(30);
    init_cybergear(&mi_motor[0], 0x7F, Motion_mode);
    set_zeropos_cybergear(&mi_motor[0]);
    Set_Motor_Parameter(&mi_motor[0], 0x2013, 0.1, 'f');
    osDelay(30);
    HAL_UART_Receive_IT(&huart3, (uint8_t *)rxbuffer, 1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
    osDelay(30);
    RetargetInit(&huart2);
    osDelay(30);
    /* Infinite loop */
    for (;;) {
        osDelay(20);
    }
    /* USER CODE END StartDefaultTask */
}

#endif // !1
