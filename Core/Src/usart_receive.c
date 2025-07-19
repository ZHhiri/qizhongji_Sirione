/*USART中断回调函数处理文件*/

#include "usart.h"
#include "stm32f4xx_it.h"
#include "decode.h"

uint8_t Rxbuffer_3[195];

LidarPointTypedef Lidar1;

uint16_t UartFlag[6];
uint8_t usart3_rx[1];
#define LENGTH 100
uint16_t inner_ring_flag;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART3) {
        static uint16_t u1state = 0; //状态机计数
        static uint16_t crc1    = 0; //校验和
        uint8_t tmp1   = usart3_rx[0];
        if (u1state < 4)
        {
            if (tmp1 == 0xAA)
            {
                Rxbuffer_3[u1state] = tmp1;
                u1state++;
                
            } else {
                u1state = 0;
            }
        } else if (u1state < 194) {
            Rxbuffer_3[u1state] = tmp1;
            u1state++;
            crc1 += tmp1;
        }else if(u1state==194){
            Rxbuffer_3[u1state] = tmp1;
            if (tmp1 == crc1 % 256) 
            {
                UartFlag[0] = 1;
            }
            u1state = 0;
            crc1    = 0;
        } else {
        };
				
				//STP_23L_Decode(Rxbuffer_3,&Lidar1);
				
        HAL_UART_Receive_IT(&huart3, usart3_rx, 1);
    }
    else if(huart->Instance == USART2)
	{
		//HAL_UART_Receive_DMA(&huart2,(uint8_t*)goal_angle,LENGTH);
	}
}