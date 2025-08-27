/*USART中断回调函数处理文件*/

#include "usart.h"
#include "stm32f4xx_it.h"
#include "decode.h"
#include "param.h"
uint8_t Rxbuffer_3[195];

LidarPointTypedef Lidar1;

uint8_t number[6];
uint8_t trust = 0;
uint16_t UartFlag[6];
uint8_t usart3_rx[1];
uint8_t rxbuffer[1];
char num[14]    = "B000000S000000";
uint16_t order1 = 0;
uint16_t order2 = 0;
uint16_t rxflag = 0;

uint16_t box[6]   = {0, 0, 0, 0, 0, 0};
uint16_t stack[6] = {0, 0, 0, 0, 0, 0};
#define LENGTH 100
uint16_t inner_ring_flag;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    if (huart->Instance == USART3) {
        if (rxbuffer[0] == 'B' && rxflag == 0) {
            num[0] = 'B';
            order1 = 1;
            rxflag = 2;
        }
        if ((rxbuffer[0] == '0' || rxbuffer[0] == '1' || rxbuffer[0] == '2' || rxbuffer[0] == '3' ||
             rxbuffer[0] == '4' || rxbuffer[0] == '5' || rxbuffer[0] == '6' || rxbuffer[0] == 'S') &&
            rxflag == 2 && order1 < 14) {
            num[order1] = rxbuffer[0];
            order1++;
        }

        if (order1 == 14) {
            order1 = 0;
            rxflag = 1;
        }
        HAL_UART_Receive_IT(&huart3, (uint8_t *)rxbuffer, 1);
    }
}

uint16_t marrying(uint16_t *box, uint16_t *stack, char *num)
{
    uint16_t sum_box   = 0;
    uint16_t sum_stack = 0;
    if (num[0] == 'B' && num[7] == 'S') {
        for (order2 = 0; order2 < 6; order2++) {
            box[order2]   = num[order2 + 1] - '0';
            stack[order2] = num[order2 + 8] - '0';
            sum_box += box[order2];
            sum_stack += stack[order2];
        }
    } else {
        // HAL_UART_Transmit(&huart2, (uint8_t*)"Error: Invalid data format!\r\n", 28, 100);
    }
    if (sum_box == 21 && sum_stack >= 15 && sum_stack <= 20) {
        HAL_UART_Transmit(&huart2, (uint8_t *)box, 6, 100);
        // HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
        HAL_UART_Transmit(&huart2, (uint8_t *)stack, 6, 100);
        // HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);

        xinagzi[0] = box[5];
        xinagzi[1] = box[2];
        xinagzi[2] = box[4];
        xinagzi[3] = box[1];
        xinagzi[4] = box[3];
        xinagzi[5] = box[0];
        zhiduo[0]  = stack[0];
        zhiduo[1]  = stack[1];
        zhiduo[2]  = stack[2];
        zhiduo[3]  = stack[3];
        zhiduo[4]  = stack[4];
        zhiduo[5]  = stack[5];

        sum_box   = 0;
        sum_stack = 0;
        for (order2 = 0; order2 < 6; order2++) {
            box[order2]   = 0;
            stack[order2] = 0;
        }
        rxflag = 0;
        return 1;
    } else {
        return 0;
    }
}