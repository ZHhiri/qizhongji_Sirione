# 2025.0827电控部分Readme

2023313503詹智捷

## 个人贡献部分

开始联调之前，我们组电控主要的任务是让各部分电机正常运行、STM32单片机与树莓派通讯和无视觉识别情况下的运行逻辑验证，而我主要负责的部分是：

其一、调升降部分、左右部分以及旋转部分电机的PID

其二、完善和修正无视觉识别情况下的机器运行逻辑部分代码

其三、STM32单片机和树莓派通讯部分代码

其四、完成焊线、焊排针、以及走线任务

开始联调之后，我们组电控主要的任务是简化并验证各种情况下代码的运行逻辑是否有误、继续调整各部分的PID和解决准确度的问题，我在其中主要负责的部分是：

其一、继续调电机的PID，尤其是旋转部分小米电机的PID始终存在问题

其二、简化机器运行逻辑部分的代码，减少一到两个无用的流程、增加一些保护性的动作防止机器在一些特殊位置与架子干涉

其三、焊线与焊排针以及走线任务



### 树莓派接收与验证部分

要求视觉发送的内容为BXXXXXXSXXXXXX，B后面为箱子的位置、S后面为纸垛的位置。通过串口3一次接收一位数据，逐次接收。

通过B后六位数字之和恒为21，S后六位数字之和恒大于14小于21进行验证，累计接收三次即可（树莓派发送前会自己判断一次正误）

```c
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
char num[14] = "B000000S000000";
uint16_t order1 = 0;
uint16_t order2 = 0;
uint16_t rxflag = 0;


uint16_t box[6] = {0, 0, 0, 0, 0, 0};
uint16_t stack[6] = {0, 0, 0, 0, 0, 0};
#define LENGTH 100
uint16_t inner_ring_flag;

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    
     if (huart->Instance == USART3)
		{
            if (rxbuffer[0] == 'B' && rxflag == 0)
            {
                num[0] = 'B';
                order1 = 1;
                rxflag = 2;
            }
            if((rxbuffer[0] == '0' ||rxbuffer[0] == '1' || rxbuffer[0] == '2' || rxbuffer[0] == '3' || 
               rxbuffer[0] == '4' || rxbuffer[0] == '5' || rxbuffer[0] == '6' || rxbuffer[0] == 'S' ) &&  rxflag == 2&&order1 < 14)
            {
                num[order1] = rxbuffer[0];
                order1++;
            }

            if(order1 == 14)
            {
                order1 = 0;
                rxflag = 1;
            
            }
			HAL_UART_Receive_IT(&huart3,(uint8_t*)rxbuffer,1);

}
}

uint16_t marrying(uint16_t *box, uint16_t *stack, char *num)
{
    uint16_t sum_box = 0;
    uint16_t sum_stack = 0;
    if(num[0] == 'B' && num[7] == 'S')
    {
        for(order2 = 0; order2 < 6; order2++)
        {
            box[order2] = num[order2 + 1] - '0';
            stack[order2] = num[order2 + 8] - '0';
            sum_box += box[order2];
            sum_stack += stack[order2];
        }
    }
    else
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)"Error: Invalid data format!\r\n", 28, 100);
    }
    if(sum_box == 21 && sum_stack >= 15 && sum_stack <=20)
    {
        HAL_UART_Transmit(&huart2, (uint8_t*)box, 6, 100);
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);
        HAL_UART_Transmit(&huart2, (uint8_t*)stack, 6, 100);
        HAL_UART_Transmit(&huart2, (uint8_t*)"\r\n", 2, 100);

        xinagzi[0] = box[5];
        xinagzi[1] = box[2];
        xinagzi[2] = box[4];
        xinagzi[3] = box[1];
        xinagzi[4] = box[3];
        xinagzi[5] = box[0];
        zhiduo[0] = stack[0];
        zhiduo[1] = stack[1];
        zhiduo[2] = stack[2];
        zhiduo[3] = stack[3];
        zhiduo[4] = stack[4];
        zhiduo[5] = stack[5];
        
        sum_box = 0;
        sum_stack = 0;
        for(order2 = 0; order2 < 6; order2++)
        {
            box[order2] = 0;
            stack[order2] = 0;
        }
       rxflag = 0;
        return 1;
    }
    else
    {
        return 0;
    }
    
}
```



### 前后位置修正部分

通过前后拉线编码器的读值与目的值进行对比判断，减少打滑带来的准确性问题。

```C
//位置修正函数 即利用拉线编码器的读值修正位置
void y_calibration(float laxian_ref,DJI_t * motor_1,DJI_t * motor_2,float laxian_fdb,float number)
{
    uint16_t flag = 1;
    
    float diff = fabs(motor_1->posPID.ref - motor_1->posPID.fdb);
    if(fabs(laxian_ref - laxian_fdb) > 5 ) 
    {
        if(flag == 1)
        {   laxian_pid.ref=laxian_ref;
            laxian_pid.fdb=laxian_fdb;
            PosePID_Calc(&laxian_pid);
               WheelCorrect_StartTick = WheelCorrect_NowTick;
                    Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                    Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
           mygantry.gantrypos.y = mygantry.gantrypos.y - laxian_pid.output/(3.14159*85);
           
            flag = 0;
        }
        if(diff < 90)
        {
            flag = 1;
        }
    }
    else
    {   
       laxian_pid.integral=0;
        // hDJI[1].posPID.integral=0;
        // hDJI[2].posPID.integral=0;
        runflag = number;
    }
}
```



### 机器运行逻辑部分

即按照三次循环的内容进行抓取与放置，每一组上下箱子进行一次循环，循环内不同的只有抓取时和放置时的位置参数值，这些参数都在之前的接收树莓派信号的数组遍历匹配的函数中进行赋值了。

```C
    for (;;) {
        for (group = 0; group < 3;) {

            int idx1 = group * 2;
            int idx2 = group * 2 + 1;
            process_group_special(mapping[idx1], mapping[idx2], group + 1);
            if (runflag == 100) {
                float diff_z = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                float diff_y = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                float diff_x = fabs(mygantry.gantrypos.x / 0.037 -Encoder_value / 0.037);
                if (diff_x < 90 && diff_y < 90 && diff_z < 500) // diff<4°/360°*8191
                {
                    runflag = 0;
                    WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                    Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                    WheelCorrect_StartTick = WheelCorrect_NowTick;
                    Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                    Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    mygantry.gantrypos.z = z_highest;
                }
            }

            if (stateflag == 0) {
                if (runflag == 0) {

                    
                    mygantry.gantrypos.x = x_box;
         if (yaw_flag == 0) {
                        motor_controlmode(&mi_motor[0], 0, yaw_0, 0, 3, 2);
                        osDelay(1);
                    
                    if (fabs(mi_motor[0].Angle - yaw_0) < 0.001) {
                        yaw_flag = 1;
                        osDelay(1);
                    }
                        motor_controlmode(&mi_motor[0], 0, yaw_0, 0, 6, 1.9);
                        osDelay(1);
                    }
                    float diff_x = fabs(mygantry.gantrypos.x/0.037 -Encoder_value/0.037);
                    if (diff_x < 90) {
                        runflag = 1;
                    }
                }

                if (runflag == 1) {
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1100); // Open
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1100); // Open

                    osDelay(500);               // 等待Servo
                    runflag                = 2; // 进入下一个环节
                    WheelCorrect_StartTick = WheelCorrect_NowTick;
                    Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                    Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                     hDJI[1].posPID.fdb=hDJI[1].posPID.ref=(laxian_pid.fdb-842.015)/3.1415926/85;
                    hDJI[2].posPID.fdb=hDJI[2].posPID.ref=-(laxian_pid.fdb-842.015)/3.1415926/85;
                    if (group==0)
                    {
                        target_y=mygantry.gantrypos.y = y_box;
                    }
                    else if (group==1)
                    {
                        target_y=mygantry.gantrypos.y = y_box_2;
                    }
                    else if (group==2)
                    {
                        target_y=mygantry.gantrypos.y = y_box_3;
                    }
                     
                }

                if (runflag == 2) {

                    osDelay(500); // 等待Servo
                    mygantry.gantrypos.z = z_low_chushi;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    float diff_y         = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    if (diff_z < 700 && diff_y <120) {
                         y_calibration(y_box_laxian,mygantry.Motor_Y,mygantry.Motor_Y2,Encoder_value_y,3);
                        WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                        Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                        
                    }
                }

                if (runflag == 3) {
                    mygantry.gantrypos.z = z_low_crawl;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 600) {
                        runflag = 4;
                    }
                }

                if (runflag == 4) {
                    osDelay(100);                                      // 等待Servo
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 700); // Close
                    osDelay(500);
                    runflag = 5;
                     WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                    Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                    mygantry.gantrypos.z = z_low; // 抓取完毕后抬高
                }

                if (runflag == 5) {
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 400) {
                        runflag                = 6;
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                          hDJI[1].posPID.fdb=hDJI[1].posPID.ref=(laxian_pid.fdb-842.015)/3.1415926/85;
                    hDJI[2].posPID.fdb=hDJI[2].posPID.ref=-(laxian_pid.fdb-842.015)/3.1415926/85;
                        target_y=mygantry.gantrypos.y = y_stay;
                    }
                }

                if (runflag == 6) {
                    mygantry.gantrypos.z = z_highest;
                    float diff_y = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    if (diff_y < 90) {
                                osDelay(600);   
                         if (yaw_flag == 0) {
                        motor_controlmode(&mi_motor[0], 0, yaw_180, 0, 3, 1.93);
                        osDelay(1);
                    
                    if (fabs(mi_motor[0].Angle - yaw_180) < 0.001) {
                        yaw_flag = 1;
                        osDelay(1);
                    }
                        osDelay(1);
                    }
                    osDelay(400);
                        WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                        Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                         y_calibration(y_stay_laxian,mygantry.Motor_Y,mygantry.Motor_Y2,Encoder_value_y,7);
                      mygantry.gantrypos.z = z_highest;
                    }
                }

                if (runflag == 7) {
                    float diff_z = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 550) {
                        runflag                = 8;
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                          hDJI[1].posPID.fdb=hDJI[1].posPID.ref=(laxian_pid.fdb-842.015)/3.1415926/85;
                    hDJI[2].posPID.fdb=hDJI[2].posPID.ref=-(laxian_pid.fdb-842.015)/3.1415926/85;
                           if (group==0)
                    {
                        target_y=mygantry.gantrypos.y = y_box;
                    }
                    else if (group==1)
                    {
                        target_y=mygantry.gantrypos.y = y_box_2;
                    }
                    else if (group==2)
                    {
                        target_y=mygantry.gantrypos.y = y_box_3;
                    }
                    }
                }

                if (runflag == 8) {
                    
                    mygantry.gantrypos.x = x_box;
                    float diff_y         = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    float diff_x = fabs(mygantry.gantrypos.x/0.037 -Encoder_value/0.037);

                    if (diff_x < 90 && diff_y < 90) {
                         y_calibration(y_box_laxian,mygantry.Motor_Y,mygantry.Motor_Y2,Encoder_value_y,9);
                         WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                    Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                   
                    }
                }

                if (runflag == 9) {
                     mygantry.gantrypos.z = z_high_crawl;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 600) {
                        runflag = 10;
                    }
                }

                if (runflag == 10) {
                    
                    __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 700); // Close
                    osDelay(500);
                    WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                    Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                    mygantry.gantrypos.z = z_high;
                    runflag                  = 11;
                }

                if (runflag == 11) {

                    
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 550) {
                        runflag                  = 12;
                        WheelCorrect_StartTick   = WheelCorrect_NowTick;
                        Wheel_StartPos[1]        = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]        = hDJI[2].posPID.fdb;
                        WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                        Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                        hDJI[1].posPID.fdb=hDJI[1].posPID.ref=(laxian_pid.fdb-842.015)/3.1415926/85;
                        hDJI[2].posPID.fdb=hDJI[2].posPID.ref=-(laxian_pid.fdb-842.015)/3.1415926/85;
                        target_y=mygantry.gantrypos.y = y_stack;//
                        mygantry.gantrypos.z = z_high;
                         
                    }
                    osDelay(600);
                }
                //
                if (runflag == 12) {
                    
                    
                   osDelay(300);
                    mygantry.gantrypos.x = x_stack;
                      if (yaw_rotate==yaw_270||yaw_rotate==yaw_90)
                        {
                             if (yaw_flag == 0) {
                        motor_controlmode(&mi_motor[0], 0, yaw_rotate, 0, 3, 2.2);
                        osDelay(1); 
                                        
                            /* code */
                        }
                    }
                        else if (yaw_rotate==yaw_180||yaw_rotate==yaw_0)
                        {
                            motor_controlmode(&mi_motor[0], 0, yaw_rotate, 0, 3, 2.5);
                        }

                    float diff_y = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    float diff_x = fabs(mygantry.gantrypos.x/0.037 -Encoder_value/0.037);
                    float diff_z = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_x < 90 && diff_y < 90 && diff_z < 600) {
                        y_calibration(y_stack_laxian,mygantry.Motor_Y,mygantry.Motor_Y2,Encoder_value_y,14); 
                        WheelCorrect_StartTick   = WheelCorrect_NowTick;
                        Wheel_StartPos[1]        = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]        = hDJI[2].posPID.fdb;
                        WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                        Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                       
                    }
                }


                if (runflag == 14) {
                     mygantry.gantrypos.z = z_place;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    if (diff_z < 500) {
                        osDelay(300);
                        if (mapping[idx1] == 0) {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1300); // Open
                            /* code */
                        } else {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1300); // Open
                        }
                        osDelay(800);

                        runflag                  = 15;
                        WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                        Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                        mygantry.gantrypos.z = z_high;
                        osDelay(500);
                        WheelCorrect_StartTick   = WheelCorrect_NowTick;
                        Wheel_StartPos[1]        = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]        = hDJI[2].posPID.fdb;
                        
                        hDJI[1].posPID.fdb=hDJI[1].posPID.ref=(laxian_pid.fdb-842.015)/3.1415926/85;
                        hDJI[2].posPID.fdb=hDJI[2].posPID.ref=-(laxian_pid.fdb-842.015)/3.1415926/85;
                        
                        target_y=mygantry.gantrypos.y = -1;
                    }
                }

                if (runflag == 15) {

                     if (yaw_rotate==yaw_0||yaw_rotate==yaw_180||yaw_rotate==yaw_now||yaw_rotate==yaw2_now) 
                    {
                       
                    } 
                    else{
                     if (yaw_flag == 0) {
                        motor_controlmode(&mi_motor[0], 0, yaw_0, 0, 3, 2);
                        osDelay(1);
                    
                    if (fabs(mi_motor[0].Angle - yaw_0) < 0.001) {
                        yaw_flag = 1;
                        osDelay(1);
                    }
                        motor_controlmode(&mi_motor[0], 0, yaw_0, 0, 6, 1.9);
                        osDelay(1);
                    }
                    }
                    mygantry.gantrypos.x = x2_stack;
                    float diff_x = fabs(mygantry.gantrypos.x/0.037 -Encoder_value/0.037);
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);
                    float diff_y         = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);
                    if (diff_z < 550&&diff_x < 90&&diff_y < 90) {
                        if (yaw2_rotate==yaw_270||yaw2_rotate==yaw_90)
                        {
                             if (yaw_flag == 0) {
                        motor_controlmode(&mi_motor[0], 0, yaw2_rotate, 0, 3, 1.8);
                        osDelay(1); 

                        }
                    }
                        else if (yaw2_rotate==yaw_180||yaw2_rotate==yaw_0)
                        {
                            motor_controlmode(&mi_motor[0], 0, yaw2_rotate, 0, 3, 1.93);
                        }
                        
                             
                    if ((mapping[idx1] == 0&&mapping[idx2] == 2)|| (mapping[idx1] == 2 && mapping[idx2] == 0)||(mapping[idx1] == 5 && mapping[idx2] == 0)||(mapping[idx1] == 0 && mapping[idx2] == 5)) 
                    {
                        osDelay(1000);
                    } 
                    else if(((mapping[idx1] == 0 && mapping[idx2] == 1))||(mapping[idx1] == 1 && mapping[idx2] == 0)||(mapping[idx1] == 6 && mapping[idx2] == 0)||(mapping[idx1] == 0 && mapping[idx2] == 6))
                    {
                        osDelay(400);
                    }
                        osDelay(200);
                        runflag                = 16;
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                        hDJI[1].posPID.fdb=hDJI[1].posPID.ref=(laxian_pid.fdb-842.015)/3.1415926/85;
                        hDJI[2].posPID.fdb=hDJI[2].posPID.ref=-(laxian_pid.fdb-842.015)/3.1415926/85;
                        target_y=mygantry.gantrypos.y = y2_stack;//-0.2*group
                    }
                }

                if (runflag == 16) {    
                    float diff_y         = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);

                    if ( diff_y < 90) {
                         y_calibration(y_stack_2_laxian,mygantry.Motor_Y,mygantry.Motor_Y2,Encoder_value_y,17);
                        //runflag                  = 17;
                        WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                        Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                        
                    }
                }

                if (runflag == 17) {

                    mygantry.gantrypos.z = z2_place;
                    float diff_z         = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);

                    if (diff_z < 500) {
                        runflag = 18;
                        if (mapping[idx1] == 0) {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1100); // Open
                            /* code */
                        } else {
                            __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1100); // Open
                        }
                        osDelay(800);
                        WheelCorrect_StartTick_2 = WheelCorrect_NowTick_2;
                        Wheel_StartPos[4]        = hDJI[4].posPID.fdb;
                        mygantry.gantrypos.z = z_highest;
                        osDelay(500);
                         WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                          hDJI[1].posPID.fdb=hDJI[1].posPID.ref=(laxian_pid.fdb-842.015)/3.1415926/85;
                    hDJI[2].posPID.fdb=hDJI[2].posPID.ref=-(laxian_pid.fdb-842.015)/3.1415926/85;
                        target_y=mygantry.gantrypos.y = 0.0;
                    }
                }

                if (runflag == 18) {
                    
                    osDelay(400);
                    float diff_z = fabs(mygantry.gantrypos.z * 8191 - hDJI[4].AxisData.AxisAngle_inDegree);

                    if (diff_z < 550) {
                        runflag = 19;
                        runflag = 100;
                        

                        group++;
                        WheelCorrect_StartTick = WheelCorrect_NowTick;
                        Wheel_StartPos[1]      = hDJI[1].posPID.fdb;
                        Wheel_StartPos[2]      = hDJI[2].posPID.fdb;
                    }
                }
                  hDJI[1].posPID.fdb=hDJI[1].posPID.ref=(laxian_pid.fdb-842.015)/3.1415926/85;
                    hDJI[2].posPID.fdb=hDJI[2].posPID.ref=-(laxian_pid.fdb-842.015)/3.1415926/85;
                if (group == 3) {
                    target_y=mygantry.gantrypos.y = y_box;
                    // mygantry.gantrypos.x = x_box_1;
                    mygantry.gantrypos.x = x_box_2;
                    /* code */
                    osDelay(500);
                    if (yaw_flag == 0) {
                        motor_controlmode(&mi_motor[0], 0, yaw_90, 0, 3, 2);
                        osDelay(1);
                    
                    if (fabs(mi_motor[0].Angle - yaw_90) < 0.005) {
                        yaw_flag = 1;
                        osDelay(1);
                    }
                        motor_controlmode(&mi_motor[0], 0, yaw_90, 0, 10, 8);
                        osDelay(1);
                    }
                    float diff_x = fabs(mygantry.gantrypos.x/0.037 -Encoder_value/0.037);
                    float diff_y = fabs(mygantry.gantrypos.y * 8191 - hDJI[1].AxisData.AxisAngle_inDegree);

                    if (diff_x < 90 && diff_y < 90) {
                        runflag = 20;
                    }
                }
            }
        }
        osDelay(20);
    }
```



## 简述代码结构

按照代码运行流程进行解释：

首先开启串口3的接收中断，直到判断接收到的数据正确后对Freertos进行初始化（即开启freertos）并关闭接收中断；

freertos存在三个进程，进程一StartDefaultTask是对各个函数进行初始化、进程二upperservotask是以1ms的频率对速度规划、PID等进行反复计算、进程三uppergoingtask是通过置标志位推进机器的运行步骤。

其中，由于我们的运行逻辑是取的顺序不变，按照视觉识别情况对放置的顺序进行改变，所以uppergoingtask中含有一个函数void process_group_special(int16_t mapX, int16_t mapY, int group_id)，是对视觉识别传入的数组进行遍历，找到符合传入数据的那个情况，并按照该情况将各个参数赋值，赋值完成后按序推进机器运行步骤。

其次，这个标志位的赋值是在误差判断中的，即每一个运行步骤中判断执行下一步的依据是上一步的位置是否处于某个小误差范围内，如果符合则执行下一步，如果不符合则继续调整位置，这里就出现了一个问题，后续我们采用其他办法进行了解决，暂且按下不表。

然后，为使得我们每一步的移动打滑更小，我们插入了梯形速度规划算法，这就要求我们需要在合适的地方对规划的初始时间和初始位置进行重设，因为旋转部分的代码相当于一个堵塞，我们需要将旋转部分的代码、重设规划的初始时间和初始位置的代码的位置进行调整。

最后，对于can总线接收发送的部分，由于我们采用了两个can总线，其中can1是挂载了4个大疆电机、2个拉线编码器，can2挂载了1个小米电机，can1全部采用标准帧格式收发，由于拉线编码器只需要接收因此只额外写了接收的函数，can2采用拓展帧格式收发。

![简述代码流程图](https://github.com/ZHhiri/qizhongji_Sirione/blob/main/简述代码流程图.jpg)



## 遇到的问题与解决

遇到的最大问题除去最后一天前晚上的分线板出现故障以外，就是机器运行逻辑中一些步骤会干涉货架以及角落的摄像头杆、如何保证准确性等等问题。

经过优化机器运行逻辑之后，最大的问题就是前后移动的准确性问题。

由于我们的机器采用的是轮胎在轨道上面滚动的方式驱动前后，难免出现打滑导致位置不准确的情况出现，又由于机械机构的限制无法装上激光测距模块，于是只能考虑采用编码器的形式，害怕打滑的情况重新出现于是放弃了常规的旋转编码器，选择了拉线编码器。拉线编码器的弊端就在于存在限速，但是如果直接用拉线编码器作为前后电机的反馈可能会导致前后移动超速，或经过调整PID后出现前后移动较为缓慢拖延时间的问题。


于是我决定采用修正的方式，即先读取拉线编码器在各个特殊点的值并记录下来后，让前后电机正常跑，通过读取大疆电机编码器的返回值确认是否到位，确认到位后通过拉线编码器的读值与目标特殊点的记录值进行对比再进行移动，移动的方式即采用固定输出一个较小的前进位置并循环直到到位为止，这样既可不过分调整前后电机的PID，又可以较准确快速地完成到达目的地的任务。

