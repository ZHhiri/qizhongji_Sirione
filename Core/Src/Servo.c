#include "Servo.h"

void Servo_Init( TIM_HandleTypeDef *htim, uint32_t Channel) {
    // 初始化舵机相关设置
    if (htim == NULL) {
        return; // 确保传入的定时器句柄不为空
    }
    HAL_TIM_PWM_Start(htim, Channel); // 启动PWM输出
    
    __HAL_TIM_SET_COMPARE(htim, Channel, SERVO_OFF_ANGLE); // 设置初始角度为放松状态
    // 这里SERVO_OFF_ANGLE是放松状态的角度
    // 如果需要，可以在这里添加更多的初始化代码 
}

void Servo_SetAngle(uint16_t SERVO_ANGLE) {
    //SERVO_ANGLE = (angle - 1000) * 180 / 4000 - 90; // 将角度转换为舵机的PWM值

    if (SERVO_ANGLE < SERVO_MAX_ANGLE || SERVO_ANGLE > SERVO_MIN_ANGLE) {
        return; // 角度超出范围，忽略设置
    }
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_3, SERVO_ANGLE);
}

void Servo_On(void) {
    // 舵机收紧：使爪子夹紧牛奶盒
    Servo_SetAngle(SERVO_ON_ANGLE);
}

void Servo_Off(void) {
    // 舵机放松：使爪子松开牛奶盒
    Servo_SetAngle(SERVO_OFF_ANGLE);
}
