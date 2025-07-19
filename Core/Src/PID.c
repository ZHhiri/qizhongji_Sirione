#include "PID.h"
#include "math.h"

/*比例算法*/
//比例算法
void P_Calc(PID_t *pid){
	pid->cur_error = pid->ref - pid->fdb;
	pid->output = pid->KP * pid->cur_error;
	/*设定输出上限*/
	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
	
	if(fabs(pid->output)<pid->outputMin)
		pid->output=0;

}


/*增量式PID算法*/
void IncrPID_Calc(PID_t *pid)
{
    pid->cur_error = pid->ref - pid->fdb;
    pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->cur_error;

    /*设定输出上限*/
    if (pid->output > pid->outputMax )
    pid->output = pid->outputMax;
    if(pid->output < -pid->outputMax)
    pid->output = -pid->outputMax;

    /*if (pid->output < pid->outputMin)
    pid->output = pid->outputMin;
    if (pid->output > -pid->outputMin)
    pid->output = -pid->outputMin;*/

    if(pid->cur_error < 200 && pid->cur_error > -200)
        pid->output = pid->KP * (pid->cur_error - pid->error[1]); // 防抖处理
}

/*位置式PID算法*/
void PosePID_Calc(PID_t *pid)
{
    pid->cur_error = pid->ref - pid->fdb;
    pid->integral += pid->cur_error;

    /*防止积分饱和*/
    if (pid->integral > pid->integralMax)
        pid->integral = pid->integralMax;
    if (pid->integral < pid->integralMin)
        pid->integral = pid->integralMin;

    pid->output = pid->KP * pid->cur_error + pid->KI * pid->integral + pid->KD * (pid->error[1] - pid->error[0]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->cur_error;

    /*设定输出上限*/
  
    if (pid->output > pid->outputMax )
    pid->output = pid->outputMax;
    if(pid->output < -pid->outputMax)
    pid->output = -pid->outputMax;
    if(pid->cur_error < 70 && pid->cur_error > -70)
        pid->output = 0; // 防抖处理

}

/*PD算法*/
void PD_Calc(PID_t *pid)
{
    pid->cur_error = pid->ref - pid->fdb;
    pid->output = pid->KP * pid->cur_error + pid->KD * (pid->error[1] - pid->error[0]);
    pid->error[0] = pid->error[1];
    pid->error[1] = pid->cur_error;

    /*设定输出上限*/
    if (pid->output > pid->outputMax)
        pid->output = pid->outputMax;
    if (pid->output < pid->outputMin)
        pid->output = pid->outputMin;
}

/**
 * @brief T型速度规划函数
 * @param initialAngle 初始角度
 * @param maxAngularVelocity 最大角速度
 * @param AngularAcceleration 角加速度
 * @param targetAngle 目标角度
 * @param currentTime 当前时间
 * @param currentTime 当前角度
 * @todo 转换为国际单位制
 */
// void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle)
// {

//     float angleDifference = targetAngle - initialAngle;     // 计算到目标位置的角度差
//     float sign            = (angleDifference > 0) ? 1 : -1; // 判断角度差的正负(方向)

//     float accelerationTime = maxAngularVelocity / AngularAcceleration;                                                      // 加速(减速)总时间
//     float constTime        = (fabs(angleDifference) - AngularAcceleration * pow(accelerationTime, 2)) / maxAngularVelocity; // 匀速总时间
//     float totalTime        = constTime + accelerationTime * 2;                                                              // 计算到达目标位置所需的总时间

//     // 判断能否达到最大速度
//     if (constTime > 0) {
//         // 根据当前时间判断处于哪个阶段
//         if (currentTime <= accelerationTime) {
//             // 加速阶段
//             *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
//         } else if (currentTime <= accelerationTime + constTime) {
//             // 匀速阶段
//             *currentAngle = initialAngle + sign * maxAngularVelocity * (currentTime - accelerationTime) + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2);
//         } else if (currentTime <= totalTime) {
//             // 减速阶段
//             float decelerationTime = currentTime - accelerationTime - constTime;
//             *currentAngle          = initialAngle + sign * maxAngularVelocity * constTime + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
//         } else {
//             // 达到目标位置
//             *currentAngle = targetAngle;
//         }
//     } else {
//         maxAngularVelocity = sqrt(fabs(angleDifference) * AngularAcceleration);
//         accelerationTime   = maxAngularVelocity / AngularAcceleration;
//         totalTime          = 2 * accelerationTime;
//         constTime          = 0;
//         // 根据当前时间判断处于哪个阶段
//         if (currentTime <= accelerationTime) {
//             // 加速阶段
//             *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
//         } else if (currentTime <= totalTime) {
//             // 减速阶段
//             float decelerationTime = currentTime - accelerationTime; // 减速时间
//             *currentAngle          = initialAngle + sign * 0.5 * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
//         } else {
//             // 达到目标位置
//             *currentAngle = targetAngle;
//         }
//     }
// }