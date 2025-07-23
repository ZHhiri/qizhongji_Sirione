/*DJI电机速度位置伺服*/

#include "Caculate.h"
#include "math.h"
#include "PID.h"

//位置伺服函数
void positionServo(float ref, DJI_t * motor){
	
	motor->posPID.ref = ref*8191;
	motor->posPID.fdb = motor->AxisData.AxisAngle_inDegree;
	//IncrPID_Calc(&motor->posPID);
	PosePID_Calc(&motor->posPID);

	motor->speedPID.ref = motor->posPID.output;
	motor->speedPID.fdb = motor->FdbData.rpm;
	//IncrPID_Calc(&motor->speedPID);
	PosePID_Calc(&motor->speedPID);

}


void RS485_positionServo(float ref, DJI_t * motor,float fdb){
	
	motor->posPID.ref = ref;
	motor->posPID.fdb = fdb;
	//IncrPID_Calc(&motor->posPID);
	RS485_PosePID_Calc(&motor->posPID);

	motor->speedPID.ref = motor->posPID.output;
	motor->speedPID.fdb = motor->FdbData.rpm;
	//IncrPID_Calc(&motor->speedPID);
	RS485_PosePID_Calc(&motor->speedPID);

}
void positionServo_2(float ref, DJI_t * motor,DJI_t * motor_2){
	
	motor->posPID.ref = ref*8191;
	motor->posPID.fdb = motor->AxisData.AxisAngle_inDegree;
	//IncrPID_Calc(&motor->posPID);
	PosePID_Calc(&motor->posPID);
    motor->speedPID.ref = motor->posPID.output;
	motor->speedPID.fdb = motor->FdbData.rpm;
	//IncrPID_Calc(&motor->speedPID);
	PosePID_Calc(&motor->speedPID);
    motor_2->speedPID.ref =-motor->posPID.output;
	motor_2->speedPID.fdb = motor_2->FdbData.rpm;
	//IncrPID_Calc(&motor->speedPID);
	PosePID_Calc(&motor->speedPID);




}
//速度伺服函数
void speedServo(float ref, DJI_t * motor){
	motor->speedPID.ref = ref;
	motor->speedPID.fdb = motor->FdbData.rpm;
	IncrPID_Calc(&motor->speedPID);
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
void VelocityPlanning(float initialAngle, float maxAngularVelocity, float AngularAcceleration, float targetAngle, float currentTime, volatile float *currentAngle)
{

    float angleDifference = targetAngle - initialAngle;     // 计算到目标位置的角度差
    float sign            = (angleDifference > 0) ? 1 : -1; // 判断角度差的正负(方向)

    float accelerationTime = maxAngularVelocity / AngularAcceleration;                                                      // 加速(减速)总时间
    float constTime        = (fabs(angleDifference) - AngularAcceleration * pow(accelerationTime, 2)) / maxAngularVelocity; // 匀速总时间
    float totalTime        = constTime + accelerationTime * 2;                                                              // 计算到达目标位置所需的总时间

    // 判断能否达到最大速度
    if (constTime > 0) {
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= accelerationTime + constTime) {
            // 匀速阶段
            *currentAngle = initialAngle + sign * maxAngularVelocity * (currentTime - accelerationTime) + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime - constTime;
            *currentAngle          = initialAngle + sign * maxAngularVelocity * constTime + 0.5 * sign * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
        }
    } else {
        maxAngularVelocity = sqrt(fabs(angleDifference) * AngularAcceleration);
        accelerationTime   = maxAngularVelocity / AngularAcceleration;
        totalTime          = 2 * accelerationTime;
        constTime          = 0;
        // 根据当前时间判断处于哪个阶段
        if (currentTime <= accelerationTime) {
            // 加速阶段
            *currentAngle = initialAngle + sign * 0.5 * AngularAcceleration * pow(currentTime, 2);
        } else if (currentTime <= totalTime) {
            // 减速阶段
            float decelerationTime = currentTime - accelerationTime; // 减速时间
            *currentAngle          = initialAngle + sign * 0.5 * AngularAcceleration * pow(accelerationTime, 2) + sign * (maxAngularVelocity * decelerationTime - 0.5 * AngularAcceleration * pow(decelerationTime, 2));
        } else {
            // 达到目标位置
            *currentAngle = targetAngle;
           
        }
    }
}

/**
 * @brief 初始化速度规划结构体
 * @param plan 速度规划结构体指针
 * @param StartPosition 起始位置
 * @param StopPosition 结束位置
 * @param MaxAccelerate 最大加速度
 * @param MaxVelocity 最大速度
 */
void VelocityPlan_Init(VelocityPlan_T* plan, float StartPosition, float StopPosition, float MaxAccelerate, float MaxVelocity) {
    /* 属性赋值 */
    plan->Position[0] = StartPosition;
    plan->Position[3] = StopPosition;
    plan->MaxVelocity = fabsf(MaxVelocity);
    plan->MaxAccelerate = fabsf(MaxAccelerate);
    plan->DeltaPosition = fabsf(StartPosition - StopPosition);

    plan->type = powf(plan->MaxVelocity, 2) / plan->MaxAccelerate <= plan->DeltaPosition; // 匀速段判断
    plan->direction = (StartPosition - StopPosition) > 0 ? -1 : 1;                         // 方向判断
    printf("%d\n", (int)plan->direction);
    
    switch (plan->type) {
        case 0: // 无匀速段
            plan->Time[0] = plan->Time[1] = sqrtf(plan->DeltaPosition / plan->MaxAccelerate);
            plan->Time[2] = plan->Time[1] * 2;
            plan->Position[1] = plan->Position[2] = (StartPosition + StopPosition) / 2;
            break;

        case 1: // 有匀速段
            plan->Time[0] = plan->MaxVelocity / plan->MaxAccelerate;
            plan->Time[1] = plan->DeltaPosition / plan->MaxVelocity;
            plan->Time[2] = plan->Time[0] + plan->Time[1];
            plan->Position[1] = StartPosition + powf(plan->MaxVelocity, 2) / (2 * plan->MaxAccelerate) * plan->direction;
            plan->Position[2] = StopPosition - powf(plan->MaxVelocity, 2) / (2 * plan->MaxAccelerate) * plan->direction;
            break;
    }
}

/**
 * @brief 计算给定时间点的位置
 * @param plan 速度规划结构体指针
 * @param NowTime 当前时间
 * @return 当前时间点应该达到的目标位置（这个返回值就是要给位置环PID的设定值）
 */
float CalcPos(VelocityPlan_T* plan, float NowTime) {
    if (NowTime < plan->Time[0]) {
        plan->CurrPosition = plan->Position[0] + 0.5f * plan->MaxAccelerate * powf(NowTime, 2) * plan->direction;
    } else if (NowTime < plan->Time[1]) {
        plan->CurrPosition = plan->Position[1] + plan->MaxVelocity * (NowTime - plan->Time[0]) * plan->direction;
    } else if (NowTime < plan->Time[2]) {
        plan->CurrPosition = plan->Position[2] + plan->MaxVelocity * (NowTime - plan->Time[1]) * plan->direction
                             - 0.5f * plan->MaxAccelerate * powf(NowTime - plan->Time[1], 2) * plan->direction;
    } else {
        plan->CurrPosition = plan->Position[3];
    }
    return plan->CurrPosition;
}
/*下述内容在PID.o中重复 于是不再重复定义*/
////增量式PID算法
//void PID_Calc(PID_t *pid){
//	pid->cur_error = pid->ref - pid->fdb;
//	pid->output += pid->KP * (pid->cur_error - pid->error[1]) + pid->KI * pid->cur_error + pid->KD * (pid->cur_error - 2 * pid->error[1] + pid->error[0]);
//	pid->error[0] = pid->error[1];
//	pid->error[1] = pid->ref - pid->fdb;
//	/*设定输出上限*/
//	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
//	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;

//}

////比例算法
//void P_Calc(PID_t *pid){
//	pid->cur_error = pid->ref - pid->fdb;
//	pid->output = pid->KP * pid->cur_error;
//	/*设定输出上限*/
//	if(pid->output > pid->outputMax) pid->output = pid->outputMax;
//	if(pid->output < -pid->outputMax) pid->output = -pid->outputMax;
//	
//	if(fabs(pid->output)<pid->outputMin)
//		pid->output=0;

//}
