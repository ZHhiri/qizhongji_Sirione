#include "PID.h"
#include "math.h"
#include "mi_motor.h"
#include "param.h"
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

float mi_pid_compute( PIDController* pid, float error, float dt) {
    // 积分项（带限幅）
    pid->integral += error * dt;
    if (pid->integral > pid->max_integral) pid->integral = pid->max_integral;
    else if (pid->integral < -pid->max_integral) pid->integral = -pid->max_integral;
    
    // 微分项
    float derivative = (error - pid->prev_error) / dt;
    pid->prev_error = error;
    pid->output=pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
    return pid->kp * error + pid->ki * pid->integral + pid->kd * derivative;
}
void position_control_with_pid( float target_pos) {
     
    
    
     {
        // 1. 获取实际位置（通过通信类型2反馈帧）
        float actual_pos = mi_motor[0].Angle; // 需实现此函数
        
        // 2. 计算位置误差
        float error = target_pos - actual_pos;
        
        // 3. 计算PID输出（作为前馈力矩）
        float torque_feedforward = mi_pid_compute(&miPIDController, error, 0.001f); // dt=10ms
        
        // 4. 发送运控指令（将积分项融入力矩）
        motor_controlmode(&mi_motor[0], torque_feedforward, target_pos, 0, 12, 5);
        //set_position(id, target_pos, 0.0f, torque_feedforward, 50.0f, 1.2f);
        
        
    }
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
    if(pid->cur_error < 1 && pid->cur_error > -1)
        pid->output = 0; // 防抖处理

}

void RS485_PosePID_Calc(PID_t *pid)
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
    if(pid->cur_error < 90 && pid->cur_error > -90)
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

