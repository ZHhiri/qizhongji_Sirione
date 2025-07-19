#ifndef __UPPERSERVO_H__
#define __UPPERSERVO_H__

#include "DJI.h"
#include "Caculate.h"
void upperservotask(void const * argument);
void gantry_Motor_init();
void pid_reset(PID_t *pid, float kp, float ki, float kd);
void v_correct(float target[5]);

extern VelocityPlan_T Y_wheel;
#endif
