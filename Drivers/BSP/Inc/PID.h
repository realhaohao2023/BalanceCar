#ifndef __PID_H__
#define __PID_H__

#include "main.h"
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "tim.h"
#include "usart.h"
#include "motor.h"


typedef struct PID {
		float  Kp;         //  Proportional Const  P系数
		float  Ki;           //  Integral Const      I系数
		float  Kd;         //  Derivative Const    D系数
		
		float  PrevError ;          //  Error[-2]  
		float  LastError;          //  Error[-1]  
		float  Error;              //  Error[0 ]  
		float  DError;            //pid->Error - pid->LastError	
		float  SumError;           //  Sums of Errors  
		
		float  output;
		
		float  Integralmax;      //积分项的最大值
		float  outputmax;        //输出项的最大值
} PID;


//PID Balance_PID;  //直立环

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float Integralmax);

// int Velocity(int Target,int encoder_L,int encoder_R);
// int Vertical(float Med,float Angle,float gyro_Y);
// int Turn(float gyro_Z,int Target_turn);
float PID_Balance_Calc(PID *pid, float Angle,float Gyro);
float abs_limit(float value, float ABS_MAX);
float PID_Speed_Calc(PID *pid, int encoder_left, int encoder_right);
int PWM_Limit(int IN,int max,int min);

#endif // !__PID_H__
