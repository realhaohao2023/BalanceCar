#include "PID.h"


// //传感器数据变量
// int Encoder_Left,Encoder_Right;
// float pitch,roll,yaw;
// short gyrox,gyroy,gyroz;
// short	aacx,aacy,aacz;
 

// //参数
// float Vertical_Kp=-180,Vertical_Kd=-5;			//直立环 数量级（Kp：0~1000、Kd：0~10）PD控制器
// float Velocity_Kp=-3.2,Velocity_Ki=-0.006;		    //速度环 数量级（Kp：0~1）             PI控制器
// float Turn_Kp=10,Turn_Kd=0.1;						//转向环

// //速度环PI控制器
// //输入：期望速度、左编码器、右编码器
// //该任务放入freertos.c中的SpeedPID函数中
// int Velocity(int Target,int encoder_L,int encoder_R)
// {
// 	static int Err_LowOut_last,Encoder_S;
// 	static float a=0.7;
// 	int Err,Err_LowOut,temp;
// 	//Velocity_Ki=Velocity_Kp/200;
// 	//1、计算偏差值
// 	//Err=(encoder_L+encoder_R)-Target;
// 	Err=(encoder_L+encoder_R)/2-Target;
// 	//2、低通滤波
// 	Err_LowOut=(1-a)*Err+a*Err_LowOut_last;
// 	Err_LowOut_last=Err_LowOut;
// 	//3、积分
// 	Encoder_S+=Err_LowOut;
// 	//4、积分限幅(-20000~20000)
// 	Encoder_S=Encoder_S>20000?20000:(Encoder_S<(-20000)?(-20000):Encoder_S);
	
// 	//5、速度环计算
// 	temp=Velocity_Kp*Err_LowOut+Velocity_Ki*Encoder_S;
// 	//printf("temp=%d\n",temp);
// 	return temp;

//     //set_speed(temp,temp);//PID计算后，设置电机速度
// }

// //直立环PD控制器
// //输入：期望角度、真实角度、角速度
// int Vertical(float Med,float Angle,float gyro_Y)
// {
// 	int temp;
// 	temp=Vertical_Kp*(Angle-Med)+Vertical_Kd*gyro_Y;
// 	return temp;
// }

// //转向环PD控制器
// //输入：角速度、角度值
// int Turn(float gyro_Z,int Target_turn)
// {
// 	int temp;
// 	temp=Turn_Kp*Target_turn+Turn_Kd*gyro_Z;
// 	return temp;
// }



//第二种方案
//直立环
#define Middle_angle 3  //直立环的机械中值

int8_t Angle_Balance;




//直立环
float PID_Balance_Calc(PID *pid, float Angle,float Gyro)  
{  
   float Angle_bias,Gyro_bias;
	 Angle_bias=Middle_angle-Angle;                    				//求出平衡的角度中值 和机械相关
	 Gyro_bias=0-Gyro; 
	 
	 pid->output= -pid->Kp/100*Angle_bias-Gyro_bias*pid->Kd/100; //计算平衡控制的电机PWM  PD控制   kp是P系数 kd是D系数 
	
	if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
	if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;
	
	//printf("output=%f\n",pid->output);
	return pid->output;
}

//速度环
//为了防止积分项过度累积，引入积分项的限幅是一种常见的做法。
//限制积分项的幅值可以防止积分项过度增加，从而限制了系统的累积误差。这样可以避免系统过度响应或者不稳定。
float abs_limit(float value, float ABS_MAX)   //积分限幅，设置最大值。
{
	if(value > ABS_MAX)
		value = ABS_MAX;

	if(value< -ABS_MAX)
		value = -ABS_MAX;
	return value;
}

float PID_Speed_Calc(PID *pid, int encoder_left, int encoder_right)  
{  
    static float Encoder_bias;
	pid->Error = 0-(encoder_left+encoder_right);    //获取最新速度偏差==测量速度（左右编码器之和）-目标速度（此处为零）             
    Encoder_bias *=0.8;    //一阶低通滤波器     
    Encoder_bias += pid->Error*0.2;  //一阶低通滤波器  
    pid->SumError +=Encoder_bias;
    
	pid->output  = -pid->Kp* Encoder_bias 
	               -pid->Ki* abs_limit( pid->SumError, 10000);

	if(motor_stop(Angle_Balance)==1)   pid->SumError=0;      //电机关闭后清除积分	
	
	if(pid->output > pid->outputmax )    pid->output = pid->outputmax;
	if(pid->output < - pid->outputmax )  pid->output = -pid->outputmax;						
	return pid->output ;   //输出为pwm值
}

void PID_Init(PID *pid, float Kp, float Ki, float Kd, float Integralmax)
{
	pid->Kp = Kp;
	pid->Ki = Ki;
	pid->Kd = Kd;
	pid->Integralmax = Integralmax;
	pid->outputmax = 100;
	pid->PrevError = 0;
	pid->LastError = 0;
	pid->Error = 0;
	pid->DError = 0;
	pid->SumError = 0;
	pid->output = 0;
}

int PWM_Limit(int IN,int max,int min)
{
	int OUT = IN;
	if(OUT>max) OUT = max;
	if(OUT<min) OUT = min;
	return OUT;
}