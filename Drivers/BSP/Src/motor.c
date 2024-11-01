/*
 * motor.c
 *
 *  Created on: Sep 23, 2024
 *      Author: 20541
 */
/*左轮 TIM4_CH3 PB8 PWMA PB14 15
 *右轮 TIM4_CH4 PB9 PWMB PB12 13
 */
#include "motor.h"
#include "tim.h"



void motor_init()
{
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim4,TIM_CHANNEL_4);
}


void set_speed_l(int16_t speed_l)
{
	if (speed_l > 0)
	{
		//限幅，最大100
		if(speed_l>100)
		{
			speed_l=100;
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, speed_l);

	}
	else
	{
		if (speed_l < -100)
		{
			speed_l = -100;
		}
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_4, -speed_l);

	}
}

void set_speed_r(int16_t speed_r)
{
	if (speed_r > 0)
	{
		if(speed_r>100)
		{
			speed_r=100;
		}

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_RESET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_SET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, speed_r);

	}
	else
	{
		if (speed_r < -100)
		{
			speed_r = -100;
		}
		
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, GPIO_PIN_SET);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_15, GPIO_PIN_RESET);
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, -speed_r);
	}
}

void set_speed(int16_t left,int16_t right)
{
	set_speed_l(left);
	set_speed_r(right);

}

int8_t motor_stop(int8_t angle)
{
	uint8_t temp=0;
	if (angle > 40 || angle < -40)
	{
		set_speed(0,0);
		temp=0;
	}
	else
	{
		temp=1;
	}
	return temp;	
}
