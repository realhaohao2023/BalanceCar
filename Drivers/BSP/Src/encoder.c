/*
 * encoder.c
 *
 *  Created on: Sep 23, 2024
 *      Author: 20541
 */
#include "encoder.h"


int16_t enLeft, enRight;


void enco_init()
{

	HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL); //开启编码器模式
	HAL_TIM_Base_Start_IT(&htim2);                  //开启编码器的中断

	HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_ALL); //开启编码器模式
    HAL_TIM_Base_Start_IT(&htim3);                  //开启编码器的中断

}

//并非速度，而是函数执行周期内的编码器计数值，根据具体的函数执行周期来计算速度
//使用freeRTOS,而非定时器定时中断
//int16_t Get_SpLeft()
//{
//	enLeft = __HAL_TIM_GetCounter(&htim2);//获取计数值
//	__HAL_TIM_SetCounter(&htim2, 0);//清空计数值
//	return enLeft;
//}
//
//int16_t Get_SpRight()
//{
//	enRight = __HAL_TIM_GetCounter(&htim3);//获取计数值
//	__HAL_TIM_SetCounter(&htim3, 0);//清空计数值
//	return enRight;
//}

void encoder_GetData(int16_t *SpLeft, int16_t *SpRight, uint8_t enPriod)
{

	enLeft = __HAL_TIM_GetCounter(&htim2);//获取计数值
	__HAL_TIM_SetCounter(&htim2, 0);//清空计数值
	

	enRight = __HAL_TIM_GetCounter(&htim3);//获取计数值
	__HAL_TIM_SetCounter(&htim3, 0);//清空计数值
	
}





