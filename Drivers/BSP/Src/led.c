/*
 * led.c
 *
 *  Created on: Sep 23, 2024
 *      Author: 20541
 */
#include "led.h"


void led1_on()
{
	HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_RESET);
}

void led1_off()
{
	HAL_GPIO_WritePin(GPIOA, LED1_Pin, GPIO_PIN_SET);
}

void led1_turn()
{

    HAL_GPIO_TogglePin(GPIOA, LED1_Pin);

}

void led2_on()
{
    HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_RESET);
}

void led2_off()
{
    HAL_GPIO_WritePin(GPIOA, LED2_Pin, GPIO_PIN_SET);
}

void led2_turn()
{
    HAL_GPIO_TogglePin(GPIOA, LED2_Pin);
}

void led3_on()
{
    HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_RESET);
}

void led3_off()
{
    HAL_GPIO_WritePin(GPIOA, LED3_Pin, GPIO_PIN_SET);
}

void led3_turn()
{
    HAL_GPIO_TogglePin(GPIOA, LED3_Pin);
}

void led4_on()
{
    HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_RESET);
}

void led4_off()
{
    HAL_GPIO_WritePin(GPIOB, LED4_Pin, GPIO_PIN_SET);
}

void led4_turn()
{
    HAL_GPIO_TogglePin(GPIOB, LED4_Pin);
}

