/*
 * key.c
 *
 *  Created on: Sep 24, 2024
 *      Author: 20541
 */
#include "key.h"


KEYS ScanPressedKey(uint32_t timeout)
{
	uint32_t tickstart = HAL_GetTick();
	const uint32_t btnDelay = 20;
	while (1)
	{
#ifdef KEY1_Pin
		if (HAL_GPIO_ReadPin(GPIOB, KEY1_Pin) == GPIO_PIN_RESET)
		{
			HAL_Delay(btnDelay);
			if (HAL_GPIO_ReadPin(GPIOB, KEY1_Pin) == GPIO_PIN_RESET)
				return KEY1;
		}
#endif
#ifdef KEY2_Pin
		if (HAL_GPIO_ReadPin(GPIOB, KEY2_Pin) == GPIO_PIN_RESET)
		{
			HAL_Delay(btnDelay);
			if (HAL_GPIO_ReadPin(GPIOB, KEY2_Pin) == GPIO_PIN_RESET)
				return KEY2;
		}
#endif
#ifdef KEY3_Pin
		if (HAL_GPIO_ReadPin(GPIOB, KEY3_Pin) == GPIO_PIN_RESET)
		{
			HAL_Delay(btnDelay);
			if (HAL_GPIO_ReadPin(GPIOB, KEY3_Pin) == GPIO_PIN_RESET)
				return KEY3;
		}
#endif
#ifdef KEY4_Pin
		if (HAL_GPIO_ReadPin(GPIOA, KEY4_Pin) == GPIO_PIN_SET)
		{
			HAL_Delay(btnDelay);
			if (HAL_GPIO_ReadPin(GPIOA, KEY4_Pin) == GPIO_PIN_SET)
				return KEY4;
		}
#endif
		if (timeout != KEY_WAIT_ALWAYS) {
			if ((HAL_GetTick() - tickstart) > timeout)
				break;
		}
	}
	return KEY_NONE;
}

