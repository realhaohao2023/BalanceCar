/*
 * encoder.h
 *
 *  Created on: Sep 23, 2024
 *      Author: 20541
 */
#ifndef __ENCODER_H
#define __ENCODER_H

#include "tim.h"
#include "OLED.h"

void enco_init();
//int16_t Get_SpLeft();
//int16_t Get_SpRight();
//void encoder_GetData(int16_t *EncoL, int16_t *EncoR);
void encoder_GetData(int16_t *SpLeft, int16_t *SpRight, uint8_t enPriod);


#endif

