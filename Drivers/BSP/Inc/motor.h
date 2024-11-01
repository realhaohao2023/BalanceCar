/*
 * motor.h
 *
 *  Created on: Sep 23, 2024
 *      Author: 20541
 */
#ifndef __MOTOR_H
#define __MOTOR_H

#include "tim.h"
#include "gpio.h"

void motor_init();
void set_speed_l(int16_t speed_l);
void set_speed_r(int16_t speed_r);
void set_speed(int16_t left,int16_t right);
int8_t motor_stop(int8_t angle);

#endif

