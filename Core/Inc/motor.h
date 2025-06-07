/*
 * motor.h
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#ifndef INC_MOTOR_H_
#define INC_MOTOR_H_

#include <stdint.h>
#include <math.h>
#include "main.h"
#include "gpio.h"
#include "tim.h"

#define MOTOR_MAX_SPEED 1000  // 电机最大速度
#define MOTOR_MIN_SPEED -1000 // 电机最小速度

void Motor_Init(void);
void Motor_SetSpeed(int left_speed, int right_speed);
void Motor_Stop(void);

#endif /* INC_MOTOR_H_ */
