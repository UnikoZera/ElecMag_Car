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
void Get_Motor_Info(void);
void Motor_Stop(void);

typedef struct
{
    float speed;
    float angle;        // 角度
    float acceleration; // 加速度
} Motor_DataTypeDef;

extern Motor_DataTypeDef motor_left_data; // 电机数据结构体
extern Motor_DataTypeDef motor_right_data; // 电机数据结构体

#endif /* INC_MOTOR_H_ */
