/*
 * motor.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "motor.h"

void Motor_Init(void)
{
    // 初始化电机相关GPIO和定时器
    // 这里假设使用TIM1和TIM2控制左、右电机的PWM输出
    // HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 左电机
    // HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1); // 右电机

    Motor_SetSpeed(0, 0);
}

/*
 * @brief 设置电机速度
 * @param left_speed: 左电机速度
 * @param right_speed: 右电机速度
 * -1000~1000表示速度范围
 * 该函数会先对速度进行限制，然后设置PWM占空比以控制电机速度。
 */
void Motor_SetSpeed(int left_speed, int right_speed)
{
    // 先进行速度限制
    if (left_speed > MOTOR_MAX_SPEED)
    {
        left_speed = MOTOR_MAX_SPEED;
    }
    else if (left_speed < MOTOR_MIN_SPEED)
    {
        left_speed = MOTOR_MIN_SPEED;
    }

    if (right_speed > MOTOR_MAX_SPEED)
    {
        right_speed = MOTOR_MAX_SPEED;
    }
    else if (right_speed < MOTOR_MIN_SPEED)
    {
        right_speed = MOTOR_MIN_SPEED;
    }
    


}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}