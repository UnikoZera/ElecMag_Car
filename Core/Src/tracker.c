/*
 * tracker.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "tracker.h"

#define MOTOR_MAX_OUTPUT 1000.0f // 电机最大输出
#define MOTOR_MIN_OUTPUT -1000.0f // 电机最小输出
#define MOTOR_MAX_INTEGRAL 200.0f // 电机积分最大值
#define MOTOR_MIN_INTEGRAL -200.0f // 电机积分最小值

#define MOTOR_KP 1.2f // 左右电机速度PID比例系数
#define MOTOR_KI 0.1f // 左右电机速度PID积分系数
#define MOTOR_KD 0.05f // 左右电机速度PID微分系数
#define MOTOR_DT 0.01f // PID采样周期，单位为秒

#define ADC_CENTER Vref/2.0f

PID_TypeDef motor_left_pid;    // 左电机速度PID
PID_TypeDef motor_right_pid;   // 右电机速度PID

PID_TypeDef direction_pid;     // 方向控制PID

void Tracker_Init(void)
{
    /* 初始化左电机PID - 速度控制 */
    PID_Init(&motor_left_pid, MOTOR_KP, MOTOR_KI, MOTOR_KD, MOTOR_DT);
    PID_SetOutputLimits(&motor_left_pid, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT);
    PID_SetIntegralLimits(&motor_left_pid, MOTOR_MIN_INTEGRAL, MOTOR_MAX_INTEGRAL);
    
    /* 初始化右电机PID - 速度控制 */
    PID_Init(&motor_right_pid, MOTOR_KP, MOTOR_KI, MOTOR_KD, MOTOR_DT);
    PID_SetOutputLimits(&motor_right_pid, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT);
    PID_SetIntegralLimits(&motor_right_pid, MOTOR_MIN_INTEGRAL, MOTOR_MAX_INTEGRAL);
    
    /* 初始化方向控制PID */
    PID_Init(&direction_pid, 2.0f, 0.05f, 0.1f, 0.01f);
    PID_SetOutputLimits(&direction_pid, -500.0f, 500.0f);
    PID_SetIntegralLimits(&direction_pid, -100.0f, 100.0f);
}


void PID_Motor_Controllers_Updater(float target_left_speed, float target_right_speed)
{
    // 更新左电机速度PID
    PID_SetTarget(&motor_left_pid, target_left_speed);
    float left_output = PID_Compute(&motor_left_pid, motor_left_data.speed);
    
    // 更新右电机速度PID
    PID_SetTarget(&motor_right_pid, target_right_speed);
    float right_output = PID_Compute(&motor_right_pid, motor_right_data.speed);

    // 设置电机速度
    Motor_SetSpeed(left_output, right_output);
}

void Tracker_Compute(void)
{
    
    
}