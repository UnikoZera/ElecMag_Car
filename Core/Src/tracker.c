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
    PID_Init(&motor_right_pid, MOTOR_KP, MOTOR_KI, MOTOR_KD, MOTOR_DT);;
    PID_SetOutputLimits(&motor_right_pid, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT);
    PID_SetIntegralLimits(&motor_right_pid, MOTOR_MIN_INTEGRAL, MOTOR_MAX_INTEGRAL);;
    
    /* 初始化方向控制PID */
    PID_Init(&direction_pid, 2.0f, 0.05f, 0.1f, 0.01f);
    PID_SetOutputLimits(&direction_pid, -500.0f, 500.0f);
    PID_SetIntegralLimits(&direction_pid, -100.0f, 100.0f);
}


void PID_Controllers_Updater(float target_speed, 
                            float current_left_speed, 
                            float current_right_speed,
                            float target_direction,
                            float current_direction,
                            float target_angle,
                            float current_angle)
{
    // 更新左电机速度PID
    PID_SetTarget(&motor_left_pid, target_speed);
    float left_output = PID_Compute(&motor_left_pid, current_left_speed);
    
    // 更新右电机速度PID
    PID_SetTarget(&motor_right_pid, target_speed);
    float right_output = PID_Compute(&motor_right_pid, current_right_speed);
    
    // 更新方向控制PID
    PID_SetTarget(&direction_pid, target_direction);
    float direction_output = PID_Compute(&direction_pid, current_direction);
    
    // 更新平衡控制PID
    PID_TypeDef balance_pid;
    PID_Init(&balance_pid, 15.0f, 0.8f, 0.3f, 0.01f);
    PID_SetOutputLimits(&balance_pid, -800.0f, 800.0f);
    PID_SetIntegralLimits(&balance_pid, -150.0f, 150.0f);
    
    // 计算平衡输出
    float balance_output = PID_Compute(&balance_pid, current_angle);
    
    // 设置电机速度
    Motor_SetSpeed(left_output + direction_output + balance_output,
                   right_output - direction_output + balance_output);
}

void Tracker_Compute(void)
{


}