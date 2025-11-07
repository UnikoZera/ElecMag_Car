/*
 * pid.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "pid.h"

/**
 * @brief PID控制器初始化
 * @param pid: PID控制器结构体指针
 * @param kp: 比例系数
 * @param ki: 积分系数
 * @param kd: 微分系数
 * @param dt: 采样周期(秒)这里其实只是给个默认值，实际计算时可以动态调整(tim4来给1us级别的时间间隔)
 */
void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float dt)
{
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->dt = dt;

    // 这里还是得初始化一下时间戳，防止误差
    pid->last_time = __HAL_TIM_GetCounter(&htim4);

    pid->setpoint = 0.0f;
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;

    /* 默认输出限制 */
    pid->output_max = 1000.0f;
    pid->output_min = -1000.0f;

    /* 默认积分限幅 */
    pid->integral_max = 100.0f;
    pid->integral_min = -100.0f;
}

void PID_SetTarget(PID_TypeDef *pid, float setpoint)
{
    pid->setpoint = setpoint;
}

void PID_SetOutputLimits(PID_TypeDef *pid, float min, float max)
{
    pid->output_min = min;
    pid->output_max = max;
}

void PID_SetIntegralLimits(PID_TypeDef *pid, float min, float max)
{
    pid->integral_min = min;
    pid->integral_max = max;
}

float PID_Compute(PID_TypeDef *pid, float input)
{
    float error;
    float output;

    /* 计算时间间隔 */
    uint32_t current_time = __HAL_TIM_GetCounter(&htim4);
    if (current_time < pid->last_time)
    {
        // 计数器溢出处理
        pid->dt = (current_time + (__HAL_TIM_GET_AUTORELOAD(&htim4) + 1 - pid->last_time)) / 1000000.0f;
    }
    else
    {
        pid->dt = (current_time - pid->last_time) / 1000000.0f;
    }
    pid->last_time = current_time;

    /* 计算误差 */
    error = pid->setpoint - input;

    /* 积分项计算 */
    pid->integral += error * pid->dt;

    /* 积分限幅 */
    if (pid->integral > pid->integral_max)
    {
        pid->integral = pid->integral_max;
    }
    else if (pid->integral < pid->integral_min)
    {
        pid->integral = pid->integral_min;
    }

    /* 微分项计算 */
    pid->derivative = (error - pid->last_error) / pid->dt;

    /* PID输出计算 */
    output = pid->kp * error + pid->ki * pid->integral + pid->kd * pid->derivative;

    /* 输出限幅 */
    if (output > pid->output_max)
    {
        output = pid->output_max;
    }
    else if (output < pid->output_min)
    {
        output = pid->output_min;
    }

    pid->last_error = error;

    return output;
}

void PID_Reset(PID_TypeDef *pid)
{
    pid->last_error = 0.0f;
    pid->integral = 0.0f;
    pid->derivative = 0.0f;
}
