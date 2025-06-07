/*
 * pid.h
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#ifndef INC_PID_H_
#define INC_PID_H_

#include <stdint.h>

/* PID控制器结构体定义 */
typedef struct {
    /* PID参数 */
    float kp;           // 比例系数
    float ki;           // 积分系数 
    float kd;           // 微分系数
    
    /* PID计算中间变量 */
    float setpoint;     // 目标值
    float last_error;   // 上一次误差
    float integral;     // 积分累积值
    float derivative;   // 微分值
    
    /* 输出限制 */
    float output_max;   // 输出最大值
    float output_min;   // 输出最小值
    
    /* 积分限幅 */
    float integral_max; // 积分上限
    float integral_min; // 积分下限
    
    /* 采样时间 */
    float dt;           // 采样周期(秒)
    
} PID_TypeDef;

void PID_Init(PID_TypeDef *pid, float kp, float ki, float kd, float dt);
void PID_SetTarget(PID_TypeDef *pid, float setpoint);
void PID_SetOutputLimits(PID_TypeDef *pid, float min, float max);
void PID_SetIntegralLimits(PID_TypeDef *pid, float min, float max);
float PID_Compute(PID_TypeDef *pid, float input);
void PID_Reset(PID_TypeDef *pid);

#endif /* INC_PID_H_ */
