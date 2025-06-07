/*
 * motor.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "motor.h"

#define PWM_PERIOD 1000 // PWM周期
#define Calculate_PWM_Value(value) ((uint32_t)((value) * PWM_PERIOD / 1000)) // 把0~100转化为pwm的真实占空比

Motor_DataTypeDef motor_left_data;  // 电机数据结构体
Motor_DataTypeDef motor_right_data; // 电机数据结构体

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); // 左电机
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2); // 右电机

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1); // 启动编码器
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2); // 启动编码器
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1); // 启动编码器
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2); // 启动编码器
    __HAL_TIM_SET_COUNTER(&htim2, 0);             // 重置编码器计数器
    __HAL_TIM_SET_COUNTER(&htim3, 0);             // 重置编码器计数器

    // 初始化电机数据
    motor_left_data.speed = 0.0f;
    motor_left_data.angle = 0.0f;
    motor_left_data.acceleration = 0.0f;
    motor_right_data.speed = 0.0f;
    motor_right_data.angle = 0.0f;
    motor_right_data.acceleration = 0.0f;
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

    if (left_speed < 0)
    {
        left_speed = -left_speed; // 取绝对值

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Calculate_PWM_Value(left_speed));
    }
    else if (left_speed > 0)
    {

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Calculate_PWM_Value(left_speed));
    }
    else
    {
        left_speed = 0;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // 停止左电机
    }

    if (right_speed < 0)
    {
        right_speed = -right_speed; // 取绝对值

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Calculate_PWM_Value(right_speed));
    }
    else if (right_speed > 0)
    {

        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Calculate_PWM_Value(right_speed));
    }
    else
    {
        right_speed = 0;
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // 停止右电机
    }    
}

void Get_Motor_Info(void)
{
    static unsigned int preLeftCount = 0;
    static unsigned int preRightCount = 0;
    static uint32_t pre_time = 0;
    static float pre_left_speed = 0.0f;
    static float pre_right_speed = 0.0f;

    uint32_t current_time = HAL_GetTick();

    // 防止第一次调用时时间差为0
    if (pre_time == 0)
    {
        pre_time = current_time;
        return;
    }

    unsigned int left_encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
    unsigned int right_encoder_count = __HAL_TIM_GET_COUNTER(&htim3);

    float time_diff = (current_time - pre_time) / 1000.0f; // 转换为秒

    if (time_diff > 0)
    {
        // 计算左电机数据
        motor_left_data.angle = (float)left_encoder_count * 360.0f / 500.0f;
        float left_angle_diff = (float)(left_encoder_count - preLeftCount) * 360.0f / 500.0f;
        motor_left_data.speed = left_angle_diff / time_diff;                                 // 度/秒
        motor_left_data.acceleration = (motor_left_data.speed - pre_left_speed) / time_diff; // 度/秒²

        // 计算右电机数据
        motor_right_data.angle = (float)right_encoder_count * 360.0f / 500.0f;
        float right_angle_diff = (float)(right_encoder_count - preRightCount) * 360.0f / 500.0f;
        motor_right_data.speed = right_angle_diff / time_diff;                                  // 度/秒
        motor_right_data.acceleration = (motor_right_data.speed - pre_right_speed) / time_diff; // 度/秒²

        // 更新历史数据
        pre_left_speed = motor_left_data.speed;
        pre_right_speed = motor_right_data.speed;
    }

    preLeftCount = left_encoder_count;
    preRightCount = right_encoder_count;
    pre_time = current_time;
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}