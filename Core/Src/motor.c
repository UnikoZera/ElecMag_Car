/*
 * motor.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "motor.h"

#define PWM_PERIOD 1000 // PWM周期
#define Calculate_PWM_Value(value) ((uint32_t)(value * PWM_PERIOD / 1000)) // 把0~100转化为pwm的真实占空比

#define MOTOR_REVOLUTION 500.0f
#define MOTOR_REDUCTION_RATIO 1/20.409f // 电机减速比
#define MOTOR_DIVISION 4 
#define MOTOR_PULSE_PER_REVOLUTION 40817.0f    // 每转一圈的脉冲数

#define GPIO_LEFT_PORT GPIOB // 左电机控制引脚所在端口
#define GPIO_LEFT_MOTOR_IN1 GPIO_PIN_15 // 左电机正转引脚
#define GPIO_LEFT_MOTOR_IN2 GPIO_PIN_14 // 左电机反转引脚

#define GPIO_RIGHT_PORT GPIOB // 右电机控制引脚所在端口
#define GPIO_RIGHT_MOTOR_IN1 GPIO_PIN_13 // 右电机正转引脚
#define GPIO_RIGHT_MOTOR_IN2 GPIO_PIN_12 // 右电机反转引脚

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
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN1, GPIO_PIN_RESET); // 左电机正转引脚拉低
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN2, GPIO_PIN_SET);   // 左电机反转引脚拉高
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Calculate_PWM_Value(left_speed));
    }
    else if (left_speed > 0)
    {
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN1, GPIO_PIN_SET);   // 左电机正转引脚拉高
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN2, GPIO_PIN_RESET); // 左电机反转引脚拉低
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Calculate_PWM_Value(left_speed));
    }
    else
    {
        left_speed = 0;
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN1, GPIO_PIN_RESET); // 左电机正转引脚拉低
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN2, GPIO_PIN_RESET); // 左电机反转引脚拉低
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0); // 停止左电机
    }

    if (right_speed < 0)
    {
        right_speed = -right_speed; // 取绝对值
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN1, GPIO_PIN_RESET); // 右电机正转引脚拉低
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN2, GPIO_PIN_SET);   // 右电机反转引脚拉高
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Calculate_PWM_Value(right_speed));
    }
    else if (right_speed > 0)
    {
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN1, GPIO_PIN_SET);   // 右电机正转引脚拉高
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN2, GPIO_PIN_RESET); // 右电机反转引脚拉低
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Calculate_PWM_Value(right_speed));
    }
    else
    {
        right_speed = 0;
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN1, GPIO_PIN_RESET); // 右电机正转引脚拉低
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN2, GPIO_PIN_RESET); // 右电机反转引脚拉低
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0); // 停止右电机
    }    
}

void Get_Motor_Info(void)
{
    static uint16_t preLeftCount = 0;
    static uint16_t preRightCount = 0;
    static uint32_t pre_time = 0;
    static float pre_left_speed = 0.0f;
    static float pre_right_speed = 0.0f;
    static int32_t total_left_count = 0;
    static int32_t total_right_count = 0;

    uint32_t current_time = HAL_GetTick();

    // 防止第一次调用时时间差为0
    if (pre_time == 0)
    {
        pre_time = current_time;
        preLeftCount = __HAL_TIM_GET_COUNTER(&htim2);
        preRightCount = __HAL_TIM_GET_COUNTER(&htim3);
        return;
    }

    uint16_t left_encoder_count = __HAL_TIM_GET_COUNTER(&htim2);
    uint16_t right_encoder_count = __HAL_TIM_GET_COUNTER(&htim3);

    // 计算差值，处理溢出情况
    int16_t left_diff = (int16_t)(left_encoder_count - preLeftCount);
    int16_t right_diff = (int16_t)(right_encoder_count - preRightCount);
    
    // 累加总计数，这样可以追踪多圈转动
    total_left_count += left_diff;
    total_right_count += right_diff;

    float time_diff = (current_time - pre_time) / 1000.0f; // 转换为秒

    if (time_diff > 0)
    {
        // 计算左电机数据
        motor_left_data.angle = (float)fabs(fmodf((float)total_left_count, MOTOR_PULSE_PER_REVOLUTION) * 360.0f / MOTOR_PULSE_PER_REVOLUTION);
        float left_angle_diff = (float)left_diff * 360.0f / MOTOR_PULSE_PER_REVOLUTION;
        motor_left_data.speed = left_angle_diff / time_diff;                                 // 度/秒
        motor_left_data.acceleration = (motor_left_data.speed - pre_left_speed) / time_diff; // 度/秒²

        // 计算右电机数据
        motor_right_data.angle = (float)fabs(fmodf((float)total_right_count, MOTOR_PULSE_PER_REVOLUTION) * 360.0f / MOTOR_PULSE_PER_REVOLUTION);
        float right_angle_diff = (float)right_diff * 360.0f / MOTOR_PULSE_PER_REVOLUTION;
        motor_right_data.speed = right_angle_diff / time_diff;                                 // 度/秒
        motor_right_data.acceleration = (motor_right_data.speed - pre_right_speed) / time_diff; // 度/秒²

        // 更新历史数据
        pre_left_speed = motor_left_data.speed;
        pre_right_speed = motor_right_data.speed;    
    }

    Lowpass_Filter_Encoder_Left(&motor_left_data.filtered_speed, &motor_left_data.speed, 0.33f); // 低通滤波
    Lowpass_Filter_Encoder_Right(&motor_right_data.filtered_speed, &motor_right_data.speed, 0.33f); // 低通滤波

    preLeftCount = left_encoder_count;
    preRightCount = right_encoder_count;
    pre_time = current_time;
}

void Lowpass_Filter_Encoder_Left(float *dst, float* input, float alpha)
{
    static float prev_output_left = 0.0f; 
    *dst = alpha * (*input) + (1.0f - alpha) * prev_output_left;
    prev_output_left = *dst;
}

void Lowpass_Filter_Encoder_Right(float *dst, float* input, float alpha)
{
    static float prev_output_right = 0.0f; 
    *dst = alpha * (*input) + (1.0f - alpha) * prev_output_right;
    prev_output_right = *dst;
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}