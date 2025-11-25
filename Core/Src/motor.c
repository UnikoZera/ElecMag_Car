/*
 * motor.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "motor.h"

#define PWM_PERIOD (200.0f - 1.0f)                                          // PWM周期
#define Calculate_PWM_Duty(value) ((uint32_t)(value * PWM_PERIOD / 100.0f)) // 把0~100转化为pwm的真实占空比

#define MOTOR_REVOLUTION 500.0f
#define MOTOR_REDUCTION_RATIO (1.0f / 20.409f) // 电机减速比
#define MOTOR_DIVISION 4.0f
#define MOTOR_PULSE_PER_REVOLUTION 40817.0f // 每转一圈的脉冲数

#define GPIO_LEFT_PORT GPIOB
#define GPIO_LEFT_MOTOR_IN1 GPIO_PIN_4
#define GPIO_LEFT_MOTOR_IN2 GPIO_PIN_5

#define GPIO_RIGHT_PORT GPIOB
#define GPIO_RIGHT_MOTOR_IN1 GPIO_PIN_0
#define GPIO_RIGHT_MOTOR_IN2 GPIO_PIN_1

Motor_DataTypeDef motor_left_data;  // 电机数据结构体
Motor_DataTypeDef motor_right_data; // 电机数据结构体

#pragma region PID 控制器
#define MOTOR_MAX_OUTPUT 100.0f
#define MOTOR_MIN_OUTPUT -100.0f
#define MOTOR_MAX_INTEGRAL 60.0f
#define MOTOR_MIN_INTEGRAL -60.0f

#define MOTOR_KP 0.65f   // 左右电机速度PID比例系数
#define MOTOR_KI 1.5f  // 左右电机速度PID积分系数
#define MOTOR_KD 0.03f // 左右电机速度PID微分系数
#define MOTOR_DT 0.01f  // PID采样周期，单位为秒

PID_TypeDef motor_left_speed_pid, motor_right_speed_pid;       // 电机速度PID
PID_TypeDef motor_left_position_pid, motor_right_position_pid; // 电机位置PID
#pragma endregion

typedef struct
{
    float prevNum; // i can't believe it's the only way to do it.
} LPF1State;

LPF1State lpf1_left_speed, lpf1_right_speed = {0.0f};
// LPF1State lpf1_left_acceleration, lpf1_right_acceleration = {0.0f};

static float LPF1_Update(float *dst, float *input, float alpha, LPF1State *state)
{
    *dst = alpha * (*input) + (1.0f - alpha) * state->prevNum;
    state->prevNum = *dst;
    return *dst;
}

void Motor_Init(void)
{
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);

    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_2);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_1);
    HAL_TIM_Encoder_Start(&htim3, TIM_CHANNEL_2);
    __HAL_TIM_SET_COUNTER(&htim2, 0);
    __HAL_TIM_SET_COUNTER(&htim3, 0);

    motor_left_data.speed = 0.0f;
    motor_left_data.angle = 0.0f;
    // motor_left_data.acceleration = 0.0f;
    motor_right_data.speed = 0.0f;
    motor_right_data.angle = 0.0f;
    // motor_right_data.acceleration = 0.0f;
    Motor_SetSpeed(0, 0);

    /* 初始化左电机PID - 速度控制 */
    PID_Init(&motor_left_speed_pid, MOTOR_KP, MOTOR_KI, MOTOR_KD, MOTOR_DT);
    PID_SetOutputLimits(&motor_left_speed_pid, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT);
    PID_SetIntegralLimits(&motor_left_speed_pid, MOTOR_MIN_INTEGRAL, MOTOR_MAX_INTEGRAL);

    /* 初始化右电机PID - 速度控制 */
    PID_Init(&motor_right_speed_pid, MOTOR_KP, MOTOR_KI, MOTOR_KD, MOTOR_DT);
    PID_SetOutputLimits(&motor_right_speed_pid, MOTOR_MIN_OUTPUT, MOTOR_MAX_OUTPUT);
    PID_SetIntegralLimits(&motor_right_speed_pid, MOTOR_MIN_INTEGRAL, MOTOR_MAX_INTEGRAL);

    /* 初始化左电机PID - 位置控制 */ //? 停车可能有用 2333
    PID_Init(&motor_left_position_pid, 1.6f, 1.0f, 0.005f, 0.03f);
    PID_SetOutputLimits(&motor_left_position_pid, -100.0f, 100.0f);
    PID_SetIntegralLimits(&motor_left_position_pid, -60.0f, 60.0f);

    /* 初始化右电机PID - 位置控制 */ //? 停车可能有用 2333
    PID_Init(&motor_right_position_pid, 1.6f, 1.0f, 0.005f, 0.03f);
    PID_SetOutputLimits(&motor_right_position_pid, -100.0f, 100.0f);
    PID_SetIntegralLimits(&motor_right_position_pid, -60.0f, 60.0f);
}

/*
 * @brief 设置电机速度
 * @param left_pwm: 左电机速度
 * @param right_pwm: 右电机速度
 * -100~100表示速度范围(float)
 * 该函数会先对速度进行限制，然后设置PWM占空比以控制电机速度。
 */
void Motor_SetSpeed(float left_pwm, float right_pwm)
{
    // 先进行速度限制
    if (left_pwm > MOTOR_MAX_PWM)
    {
        left_pwm = MOTOR_MAX_PWM;
    }
    else if (left_pwm < MOTOR_MIN_PWM)
    {
        left_pwm = MOTOR_MIN_PWM;
    }

    if (right_pwm > MOTOR_MAX_PWM)
    {
        right_pwm = MOTOR_MAX_PWM;
    }
    else if (right_pwm < MOTOR_MIN_PWM)
    {
        right_pwm = MOTOR_MIN_PWM;
    }

    if (left_pwm < 0)
    {
        left_pwm = -left_pwm;                                                   // 取绝对值
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN1, GPIO_PIN_RESET); // 左电机正转引脚拉低
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN2, GPIO_PIN_SET);   // 左电机反转引脚拉高
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Calculate_PWM_Duty(left_pwm));
    }
    else if (left_pwm > 0)
    {
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN1, GPIO_PIN_SET);   // 左电机正转引脚拉高
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN2, GPIO_PIN_RESET); // 左电机反转引脚拉低
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, Calculate_PWM_Duty(left_pwm));
    }
    else
    {
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN1, GPIO_PIN_RESET); // 左电机正转引脚拉低
        HAL_GPIO_WritePin(GPIO_LEFT_PORT, GPIO_LEFT_MOTOR_IN2, GPIO_PIN_RESET); // 左电机反转引脚拉低
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_2, 0);                        // 停止左电机
    }

    if (right_pwm < 0)
    {
        right_pwm = -right_pwm;                                                   // 取绝对值
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN1, GPIO_PIN_RESET); // 右电机正转引脚拉低
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN2, GPIO_PIN_SET);   // 右电机反转引脚拉高
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Calculate_PWM_Duty(right_pwm));
    }
    else if (right_pwm > 0)
    {
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN1, GPIO_PIN_SET);   // 右电机正转引脚拉高
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN2, GPIO_PIN_RESET); // 右电机反转引脚拉低
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, Calculate_PWM_Duty(right_pwm));
    }
    else
    {
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN1, GPIO_PIN_RESET); // 右电机正转引脚拉低
        HAL_GPIO_WritePin(GPIO_RIGHT_PORT, GPIO_RIGHT_MOTOR_IN2, GPIO_PIN_RESET); // 右电机反转引脚拉低
        __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);                          // 停止右电机
    }
}

void Get_Motor_Info(void)
{
    static uint16_t preLeftCount = 0;
    static uint16_t preRightCount = 0;
    static uint32_t pre_time = 0;
    // static float pre_left_speed = 0.0f;
    // static float pre_right_speed = 0.0f;
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

    // 计算差值
    int16_t left_diff = (int32_t)((left_encoder_count - preLeftCount));
    int16_t right_diff = (int32_t)(right_encoder_count - preRightCount);

    // 累加总计数，这样可以追踪多圈转动
    total_left_count += left_diff;
    total_right_count += right_diff;

    float time_diff = (current_time - pre_time) / 1000.0f; // 转换为秒

    if (time_diff > 0)
    {
        // 计算左电机数据
        motor_left_data.angle = (float)fabs(fmodf((float)total_left_count, MOTOR_PULSE_PER_REVOLUTION) * 360.0f / MOTOR_PULSE_PER_REVOLUTION);
        // motor_left_data.angle = (float)total_left_count * 360.0f / MOTOR_PULSE_PER_REVOLUTION;
        float left_angle_diff = (float)left_diff * 360.0f / MOTOR_PULSE_PER_REVOLUTION;
        motor_left_data.speed = left_angle_diff / time_diff;                                 // 度/秒
        //// motor_left_data.acceleration = (motor_left_data.speed - pre_left_speed) / time_diff; // 度/秒²

        // 计算右电机数据
        motor_right_data.angle = (float)fabs(fmodf((float)total_right_count, MOTOR_PULSE_PER_REVOLUTION) * 360.0f / MOTOR_PULSE_PER_REVOLUTION);
        float right_angle_diff = (float)right_diff * 360.0f / MOTOR_PULSE_PER_REVOLUTION;
        motor_right_data.speed = right_angle_diff / time_diff;                                  // 度/秒
        //// motor_right_data.acceleration = (motor_right_data.speed - pre_right_speed) / time_diff; // 度/秒²

        LPF1_Update(&motor_left_data.filtered_speed, &motor_left_data.speed, 0.01f, &lpf1_left_speed);
        // LPF1_Update(&motor_left_data.filtered_acceleration, &motor_left_data.acceleration, 0.01f, &lpf1_left_acceleration);
        LPF1_Update(&motor_right_data.filtered_speed, &motor_right_data.speed, 0.01f, &lpf1_right_speed);
        // LPF1_Update(&motor_right_data.filtered_acceleration, &motor_right_data.acceleration, 0.01f, &lpf1_right_acceleration);

        // 更新历史数据
        // pre_left_speed = motor_left_data.speed;
        // pre_right_speed = motor_right_data.speed;
    }


    preLeftCount = left_encoder_count;
    preRightCount = right_encoder_count;
    pre_time = current_time;
}

void Motor_Stop(void)
{
    Motor_SetSpeed(0, 0);
}

void PID_Motor_Controllers_Speed_Updater(float target_left_speed, float target_right_speed)
{
    // 更新左电机速度PID
    PID_SetTarget(&motor_left_speed_pid, -target_left_speed);
    float left_output = PID_Compute(&motor_left_speed_pid, motor_left_data.filtered_speed);

    // 更新右电机速度PID
    PID_SetTarget(&motor_right_speed_pid, target_right_speed);
    float right_output = PID_Compute(&motor_right_speed_pid, motor_right_data.filtered_speed);

    Motor_SetSpeed(left_output, right_output);
}

void PID_Motor_Controllers_Position_Updater(float target_left_position, float target_right_position)
{
    // 更新左电机位置PID
    PID_SetTarget(&motor_left_position_pid, target_left_position);
    float left_output = PID_Compute(&motor_left_position_pid, motor_left_data.angle);

    // 更新右电机位置PID
    PID_SetTarget(&motor_right_position_pid, target_right_position);
    float right_output = PID_Compute(&motor_right_position_pid, motor_right_data.angle);

    // 设置电机速度
    PID_Motor_Controllers_Speed_Updater(left_output, right_output);
}
