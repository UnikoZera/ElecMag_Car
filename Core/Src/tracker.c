/*
 * tracker.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "tracker.h"
#include "uart_vofa.h"

#define MOTOR_MAX_OUTPUT 1000.0f   // 电机最大输出
#define MOTOR_MIN_OUTPUT -1000.0f  // 电机最小输出
#define MOTOR_MAX_INTEGRAL 200.0f  // 电机积分最大值
#define MOTOR_MIN_INTEGRAL -200.0f // 电机积分最小值

#define MOTOR_KP 1.0f  // 左右电机速度PID比例系数
#define MOTOR_KI 23.3f  // 左右电机速度PID积分系数
#define MOTOR_KD 0.018f // 左右电机速度PID微分系数
#define MOTOR_DT 0.01f // PID采样周期，单位为秒

#define ADC_CENTER Vref / 2.0f

PID_TypeDef motor_left_pid;     // 左电机速度PID
PID_TypeDef motor_right_pid;    // 右电机速度PID

PID_TypeDef direction_pid;      // 方向控制PID

typedef enum
{
    TRACK_NORMAL,               // 正常跟踪(包括直线跟踪和曲线跟踪)
    TRACK_SHARP_TURN_LEFT,      // 急转弯(如90度转弯)
    TRACK_SHARP_TURN_RIGHT,     // 急转弯(如90度转弯)
    TRACK_AROUND,               // 环岛情况
    TRACK_IDLE,                 // 空闲状态(不跟踪)
    TRACK_AVOID_OBSTACLE,       // 避障跟踪
    TRACK_LOST,                 // 丢失导线
} TrackState_t;

TrackState_t current_track_state = TRACK_IDLE;
TrackState_t previous_track_state = TRACK_IDLE;

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
    float left_output = PID_Compute(&motor_left_pid, motor_left_data.filtered_speed);

    // 更新右电机速度PID
    PID_SetTarget(&motor_right_pid, target_right_speed);
    float right_output = PID_Compute(&motor_right_pid, motor_right_data.filtered_speed);

    // 设置电机速度
    float debug_data1[3];
    debug_data1[0] = left_output;
    debug_data1[1] = motor_left_data.filtered_speed;
    debug_data1[2] = motor_left_data.angle; // 角度数据
    VOFA_SendFloat(debug_data1, 3); // 发送调试数据

    Motor_SetSpeed((int)left_output, (int)right_output);
}

#pragma region 状态机函数
void State_Machine(void)
{
}

void Normal_Track(void) // 标准的巡线
{
    float sum_adc = 0.0f;
    for (int i = 0; i < 4; i++)
    {
        sum_adc += adc_data[i];
    }

    float weighted_diff = -4.0f * adc_data[0] - 1.0f * adc_data[1] +
                          1.0f * adc_data[2] + 4.0f * adc_data[3]; // 权重根据实际情况调整

    float deviation = weighted_diff / sum_adc;

    PID_SetTarget(&direction_pid, 0.0f); // 目标是零偏差
    float direction_correction = PID_Compute(&direction_pid, deviation);

    float base_speed = 300.0f; // 根据实际情况调整

    float target_left_speed = base_speed - direction_correction * 1.0f; // 比例大小也需要调整
    float target_right_speed = base_speed + direction_correction * 1.0f;

    PID_Motor_Controllers_Updater(target_left_speed, target_right_speed);
}

void Sharp_Turn_Left_Track(void)
{
    float turn_speed = 200.0f; // 降低转弯速度

    // 左转
    float target_left_speed = -turn_speed * 0.5f; // 左轮减速或反转
    float target_right_speed = turn_speed;        // 右轮正常
    PID_Motor_Controllers_Updater(target_left_speed, target_right_speed);
}

void Sharp_Turn_Right_Track(void)
{
    float turn_speed = 200.0f; // 降低转弯速度

    // 右转
    float target_left_speed = turn_speed;         // 左轮正常
    float target_right_speed = -turn_speed * 0.5f; // 右轮减速或反转
    PID_Motor_Controllers_Updater(target_left_speed, target_right_speed);
}

void Lost_Track(void)
{
    switch (previous_track_state)
    {
    case TRACK_NORMAL:
        Motor_Stop(); //byd这也能跑丢?
        break;
    case TRACK_SHARP_TURN_LEFT:
        Sharp_Turn_Left_Track();
        break;
    case TRACK_SHARP_TURN_RIGHT:
        Sharp_Turn_Right_Track();
        break;
    case TRACK_AROUND:

        break;

    case TRACK_AVOID_OBSTACLE:
        
        break;
    case TRACK_IDLE:
        Motor_Stop();
        break;
    default:
        break;
    }
}

#pragma endregion

void Tracker_Compute(void)
{
    State_Machine(); // 更新跟踪状态

    switch (current_track_state)
    {
    case TRACK_IDLE:
        Motor_Stop();
        break;
    case TRACK_NORMAL:
        Normal_Track();
        break;
    case TRACK_SHARP_TURN_LEFT:
        Sharp_Turn_Left_Track();
        break;
    case TRACK_SHARP_TURN_RIGHT:
        Sharp_Turn_Right_Track();
        break;
    case TRACK_LOST:
        Lost_Track();
        break;
    case TRACK_AROUND:
        // 环岛跟踪逻辑
        // 这里可以根据实际情况实现环岛跟踪逻辑
        break;
    case TRACK_AVOID_OBSTACLE:
        // 避障跟踪逻辑
        // 这里可以根据实际情况实现避障跟踪逻辑
        break;
    default:
        Motor_Stop();
        break;
    }
}
