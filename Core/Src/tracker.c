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
#define MOTOR_MAX_INTEGRAL 100.0f  // 电机积分最大值
#define MOTOR_MIN_INTEGRAL -60.0f // 电机积分最小值

#define MOTOR_KP 1.0f   // 左右电机速度PID比例系数
#define MOTOR_KI 21.3f  // 左右电机速度PID积分系数
#define MOTOR_KD 0.023f // 左右电机速度PID微分系数
#define MOTOR_DT 0.01f  // PID采样周期，单位为秒

#define MOTOR_LOST_THRESHOLD 1000.0f // 电机丢线阈值，单位为ADC值
#define ULTRASONIC_THRESHOLD 30.0f // 超声波避障阈值，单位为厘米

// adc权重比
#define ADC_RATIO_INSIDE 1.0f
#define ADC_RATIO_OUTSIDE 4.0f

PID_TypeDef motor_left_pid, motor_right_pid;  // 电机速度PID
PID_TypeDef direction_pid; // 方向控制PID

int tender = 1; //-1为向左,1为向右
unsigned char counter_crossroads = 0;
float motor_basic_speed = 300.0f; // 基础速度
bool enable_state_machine = true; // 是否启用状态机

typedef enum
{
    TRACK_NORMAL,         // 正常跟踪(包括直线跟踪和曲线跟踪)
    TRACK_AROUND,         // 环岛情况
    TRACK_AVOID_OBSTACLE, // 避障跟踪
    TRACK_IDLE,           // 空闲状态(不跟踪)
    TRACK_LOST,           // 丢失导线
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
    PID_SetOutputLimits(&direction_pid, 3000.0f, 3000.0f);
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
    // float debug_data1[3];
    // debug_data1[0] = left_output;
    // debug_data1[1] = motor_left_data.filtered_speed;
    // debug_data1[2] = motor_left_data.filtered_acceleration; // 角度数据
    // VOFA_SendFloat(debug_data1, 3);         // 发送调试数据

    Motor_SetSpeed((int)left_output, (int)right_output);
}

#pragma region 状态机函数
void State_Machine(void)
{
    // 这里选择是否丢线
    if (adc_data[0] < MOTOR_LOST_THRESHOLD && adc_data[1] < MOTOR_LOST_THRESHOLD && adc_data[2] < MOTOR_LOST_THRESHOLD && adc_data[3] < MOTOR_LOST_THRESHOLD)
    {
        current_track_state = TRACK_LOST;
        return; // 丢线状态不需要更新adc
    }
    
    // 这里放置是否进入环岛的判断
    // if (condition)
    // {
    //     current_track_state = TRACK_AROUND;
    //     return;
    // }

    // 这里是超声波判断是否躲避障碍物(优先级大于adc)
    // if (Ultrasonic_GetDistance() < ULTRASONIC_THRESHOLD)
    // {
    //     current_track_state = TRACK_AVOID_OBSTACLE;
    //     return; // 避障状态不需要更新adc
    // }

    // 判断是否需要停车
    //  
    //

    // 这里是adc的数据来分配具体状态
    if ((ADC_RATIO_OUTSIDE * (adc_data[3] - adc_data[0]) + (ADC_RATIO_INSIDE * (adc_data[2] - adc_data[1]))) > 0)
    {
        tender = 1; // 向右
        current_track_state = TRACK_NORMAL;
    }
    else if ((-ADC_RATIO_OUTSIDE * adc_data[0] - ADC_RATIO_INSIDE * adc_data[1] + ADC_RATIO_INSIDE * adc_data[2] + ADC_RATIO_OUTSIDE * adc_data[3]) < 0)
    {
        tender = -1; // 向左
        current_track_state = TRACK_NORMAL;
    }
    else
    {
        //可以考虑加快速度
        current_track_state = TRACK_NORMAL;
    }

}

void Normal_Track(void) // 标准的巡线
{
    float weighted_diff = ADC_RATIO_OUTSIDE * adc_data[0] - ADC_RATIO_INSIDE * adc_data[1] +
                          ADC_RATIO_INSIDE * adc_data[2] + ADC_RATIO_OUTSIDE * adc_data[3];

    float deviation = weighted_diff / 4; // 计算平均偏差值

    PID_SetTarget(&direction_pid, 0.0f); // 目标是零偏差大师！
    float direction_correction = PID_Compute(&direction_pid, deviation) * 1.0f; // 计算方向修正值

    float target_left_speed = motor_basic_speed - direction_correction;
    float target_right_speed = motor_basic_speed + direction_correction;

    PID_Motor_Controllers_Updater(target_left_speed, target_right_speed);
}

void Sharp_Turn_Track(int dir) //后退着 //? 因为我认为normalTrack应该可以对付直角转弯
{
    float turn_speed = 200.0f;
    float target_speed = turn_speed * 0.5f;

    switch (dir)
    {
    case 1:
        PID_Motor_Controllers_Updater(turn_speed, target_speed);
        break;
    case -1:
        PID_Motor_Controllers_Updater(target_speed, turn_speed);
        break;
    default:
        Motor_Stop();
        break;
    }
}

void Avoid_Obstacle_Track(int dir)
{
    switch (dir)
    {
    case -1:
    {
        //可以用个漂亮的sin函数和delay来一口气完成转弯
        enable_state_machine = true; // 启用状态机
    }
        break;
    case 1:
    {
        enable_state_machine = true; // 启用状态机
        break;
    }
    default:
        Motor_Stop(); // 停止电机
        break;
    }

    // if (condition)
    // enable_state_machine = true; // 启用状态机
}

void Around_Track(int dir)
{
    switch (dir)
    {
    case 1:
    {
        PID_Motor_Controllers_Updater(1500.0f, 1000.0f); // 右转
        Normal_Track(); // 继续正常跟踪
        break;
    }
    case -1:
    {
        PID_Motor_Controllers_Updater(1000.0f, 1500.0f); // 左转
        Normal_Track(); // 继续正常跟踪
        break;
    }
    default:
        Motor_Stop();
        break;
    }
}

void Lost_Track(void)
{
    switch (previous_track_state)
    {
    case TRACK_NORMAL:
        Sharp_Turn_Track(tender);
        break;
    case TRACK_AROUND:
        Motor_Stop();
        break;
    // case TRACK_AVOID_OBSTACLE:
    //     break; //! 这里直接采用固定模块好了
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
    if (enable_state_machine)
        State_Machine(); // 更新跟踪状态

    switch (current_track_state)
    {
    case TRACK_IDLE:
        enable_state_machine = true;
        Motor_Stop();
        break;
    case TRACK_NORMAL:
        enable_state_machine = true;
        Normal_Track();
        break;
    case TRACK_LOST:
        enable_state_machine = true;
        Lost_Track();
        break;
    case TRACK_AROUND:
        enable_state_machine = false;
        Around_Track(tender); 
        break;
    case TRACK_AVOID_OBSTACLE:
        enable_state_machine = false; // 禁用状态机
        Avoid_Obstacle_Track(tender); // 传入方向
        break;
    default:
        Motor_Stop();
        break;
    }

    // 更新上一个状态
    previous_track_state = current_track_state;
}
