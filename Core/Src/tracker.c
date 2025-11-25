/*
 * tracker.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */


#include "tracker.h"

#define MOTOR_LOST_THRESHOLD 1000.0f    // 电机丢线阈值，单位为ADC值
#define ULTRASONIC_THRESHOLD 30.0f      // 超声波避障阈值，单位为厘米
#define STRAIGHT_TRACK_THRESHOLD 100.0f // 直线跟踪阈值，单位为ADC值(在这个阈值范围内给大油门!)

const float adc_weight[5] = { -2.0f, -1.0f, 0.0f, 1.0f, 2.0f }; // 五个传感器的权重

PID_TypeDef direction_pid; // 方向控制PID

int tender = 1;                     // -1为向左,1为向右
uint16_t counter_crossroads = 0;    // 停车需要用到的变量
float motor_basic_speed = 300.0f;   // 基础速度 其实可以动态调整的，但是先这样吧 // TODO: 基础速度调整机制
bool enable_state_machine = true;   // 是否启用状态机
float weighted_sum = 0;             // 加权和
float total_sum = 0;                // 电感总和
float deviation = 0;                // 偏差值

typedef enum
{
    TRACK_NORMAL,         // 正常跟踪(包括直线跟踪和曲线跟踪)
    TRACK_AROUND,         // 环岛情况
    TRACK_AVOID_OBSTACLE, // 避障跟踪
    TRACK_IDLE,           // 空闲状态(停车)
    TRACK_LOST,           // 丢失导线
    TRACK_STRAIGHT        // 直线跟踪
} TrackState_t;

TrackState_t current_track_state = TRACK_IDLE;   // 当前跟踪状态
TrackState_t previous_track_state = TRACK_IDLE;  // 上一个跟踪状态

void Tracker_Init(void)
{
    /* 初始化方向控制PID */ // 我能不能活就看你了
    PID_Init(&direction_pid, 2.0f, 0.05f, 0.1f, 0.01f);
    PID_SetOutputLimits(&direction_pid, 3000.0f, 3000.0f);  // TODO: 方向PID输出限制
    PID_SetIntegralLimits(&direction_pid, -100.0f, 100.0f); // TODO: 积分限制

    /* 看看还有什么pid要用的 */

}

#pragma region State Machine and Track Methods
void State_Machine(void)
{
    previous_track_state = current_track_state; // 更新上一个状态

    if (distance < ULTRASONIC_THRESHOLD) // 避障优先级最高,因为这会导致丢线
    {
        current_track_state = TRACK_AVOID_OBSTACLE;
        return;
    }

    if (adc_data[0] < MOTOR_LOST_THRESHOLD && adc_data[1] < MOTOR_LOST_THRESHOLD && adc_data[2] < MOTOR_LOST_THRESHOLD && adc_data[3] < MOTOR_LOST_THRESHOLD && adc_data[4] < MOTOR_LOST_THRESHOLD)
    {
        current_track_state = TRACK_LOST; // i hate this state.
        return;
    }
    
    // 这里放置是否进入环岛的判断 // TODO: 环岛判断条件
    // if (condition)
    // {
    //     current_track_state = TRACK_AROUND;
    //     return;
    // }

    // 判断是否需要停车 // TODO: 停车判断条件
    // if (condition)
    // {
    //     current_track_state = TRACK_IDLE;
    //     return;
    // }

    if (fabsf(deviation) < STRAIGHT_TRACK_THRESHOLD)
    {
        current_track_state = TRACK_STRAIGHT; // 直线跟踪
        return;
    }
    else if (weighted_sum > 0)
    {
        tender = 1; // 向右
        current_track_state = TRACK_NORMAL;
        return;
    }
    else if (weighted_sum < 0)
    {
        tender = -1; // 向左
        current_track_state = TRACK_NORMAL;
        return;
    }
}

void Normal_Track(void) // 巡线跟踪
{
    PID_SetTarget(&direction_pid, 0.0f); // 目标是零偏差大师！
    float direction_correction = PID_Compute(&direction_pid, deviation);

    float target_left_speed = motor_basic_speed - direction_correction;
    float target_right_speed = motor_basic_speed + direction_correction;

    PID_Motor_Controllers_Speed_Updater(target_left_speed, target_right_speed);
}

 //! 因为我认为normalTrack应该可以对付直角转弯(所以sharp_turn被cancelled了)
//// void Sharp_Turn_Track(int dir)
//// {
////     float turn_speed = 200.0f;
////     float target_speed = turn_speed * 0.5f;
////     switch (dir)
////     {
////     case 1:
////         PID_Motor_Controllers_Speed_Updater(turn_speed, target_speed);
////         break;
////     case -1:
////         PID_Motor_Controllers_Speed_Updater(target_speed, turn_speed);
////         break;
////     default:
////         Motor_Stop();
////         break;
////     }
//// }

void Avoid_Obstacle_Track(int dir) // 避障跟踪
{
    switch (dir)
    {
        case -1:
        {

            break;
        }
        case 1:
        {

            break;
        }
    }

    // if (condition)
    // enable_state_machine = true; // 启用状态机
}

void Around_Track(int dir) // 环岛跟踪 // TODO: 环岛跟踪调参
{
    switch (dir)
    {
        case 1:
        {
            PID_Motor_Controllers_Speed_Updater(1500.0f, 1000.0f); // 右转
            Normal_Track(); // 继续正常跟踪
            break;
        }
        case -1:
        {
            PID_Motor_Controllers_Speed_Updater(1000.0f, 1500.0f); // 左转
            Normal_Track(); // 继续正常跟踪
            break;
        }
    }
}

void Lost_Track(void) // 丢线跟踪
{
    switch (previous_track_state)
    {
    case TRACK_NORMAL:
    {
        if (tender == 1)
        {
            PID_Motor_Controllers_Speed_Updater(0.0f, 500.0f); // 向右转找线 // TODO: 调参
        }
        else
        {
            PID_Motor_Controllers_Speed_Updater(500.0f, 0.0f); // 向左转找线
        }
        break;
    }
    case TRACK_AROUND:
    {
        Motor_Stop();
        break;
    }
    // case TRACK_AVOID_OBSTACLE:
    //     break; //! 这里直接采用固定模块好了
    case TRACK_IDLE:
    {
        Motor_Stop();
        break;
    }
    default:
        break;
    }
}

void Straight_Track(void) // 直线跟踪
{
    PID_Motor_Controllers_Speed_Updater(motor_basic_speed + 100.0f, motor_basic_speed + 100.0f); // TODO: 速度参数
}

void Stop_Track(void) // 停车跟踪
{
    // Motor_Stop();
    float target_left_position = motor_left_data.angle;
    float target_right_position = motor_right_data.angle;
    PID_Motor_Controllers_Position_Updater(target_left_position, target_right_position); // 有闭环你用不用? looking my eyes!!
}

#pragma endregion

void Tracker_Compute(void)
{
    weighted_sum = (float)adc_data[0] * adc_weight[0] +
                   (float)adc_data[1] * adc_weight[1] +
                   (float)adc_data[2] * adc_weight[2] +
                   (float)adc_data[3] * adc_weight[3] +
                   (float)adc_data[4] * adc_weight[4];

    total_sum = (float)adc_data[0] +
                (float)adc_data[1] +
                (float)adc_data[2] +
                (float)adc_data[3] +
                (float)adc_data[4];

    deviation = weighted_sum / 5.0f; // 计算偏差值(平均值)

    if (enable_state_machine)
        State_Machine(); // 更新跟踪状态

    switch (current_track_state)
    {
        case TRACK_STRAIGHT:
        {
            enable_state_machine = true;
            Straight_Track();
            break;
        }
        case TRACK_IDLE:
        {
            enable_state_machine = true;
            Stop_Track();
            break;
        }
        case TRACK_NORMAL:
        {
            enable_state_machine = true;
            Normal_Track();
            break;
        }
        case TRACK_LOST:
        {
            enable_state_machine = true;
            Lost_Track();
            break;
        }
        case TRACK_AROUND:
        {
            enable_state_machine = false;
            Around_Track(tender); 
            break;
        }
        case TRACK_AVOID_OBSTACLE:
        {
            enable_state_machine = false;
            Avoid_Obstacle_Track(tender);
            break;
        }
        default:
        {
            enable_state_machine = true;
            Motor_Stop();
            break;
        }
    }
}
