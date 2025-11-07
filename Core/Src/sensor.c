/*
 * sensor.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "sensor.h"
#define LOWPASS_FILTER_ALPHA 0.3f // 低通滤波系数

MPU6050_t gyro_data_raw;
uint16_t raw_adc_data[5]; // 通道1~5

uint16_t adc_data[5]; //直接就是,左边到右边
float gyro_data[3]; // 陀螺仪数据，顺序为Gx, Gy, Gz
float total_angle_z = 0.0f; // Z轴总角度变化

typedef struct
{
    float x;                   // 状态估计值（过滤后的DPS值）
    float P;                   // 估计误差协方差
    float Q;                   // 过程噪声协方差（系统噪声）
    float R;                   // 测量噪声协方差（传感器噪声）
    float K;                   // 卡尔曼增益
    unsigned char initialized; // 初始化标志
} DPS_KalmanFilter;

DPS_KalmanFilter gyro_x_filter, gyro_y_filter, gyro_z_filter;


#pragma region  传感器滤波函数
// 低通滤波函数for ADC数据
void Lowpass_Filter(uint16_t *dst, uint16_t *input, float alpha)
{
    static uint16_t prev_data[5] = {0}; // 上一次的输出数据

    for (int i = 0; i < 5; i++)
    {
        dst[i] = alpha * input[i] + (1 - alpha) * prev_data[i];
        prev_data[i] = dst[i];
    }
}

void Kalman_Init(DPS_KalmanFilter *filter, float process_noise, float measurement_noise)
{
    filter->x = 0.0f;              // 初始状态估计为0
    filter->P = 1.0f;              // 初始估计误差协方差较大，表示不确定性
    filter->Q = process_noise;     // 过程噪声协方差 (推荐值: 0.001 - 0.01)
    filter->R = measurement_noise; // 测量噪声协方差 (推荐值: 0.01 - 0.1)
    filter->K = 0.0f;              // 初始卡尔曼增益
    filter->initialized = 0;       // 未初始化状态
}

float Kalman_Update(DPS_KalmanFilter *filter, float measurement)
{
    if (!filter->initialized)
    {
        filter->x = measurement;
        filter->initialized = 1;
        return filter->x;
    }

    // 预测步骤 - 状态预测(简化模型假设状态不变)
    // 更新误差协方差
    filter->P = filter->P + filter->Q;

    // 计算卡尔曼增益
    filter->K = filter->P / (filter->P + filter->R);

    // 用测量值更新状态估计
    filter->x = filter->x + filter->K * (measurement - filter->x);

    // 更新误差协方差
    filter->P = (1.0f - filter->K) * filter->P;

    return filter->x;
}

void Kalman_Reset(DPS_KalmanFilter *filter)
{
    filter->x = 0.0f;
    filter->P = 1.0f;
    filter->K = 0.0f;
    filter->initialized = 0;
}

#pragma endregion

void Sensor_Init(void)
{
    // 初始化ADC
    HAL_ADCEx_Calibration_Start(&hadc1); // 校准ADC, 但是貌似没啥卵用233.
    HAL_Delay(10); // 等待校准完成
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)raw_adc_data, sizeof(raw_adc_data)/sizeof(uint16_t));
    // 初始化陀螺仪
    MPU6050_Init();
    MPU6050_Calibrate(&gyro_data_raw);
    HAL_Delay(50); // 等待校准完成
    Kalman_Init(&gyro_x_filter, 0.01f, 0.01f); // 初始化X轴卡尔曼滤波器
    Kalman_Init(&gyro_y_filter, 0.01f, 0.01f); // 初始化Y轴卡尔曼滤波器
    Kalman_Init(&gyro_z_filter, 0.01f, 0.01f); // 初始化Z轴卡尔曼滤波器
    // 初始化超声波传感器
    Ultrasonic_Init();
}

void Sensor_Updater(void)
{
    if (HAL_GetTick() % 2 == 0) // 500Hz更新陀螺仪数据
    {
        MPU6050_Read_All(&gyro_data_raw);
        gyro_data[0] = Kalman_Update(&gyro_x_filter, gyro_data_raw.Gx);
        gyro_data[1] = Kalman_Update(&gyro_y_filter, gyro_data_raw.Gy);
        gyro_data[2] = Kalman_Update(&gyro_z_filter, gyro_data_raw.Gz);

        total_angle_z += gyro_data[2] * 0.002f * 1; // 可能需要魔法数调整
    }

    Lowpass_Filter(adc_data, raw_adc_data, LOWPASS_FILTER_ALPHA);

    // 超声波传感器更新距离值在中断中完成，无需在此处处理!
    // distance 变量在中断处理程序中更新
    // 为什么你还在看注释
    // 这里什么都没有了！
    // 真的！


















    // 你在找这个嘛？
    // adc 数据已经在DMA中自动更新
    // 直接使用 adc_data 数组 {v1, v2, v3, v4, v5}
}
