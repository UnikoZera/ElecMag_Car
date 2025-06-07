/*
 * sensor.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "sensor.h"

MPU6050_t gyro_data;
uint16_t raw_adc_data[5]; // 通道1~4,最后一个是reference电压
float adc_data[4];

float Vref = 1.2f; // 参考电压

void Sensor_Init(void)
{
    // 初始化陀螺仪
    MPU6050_Init();
    MPU6050_Calibrate(&gyro_data);

    //初始化ADC
    HAL_ADCEx_Calibration_Start(&hadc1);
    HAL_ADC_Start_DMA(&hadc1, (uint32_t*)raw_adc_data, sizeof(raw_adc_data)/sizeof(uint16_t));

}

void Sensor_Updater(void)
{
    MPU6050_Read_All(&gyro_data);

    // 更新ADC数据
    Vref = 1.2 * (4095.0f / (float)raw_adc_data[4]);
    for (int i = 0; i < 4; i++)
    {
        adc_data[i] = (float)raw_adc_data[i] * Vref / 4095.0f;
    }
}