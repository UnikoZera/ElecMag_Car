/*
 * gyro.h
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#ifndef INC_GYRO_H_
#define INC_GYRO_H_

#include "main.h"
#include "i2c.h"
#include <stdint.h>

//模块的A0引脚接GND，IIC的7位地址为0x68，若接到VCC，需要改为0x69
#define MPU6050_ADDR  (0x68<<1)      //MPU6050器件读地址

// MPU6050寄存器地址
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A // 配置寄存器（低通滤波器）
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75

// 配置宏定义
#define MPU6050_SAMPLE_RATE_HZ 500   // 采样率 (Hz)
#define MPU6050_GYRO_SCALE 0         // 陀螺仪量程: 0=±250°/s, 1=±500°/s, 2=±1000°/s, 3=±2000°/s
#define MPU6050_ACCEL_SCALE 0        // 加速度计量程: 0=±2g, 1=±4g, 2=±8g, 3=±16g
#define MPU6050_DLPF_CFG 6           // 数字低通滤波器: 0=260Hz, 1=184Hz, 2=94Hz, 3=44Hz, 4=21Hz, 5=10Hz, 6=5Hz
#define MPU6050_TIMEOUT_MS 1000      // I2C超时时间(ms)
#define MPU6050_STARTUP_DELAY_MS 100 // 启动延时(ms)
#define MPU6050_RESET_DELAY_MS 100   // 复位延时(ms)

// 根据配置计算的值
#define MPU6050_SAMPLE_RATE_DIV ((1000 / MPU6050_SAMPLE_RATE_HZ) - 1)

// 陀螺仪灵敏度 (LSB/°/s)
#if MPU6050_GYRO_SCALE == 0
#define MPU6050_GYRO_SENSITIVITY 131.0f
#elif MPU6050_GYRO_SCALE == 1
#define MPU6050_GYRO_SENSITIVITY 65.5f
#elif MPU6050_GYRO_SCALE == 2
#define MPU6050_GYRO_SENSITIVITY 32.8f
#else
#define MPU6050_GYRO_SENSITIVITY 16.4f
#endif

// 加速度计灵敏度 (LSB/g)
#if MPU6050_ACCEL_SCALE == 0
#define MPU6050_ACCEL_SENSITIVITY 16384.0f
#elif MPU6050_ACCEL_SCALE == 1
#define MPU6050_ACCEL_SENSITIVITY 8192.0f
#elif MPU6050_ACCEL_SCALE == 2
#define MPU6050_ACCEL_SENSITIVITY 4096.0f
#else
#define MPU6050_ACCEL_SENSITIVITY 2048.0f
#endif

// 电源管理配置值
#define MPU6050_PWR_MGMT_1_RESET 0x80  // 复位设备
#define MPU6050_PWR_MGMT_1_WAKEUP 0x00 // 唤醒设备，使用内部振荡器

// MPU6050数据结构
typedef struct
{
    int16_t Accel_X_RAW; //X轴加速度原始数据
    int16_t Accel_Y_RAW; //Y轴加速度原始数据
    int16_t Accel_Z_RAW; //Z轴加速度原始数据
    float Ax; // X轴加速度 (g)
    float Ay; // Y轴加速度 (g)
    float Az; // Z轴加速度 (g) (包含重力)

    int16_t Gyro_X_RAW; //X轴陀螺仪原始数据
    int16_t Gyro_Y_RAW; //Y轴陀螺仪原始数据
    int16_t Gyro_Z_RAW; //Z轴陀螺仪原始数据
    float Gx; // X轴陀螺仪 (°/s)
    float Gy; // Y轴陀螺仪 (°/s)
    float Gz; // Z轴陀螺仪 (°/s)

    float Temperature; // 温度 (°C)

    // 校准偏移量
    int16_t Gyro_X_Offset;
    int16_t Gyro_Y_Offset;
    int16_t Gyro_Z_Offset;
    int16_t Accel_X_Offset;
    int16_t Accel_Y_Offset;
    int16_t Accel_Z_Offset;
} MPU6050_t;

uint8_t MPU6050_Init(void);
uint8_t MPU6050_Test_Connection(void);
void MPU6050_Calibrate(MPU6050_t *DataStruct);
void MPU6050_Read_Accel(MPU6050_t *DataStruct);
void MPU6050_Read_Gyro(MPU6050_t *DataStruct);
void MPU6050_Read_Temp(MPU6050_t *DataStruct);
void MPU6050_Read_All(MPU6050_t *DataStruct);

uint8_t MPU6050_Write_Reg(uint8_t reg, uint8_t data);
uint8_t MPU6050_Read_Reg(uint8_t reg, uint8_t *data);
uint8_t MPU6050_Read_Multi_Reg(uint8_t reg, uint8_t *data, uint8_t length);

#endif /* INC_GYRO_H_ */
