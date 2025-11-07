/*
 * gyro.c
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#include "gyro.h"

/**
 * @brief 写入MPU6050寄存器
 * @param reg 寄存器地址
 * @param data 要写入的数据
 * @retval 0: 成功, 1: 失败
 */
uint8_t MPU6050_Write_Reg(uint8_t reg, uint8_t data)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Write(&hi2c2, MPU6050_ADDR, reg, 1, &data, 1, MPU6050_TIMEOUT_MS);
    return (status == HAL_OK) ? 0 : 1;
}

/**
 * @brief 读取MPU6050寄存器
 * @param reg 寄存器地址
 * @param data 读取数据存储指针
 * @retval 0: 成功, 1: 失败
 */
uint8_t MPU6050_Read_Reg(uint8_t reg, uint8_t *data)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg, 1, data, 1, MPU6050_TIMEOUT_MS);
    return (status == HAL_OK) ? 0 : 1;
}

/**
 * @brief 读取MPU6050多个寄存器
 * @param reg 起始寄存器地址
 * @param data 读取数据存储指针
 * @param length 读取长度
 * @retval 0: 成功, 1: 失败
 */
uint8_t MPU6050_Read_Multi_Reg(uint8_t reg, uint8_t *data, uint8_t length)
{
    HAL_StatusTypeDef status = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, reg, 1, data, length, MPU6050_TIMEOUT_MS);
    return (status == HAL_OK) ? 0 : 1;
}

/**
 * @brief 初始化MPU6050
 * @retval 0: 成功, 1: 失败
 */
uint8_t MPU6050_Init(void)
{
    uint8_t check;
    HAL_StatusTypeDef status;
    // 等待MPU6050稳定
    HAL_Delay(MPU6050_STARTUP_DELAY_MS);

    // 检查设备是否存在
    status = HAL_I2C_Mem_Read(&hi2c2, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, MPU6050_TIMEOUT_MS);

    if (status != HAL_OK || check != 0x68)
    {
        return 1; // 设备不存在或通信失败
    }

    // 复位MPU6050
    if (MPU6050_Write_Reg(PWR_MGMT_1_REG, MPU6050_PWR_MGMT_1_RESET) != 0)
    {
        return 2;
    }
    HAL_Delay(MPU6050_RESET_DELAY_MS); // 等待复位完成

    // 唤醒MPU6050，使用内部振荡器
    if (MPU6050_Write_Reg(PWR_MGMT_1_REG, MPU6050_PWR_MGMT_1_WAKEUP) != 0)
    {
        return 3;
    }
    HAL_Delay(10);

    // 配置数字低通滤波器
    if (MPU6050_Write_Reg(CONFIG_REG, MPU6050_DLPF_CFG) != 0)
    {
        return 4;
    }

    // 设置数据输出速率
    if (MPU6050_Write_Reg(SMPLRT_DIV_REG, MPU6050_SAMPLE_RATE_DIV) != 0)
    {
        return 5;
    }

    // 设置陀螺仪配置
    if (MPU6050_Write_Reg(GYRO_CONFIG_REG, MPU6050_GYRO_SCALE << 3) != 0)
    {
        return 6;
    }

    // 设置加速度计配置
    if (MPU6050_Write_Reg(ACCEL_CONFIG_REG, MPU6050_ACCEL_SCALE << 3) != 0)
    {
        return 7;
    }

    return 0;
}

/**
 * @brief 测试MPU6050连接和状态
 * @retval 0: 连接正常, 1: 连接失败
 */
uint8_t MPU6050_Test_Connection(void)
{
    uint8_t who_am_i;
    uint8_t pwr_mgmt;

    // 读取WHO_AM_I寄存器
    if (MPU6050_Read_Reg(WHO_AM_I_REG, &who_am_i) != 0)
    {
        return 1; // I2C通信失败
    }

    // 检查WHO_AM_I值
    if (who_am_i != 0x68)
    {
        return 1; // 设备ID不匹配
    }

    // 读取电源管理寄存器
    if (MPU6050_Read_Reg(PWR_MGMT_1_REG, &pwr_mgmt) != 0)
    {
        return 1; // I2C通信失败
    }

    // 检查是否处于睡眠模式（bit 6）
    if (pwr_mgmt & 0x40)
    {
        return 1; // 设备处于睡眠模式
    }

    return 0; // 连接正常
}

/**
 * @brief 读取原始加速度计数据 (内部使用)
 */
static uint8_t MPU6050_Read_Raw_Accel(int16_t *ax, int16_t *ay, int16_t *az)
{
    uint8_t Rec_Data[6];
    if (MPU6050_Read_Multi_Reg(ACCEL_XOUT_H_REG, Rec_Data, 6) == 0)
    {
        *ax = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        *ay = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        *az = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        return 0;
    }
    *ax = *ay = *az = 0;
    return 1;
}

/**
 * @brief 读取原始陀螺仪数据 (内部使用)
 */
static uint8_t MPU6050_Read_Raw_Gyro(int16_t *gx, int16_t *gy, int16_t *gz)
{
    uint8_t Rec_Data[6];
    if (MPU6050_Read_Multi_Reg(GYRO_XOUT_H_REG, Rec_Data, 6) == 0)
    {
        *gx = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        *gy = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        *gz = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
        return 0;
    }
    *gx = *gy = *gz = 0;
    return 1;
}

/**
 * @brief 对MPU6050进行校准
 * @param DataStruct - MPU6050数据结构指针
 * @note 校准时请将模块水平静置
 */
void MPU6050_Calibrate(MPU6050_t *DataStruct)
{
    int16_t ax, ay, az, gx, gy, gz;
    // 使用32位整型防止累加时溢出
    int32_t Accel_Sum[3] = {0, 0, 0};
    int32_t Gyro_Sum[3] = {0, 0, 0};
    const int num_samples = 500;

    // 读取多次数据进行平均
    for (int i = 0; i < num_samples; i++)
    {
        MPU6050_Read_Raw_Accel(&ax, &ay, &az);
        MPU6050_Read_Raw_Gyro(&gx, &gy, &gz);

        Accel_Sum[0] += ax;
        Accel_Sum[1] += ay;
        Accel_Sum[2] += az;

        Gyro_Sum[0] += gx;
        Gyro_Sum[1] += gy;
        Gyro_Sum[2] += gz;
        HAL_Delay(2);
    }

    // 计算平均值作为偏移
    DataStruct->Accel_X_Offset = Accel_Sum[0] / num_samples;
    DataStruct->Accel_Y_Offset = Accel_Sum[1] / num_samples;
    // Z轴的偏移计算需要减去重力加速度的影响 (1g)
    // 理想静止状态下，Z轴读数应为 1g 对应的 LSB 值。
    // 偏移 = 原始读数平均值 - 理想值
    // 理想值 = 1g / (1g / 灵敏度) = 灵敏度值
    DataStruct->Accel_Z_Offset = (Accel_Sum[2] / num_samples) - MPU6050_ACCEL_SENSITIVITY;

    DataStruct->Gyro_X_Offset = Gyro_Sum[0] / num_samples;
    DataStruct->Gyro_Y_Offset = Gyro_Sum[1] / num_samples;
    DataStruct->Gyro_Z_Offset = Gyro_Sum[2] / num_samples;
}

/**
 * @brief 读取加速度计数据
 * @param DataStruct 数据结构指针
 */
void MPU6050_Read_Accel(MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // 从0x3B开始读取6个字节
    if (MPU6050_Read_Multi_Reg(ACCEL_XOUT_H_REG, Rec_Data, 6) == 0)
    {
        DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

        // 应用偏移量并转换为g值
        DataStruct->Ax = (DataStruct->Accel_X_RAW - DataStruct->Accel_X_Offset) / MPU6050_ACCEL_SENSITIVITY;
        DataStruct->Ay = (DataStruct->Accel_Y_RAW - DataStruct->Accel_Y_Offset) / MPU6050_ACCEL_SENSITIVITY;
        DataStruct->Az = (DataStruct->Accel_Z_RAW - DataStruct->Accel_Z_Offset) / MPU6050_ACCEL_SENSITIVITY;
    }
    else
    {
        // 通信失败时设置为0
        DataStruct->Accel_X_RAW = 0;
        DataStruct->Accel_Y_RAW = 0;
        DataStruct->Accel_Z_RAW = 0;
        DataStruct->Ax = 0;
        DataStruct->Ay = 0;
        DataStruct->Az = 0;
    }
}

/**
 * @brief 读取陀螺仪数据
 * @param DataStruct 数据结构指针
 */
void MPU6050_Read_Gyro(MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    // 从0x43开始读取6个字节
    if (MPU6050_Read_Multi_Reg(GYRO_XOUT_H_REG, Rec_Data, 6) == 0)
    {
        DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

        // 应用偏移量并转换为度/秒
        DataStruct->Gx = (DataStruct->Gyro_X_RAW - DataStruct->Gyro_X_Offset) / MPU6050_GYRO_SENSITIVITY;
        DataStruct->Gy = (DataStruct->Gyro_Y_RAW - DataStruct->Gyro_Y_Offset) / MPU6050_GYRO_SENSITIVITY;
        DataStruct->Gz = (DataStruct->Gyro_Z_RAW - DataStruct->Gyro_Z_Offset) / MPU6050_GYRO_SENSITIVITY;
    }
    else
    {
        // 通信失败时设置为0
        DataStruct->Gyro_X_RAW = 0;
        DataStruct->Gyro_Y_RAW = 0;
        DataStruct->Gyro_Z_RAW = 0;
        DataStruct->Gx = 0;
        DataStruct->Gy = 0;
        DataStruct->Gz = 0;
    }
}

/**
 * @brief 读取温度数据
 * @param DataStruct 数据结构指针
 */
void MPU6050_Read_Temp(MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    // 从0x41开始读取2个字节
    if (MPU6050_Read_Multi_Reg(TEMP_OUT_H_REG, Rec_Data, 2) == 0)
    {
        temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        // MPU6050温度计算公式: Temperature = (TEMP_OUT/340.0) + 36.53
        // 注意：36.53是理论值，实际可能需要根据芯片校准调整为21.0左右
        DataStruct->Temperature = ((float)temp / 340.0f) + 21.0f;
    }
    else
    {
        DataStruct->Temperature = 0.0f;
    }
}

/**
 * @brief 读取所有数据（加速度计、陀螺仪、温度）
 * @param DataStruct 数据结构指针
 */
void MPU6050_Read_All(MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];

    // 从0x3B开始读取14个字节（加速度计 + 温度 + 陀螺仪）
    if (MPU6050_Read_Multi_Reg(ACCEL_XOUT_H_REG, Rec_Data, 14) == 0)
    {
        // 解析加速度计数据
        DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
        DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
        DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

        // 解析温度数据
        int16_t temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
        // MPU6050温度计算公式: Temperature = (TEMP_OUT/340.0) + 36.53
        // 注意：36.53是理论值，实际可能需要根据芯片校准调整为21.0左右
        DataStruct->Temperature = ((float)temp / 340.0f) + 21.0f;

        // 解析陀螺仪数据
        DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
        DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
        DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

        // 转换为实际值（应用偏移量）
        // 加速度计转换
        DataStruct->Ax = (DataStruct->Accel_X_RAW - DataStruct->Accel_X_Offset) / MPU6050_ACCEL_SENSITIVITY;
        DataStruct->Ay = (DataStruct->Accel_Y_RAW - DataStruct->Accel_Y_Offset) / MPU6050_ACCEL_SENSITIVITY;
        DataStruct->Az = (DataStruct->Accel_Z_RAW - DataStruct->Accel_Z_Offset) / MPU6050_ACCEL_SENSITIVITY;

        // 陀螺仪转换
        DataStruct->Gx = (DataStruct->Gyro_X_RAW - DataStruct->Gyro_X_Offset) / MPU6050_GYRO_SENSITIVITY;
        DataStruct->Gy = (DataStruct->Gyro_Y_RAW - DataStruct->Gyro_Y_Offset) / MPU6050_GYRO_SENSITIVITY;
        DataStruct->Gz = (DataStruct->Gyro_Z_RAW - DataStruct->Gyro_Z_Offset) / MPU6050_GYRO_SENSITIVITY;
    }
    else
    {
        // 通信失败时清零所有数据
        DataStruct->Accel_X_RAW = 0;
        DataStruct->Accel_Y_RAW = 0;
        DataStruct->Accel_Z_RAW = 0;
        DataStruct->Gyro_X_RAW = 0;
        DataStruct->Gyro_Y_RAW = 0;
        DataStruct->Gyro_Z_RAW = 0;
        DataStruct->Ax = 0;
        DataStruct->Ay = 0;
        DataStruct->Az = 0;
        DataStruct->Gx = 0;
        DataStruct->Gy = 0;
        DataStruct->Gz = 0;
        DataStruct->Temperature = 0;
    }
}
