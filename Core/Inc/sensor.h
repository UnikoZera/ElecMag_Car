/*
 * sensor.h
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

 
//note: 这里认为mpu在I2C2总线上 adc在ADC1上，使用的是In1到In4通道

#ifndef INC_SENSOR_H_
#define INC_SENSOR_H_

#include "gyro.h"
#include "main.h"
#include "adc.h"
#include "tim.h"
#include "gpio.h"

extern MPU6050_t gyro_data;
extern float adc_data[4];

void Sensor_Init(void);
void Sensor_Updater(void);

#endif /* INC_SENSOR_H_ */
