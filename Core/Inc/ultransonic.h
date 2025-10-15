/*
 * ultransonic.h
 *
 *  Created on: Oct 7, 2025
 *      Author: UnikoZera
 */

#ifndef INC_ULTRANSONIC_H_
#define INC_ULTRANSONIC_H_

#include "main.h"
#include "gpio.h"
#include "tim.h"
#include "stm32f1xx_it.h"
#include <stdint.h>


// 以 TIM4 (72MHz/72=1MHz) 为时间基，产生 PC14 10us 高电平触发脉冲
void Ultrasonic_Trigger10us(void);
// 初始化触发脚状态（置低）
void Ultrasonic_Init(void);
// distance is a global variable that you can using in other files just include "ultransonic.h"

#endif /* INC_ULTRANSONIC_H_ */
