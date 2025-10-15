/*
 * ultransonic.c
 *
 *  Created on: Oct 7, 2025
 *      Author: UnikoZera
 */

#include "ultransonic.h"
#include "main.h"
#include <stdint.h>
#include "stm32f1xx.h"
#include "gpio.h"
#include "tim.h"

// 初始化超声波传感器
void Ultrasonic_Init(void)
{
    HAL_GPIO_WritePin(HC_Trigger_GPIO_Port, HC_Trigger_Pin, GPIO_PIN_RESET);
    Ultrasonic_Trigger10us();
}

// 使用 TIM4 作为 1us 计数器，产生 PC14(触发脚) 的 10us 高电平
void Ultrasonic_Trigger10us(void)
{
    HAL_TIM_Base_Stop(&htim4);
    __HAL_TIM_SET_COUNTER(&htim4, 0);

    // 拉高 10us
    HAL_GPIO_WritePin(HC_Trigger_GPIO_Port, HC_Trigger_Pin, GPIO_PIN_SET);
    HAL_TIM_Base_Start(&htim4);
    uint32_t arrp1 = __HAL_TIM_GET_AUTORELOAD(&htim4) + 1U;
    uint32_t prev = __HAL_TIM_GET_COUNTER(&htim4);
    uint32_t elapsed = 0U;
    while (elapsed < 10U)
    {
        uint32_t curr = __HAL_TIM_GET_COUNTER(&htim4);
        if (curr >= prev)
        {
            elapsed += (uint32_t)(curr - prev);
        }
        else
        {
            elapsed += (uint32_t)(arrp1 - prev + curr); // 处理环回
        }
        prev = curr;
    }
    HAL_GPIO_WritePin(HC_Trigger_GPIO_Port, HC_Trigger_Pin, GPIO_PIN_RESET);
    HAL_TIM_Base_Stop(&htim4);
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4) // timer是45ms的定时器中断 按道理来说是38ms的超声波极限，但是预留点是好事
    {
        Ultrasonic_Trigger10us();
    }
}

// void Ultrasonic_Read(void) // boy, it's gonna be finished in "stm32f1xx_it.c" file!
// {

// }