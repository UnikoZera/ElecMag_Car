/*
 * tracker.h
 *
 *  Created on: Jun 7, 2025
 *      Author: UnikoZera
 */

#ifndef INC_TRACKER_H_
#define INC_TRACKER_H_

#include "pid.h"
#include "motor.h"
#include "sensor.h"
#include <stdbool.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>

void Tracker_Init(void);
void Tracker_Compute(void);

// 可以当调试用
void PID_Motor_Controllers_Speed_Updater(float target_left_speed, float target_right_speed);
void PID_Motor_Controllers_Position_Updater(float target_left_position, float target_right_position);


#endif /* INC_TRACKER_H_ */