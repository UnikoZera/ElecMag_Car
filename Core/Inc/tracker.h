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
#include <math.h>

void Tracker_Init(void);
void Tracker_Compute(void);

#endif /* INC_TRACKER_H_ */
