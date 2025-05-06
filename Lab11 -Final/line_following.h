/*
 * line_following.h
 *
 *  Created on: Apr 26, 2025
 *      Author: jagaul
 */

#ifndef LINE_FOLLOWING_H_
#define LINE_FOLLOWING_H_

#include <inc/tm4c123gh6pm.h>
#include <open_interface.h>
#include <lcd.h>
#include "Timer.h"
#include "button.h"
#include "movement.h"
#include "scanFunctions.h"
#include "sharedStructs.h"

int followLine(oi_t *sensor_data, int* distance_mm);

void followPerimeter(oi_t *sensor_data);

int scanLine(oi_t *sensor_data, struct robotCoords* botPos, struct obSide* foundObs, int maxToAdd, int* distance_mm);

void calibrate_CliffValue(oi_t *sensor_data);

void displayCliffVals();

#endif /* LINE_FOLLOWING_H_ */
