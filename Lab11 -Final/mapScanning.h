/*
 * mapScanning.h
 *
 *  Created on: Apr 26, 2025
 *      Author: jagaul
 */

#ifndef MAPSCANNING_H_
#define MAPSCANNING_H_

#include "Timer.h"
#include "lcd.h"
#include "servo.h"
#include "button.h"
#include "ping.h"
#include "adc.h"
#include "uart-interrupt.h"
#include "line_following.h"
#include "scanFunctions.h"

void scanPerimeter(oi_t *sensor_data);

#endif /* MAPSCANNING_H_ */
