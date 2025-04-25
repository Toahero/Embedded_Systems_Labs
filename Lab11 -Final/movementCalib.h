/*
 * movementCalib.h
 *
 *  Created on: Apr 25, 2025
 *      Author: jagaul
 */

#ifndef MOVEMENTCALIB_H_
#define MOVEMENTCALIB_H_

#include "adc.h"
#include "servo.h"
#include <open_interface.h>
#include <lcd.h>
#include "Timer.h"
#include "uart-interrupt.h"
#include "ping.h"

double getBackwardAdjust(oi_t *sensor_data);

void adjMotorModifiers(double* leftAdj, double* rightAdj);

#endif /* MOVEMENTCALIB_H_ */
