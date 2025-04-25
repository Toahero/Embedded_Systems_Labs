/*
 * moveAndScan.h
 *
 *  Created on: Apr 25, 2025
 *      Author: jagaul
 */

#ifndef MOVEANDSCAN_H_
#define MOVEANDSCAN_H_

#include "adc.h"
#include "servo.h"
#include <open_interface.h>
#include <lcd.h>
#include "Timer.h"
#include "uart-interrupt.h"


int move_forward_detect(oi_t *sensor_data, int* distance_mm);

#endif /* MOVEANDSCAN_H_ */
