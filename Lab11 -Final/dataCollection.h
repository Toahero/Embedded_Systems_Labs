/*
 * dataCollection.h
 *
 *  Created on: Apr 23, 2025
 *      Author: jagaul
 */

#ifndef DATACOLLECTION_H_
#define DATACOLLECTION_H_

#include <open_interface.h>
#include <lcd.h>
#include "Timer.h"
#include "uart-interrupt.h"

void collect_cliffSignals(oi_t *sensor_data, int distance_mm);

#endif /* DATACOLLECTION_H_ */
