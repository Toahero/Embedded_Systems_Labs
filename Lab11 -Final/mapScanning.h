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
int navigateToCoords(oi_t *sensor_data, struct robotCoords* botPos, struct robotCoords* goalPos, struct fieldObs* obsLoc, int addLimit);

int scanEdge(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int maxToAdd, int maxDist);

int sweepRange(int startAng, int endAng, struct robotCoords* botPosition, struct fieldObs* obsLoc, int maxToAdd);

void testSweep(void);

void lineScanTest(oi_t *sensor_data);

void printObsData(struct fieldObs* obstacle);

#endif /* MAPSCANNING_H_ */
