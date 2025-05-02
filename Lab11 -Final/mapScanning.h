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

/*struct fieldObs{
    //0 for hole, 1 for low obstacle, 2 for high
    int itemType;
    int xCoord;
    int yCoord;
    int sizeMM;
};*/

/*struct robotCoords{
    int xCoord;
    int yCoord;
    int direction;
};*/

void scanPerimeter(oi_t *sensor_data);
int scanEdge(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int maxToAdd);

int sweepRange(int startAng, int endAng);

void testSweep(void);

void lineScanTest(oi_t *sensor_data);

void printObsData(struct fieldObs* obstacle);

#endif /* MAPSCANNING_H_ */
