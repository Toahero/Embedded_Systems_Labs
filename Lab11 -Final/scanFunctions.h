/*
 * scanFunctions.h
 *
 *  Created on: Apr 26, 2025
 *      Author: jagaul
 */

#ifndef SCANFUNCTIONS_H_
#define SCANFUNCTIONS_H_

#include "Timer.h"
#include "lcd.h"
#include "servo.h"
#include "button.h"
#include "ping.h"
#include "adc.h"
#include "uart-interrupt.h"

#include <math.h>

struct obstacle{
    int horiOffsetMM;
    int vertOffsetMM;

    int sizeMM;
};

double getObjectSize(int degWidth, double distance);

int ir_scanRange(int scanVals[], int startDeg, int endDeg, int numScans);

int multiScanIR(int angle, int numScans);

int sweepNextObs(struct obstacle* currObs, int* currAng, int endAng);

int sweepEdges(struct obsEdges* currObs, int* angle, int endAng);

float pingAt(int angle);

int scan_containsObject(int dataArray[], int arraySize, int threshold);

double getLinWidth(int degWidth, double obsDist);

double getHorizontalOffset(int angle, double dist);

double getVerticalOffset(int angle, double dist);

#endif /* SCANFUNCTIONS_H_ */
