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


int ir_scanRange(int scanVals[], int startDeg, int endDeg, int numScans);

int multiScanIR(int angle, int numScans);

int locateObjects(int startAng, int endAng, float horizOff[], float vertOff[], float objWidths[], int maxObj);

int getObjectEdges(int startDeg, int endDeg, int startAngles[], int endAngles[], int maxObj);

float pingAt(int angle);

int scan_containsObject(int dataArray[], int arraySize, int threshold);

static float getLinWidth(int degWidth, float dist);

float getHorizontalOffset(int angle, float dist);

float getVerticalOffset(int angle, float dist);

#endif /* SCANFUNCTIONS_H_ */
