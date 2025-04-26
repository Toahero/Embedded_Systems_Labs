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


void ir_scanRange(int scanVals[], int startDeg, int endDeg);

int multiScanIR(int angle, int numScans);


float pingAt(int angle);



#endif /* SCANFUNCTIONS_H_ */
