/*
 * scanFunctions.c
 *
 *  Created on: Apr 26, 2025
 *      Author: jagaul
 */
#include "Timer.h"
#include "lcd.h"
#include "servo.h"
#include "button.h"
#include "ping.h"
#include "adc.h"
#include "uart-interrupt.h"


#define IR_MINIMUM 300
#define IR_RESCAN_THRESH 100
#define IR_OBJ_THRESH 90

void ir_scanRange(int scanVals[], int startDeg, int endDeg){
    int i;
    for(i = startDeg; i < endDeg; i++){
        servo_move(i);
        scanVals[i] = multiScanIR(i, 1);
    }
}

float pingAt(int angle){
    servo_move(angle);
    return ping_getDistance();
}

int multiScanIR(int angle, int numScans){

    int scanVal;
    int scanSum = 0.0;
    int i = 0;
    int change = 0;
    int currAvg = 0;

    int failCount = 0;
    int failMax = 5;

    while(i < numScans){

        scanVal = adc_read();

        if(scanVal < IR_MINIMUM){
            scanVal = IR_MINIMUM;
        }

        scanSum += scanVal;
        currAvg = scanSum / (i+1);
        change = abs(currAvg - scanVal);

        if(change > IR_RESCAN_THRESH && failCount < failMax){
            i = 0;
            scanSum = 0;
            failCount++;
        }
        else {
            i++;
        }
    }

    return scanSum /numScans;

}
