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
#define SCAN_BUFFER_SIZE 5



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

    servo_move(angle);

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

int ir_scanRange(int scanVals[], int startDeg, int endDeg, int numScans){
    int i;
    int j = 0;
    for(i = startDeg; i < endDeg; i++){

        scanVals[j] = multiScanIR(i, numScans);
        j++;
    }

    return j;
}

int scan_containsObject(int dataArray[], int arraySize, int threshold){
    int numObjects = 0;
    int i, j;
    int scanBuffer[SCAN_BUFFER_SIZE];
    int bufferSum;
    int bufferAvg;

    int currentObj = 0;
    int startAng;

    for(i = 0; i < SCAN_BUFFER_SIZE; i++){
        scanBuffer[i] = dataArray[i];
    }

    for(i = SCAN_BUFFER_SIZE + 1; i < arraySize; i++){
        bufferSum = 0;
        for(j = 0; j < SCAN_BUFFER_SIZE; j++){
            bufferSum += scanBuffer[j];
        }
        bufferAvg = bufferSum / SCAN_BUFFER_SIZE;

        if(dataArray[i] > bufferAvg + threshold && !currentObj){
            currentObj = 1;
            startAng = i;
        }
        if(dataArray[i] < bufferAvg - threshold && currentObj){
            currentObj = 0;
            if(i - startAng > 5){
                numObjects++;
            }

        }

        scanBuffer[i % SCAN_BUFFER_SIZE] = dataArray[i];
    }

    if(currentObj == 1){
        return numObjects + 1;
    }
    return numObjects;
}
