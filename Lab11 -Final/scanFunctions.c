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
#define MINIMUM_OBJ_SIZE 5


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

int locateObjects(int startAng, int endAng, int horizOff[], int vertOff[], float objWidths[], int maxObj){
    int startDegs[maxObj];
    int endDegs[maxObj];

    int degWidth;
    int midAngle;
    float dist;

    int numObjects = getObjectEdges(startAng, endAng, startDegs, endDegs, maxObj);
    int i;
    for(i = 0; i < numObjects; i++){
        degWidth = endDegs[i] - startDegs[i];
        midAngle = startDegs[i] + (degWidth / 2);
        dist = pingAt(midAngle);

        objWidths[i] = getLinWidth(degWidth, dist);

    }
    return numObjects;
}

int getObjectEdges(int startDeg, int endDeg, int startAngles[], int endAngles[], int maxObj){
    int scanBuffer[SCAN_BUFFER_SIZE];
    int buffSum;
    int buffAvg;
    int currObj;

    int i, j;
    int numObjects = 0;
    int numScans = 3;
    int threshold = 90;
    int currIR;

    for(i = 0; i < SCAN_BUFFER_SIZE; i++){
        scanBuffer[i] = multiScanIR(startDeg, numScans);
    }

    for(i = startDeg; i < endDeg; i++){

        if(numObjects == maxObj){
            return numObjects;
        }

        buffSum = 0;
        for(j = 0; j < SCAN_BUFFER_SIZE; j++){
            buffSum += scanBuffer[j];
        }
        buffAvg = buffSum / SCAN_BUFFER_SIZE;

        currIR = multiScanIR(i, numScans);

        if(currIR > buffAvg + threshold && !currObj){
            currObj = 1;
            startAngles[numObjects] = i;
        }

        if(currIR < buffAvg - threshold && currObj){
            currObj = 0;
            if(i - startAngles[numObjects] > MINIMUM_OBJ_SIZE){
                endAngles[i] = currIR;
                numObjects++;
            }
        }
    }
    return numObjects;
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
            if(i - startAng > MINIMUM_OBJ_SIZE){
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

float getLinWidth(int degWidth, float dist){

    float radAng = degWidth * (3.14 / 180);
    float linWidth = 2* dist * tan(radAng / 2);
    return linWidth;
}

float getHorizontalOffset(int angle, float dist){
    float radAng = angle * (3.14 / 180);
    float offset = cos(radAng) * dist;
    return offset;
}

float getVerticalOffset(int angle, float dist){
    float radAng = angle * (3.14 / 180);
    float linWidth = 2* dist * tan(radAng / 2);
    return linWidth;
}
