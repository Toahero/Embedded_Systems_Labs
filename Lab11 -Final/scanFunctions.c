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

#include <math.h>

#define IR_MINIMUM 300
#define IR_RESCAN_THRESH 100
#define IR_OBJ_THRESH 90
#define SCAN_BUFFER_SIZE 5
#define MINIMUM_OBJ_SIZE 5

double getObjectSize(int degWidth, double distance);
double getHorizontalOffset(int angle, double dist);
double getVerticalOffset(int angle, double dist);

struct obstacle{
    int horiOffsetMM;
    int vertOffsetMM;

    int sizeMM;
};

struct obsEdges{
    int startDeg;
    int endDeg;
};

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


int sweepNextObs(struct obstacle* currObs, int* currAng, int endAng){
    struct obsEdges currentData;

    int result;
    int angle = *currAng;
    result = sweepEdges(&currentData, currAng, endAng);

    //*currAng = angle;

    if(result == 0){
        return 0;
    }

    int degWidth = currentData.endDeg - currentData.startDeg;
    int midDeg = currentData.startDeg + (degWidth/2);
    float midDist = pingAt(midDeg);

    //float test = getObjectSize(90, 60.9);

    double objectSize;
    objectSize = getObjectSize(degWidth, midDist);
    currObs->sizeMM = objectSize * 10;
    //currObs->sizeMM = 10 * getObjectSize(degWidth, midDist);
    double hOffset = getHorizontalOffset(midDeg, midDist);

    currObs->horiOffsetMM = 10 * hOffset;
    currObs->vertOffsetMM = 10 * getVerticalOffset(midDeg, midDist);
    return 1;
}

int sweepEdges(struct obsEdges* currObs, int* angle, int endAng){
    int scanBuffer[SCAN_BUFFER_SIZE];
    int buffSum;
    int buffAvg;
    int foundObs = 0;

    int i, j;
    int numScans = 3;
    int threshold = 90;
    int currIR;

    for(i = 0; i < SCAN_BUFFER_SIZE; i++){
        scanBuffer[i] = multiScanIR(*angle, numScans);
    }

    while(*angle < endAng){

        buffSum = 0;
        for(j = 0; j < SCAN_BUFFER_SIZE; j++){
            buffSum += scanBuffer[j];
        }
        buffAvg = buffSum / SCAN_BUFFER_SIZE;

        currIR = multiScanIR(*angle, numScans);

        if(currIR > buffAvg + threshold && !foundObs){
            foundObs = 1;
            currObs->startDeg = *angle;
        }

        if(currIR < buffAvg - threshold && foundObs){
            foundObs = 0;
            if(*angle - currObs->startDeg > MINIMUM_OBJ_SIZE){
                currObs->endDeg = *angle;
                return 1;
            }
        }

        *angle = *angle + 1;
    }
    return 0;
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

float getLinWidth(int degWidth, float obsDist){

    float radWidth = degWidth * (3.14 / 180.0);
    float linWidth = 2.0 * obsDist * tan(radWidth / 2);
    return linWidth;
}

double getObjectSize(int degWidth, double distance){
    double radWidth = degWidth * (3.14 / 180.0);
    double linWidth = 2.0 * distance * tan(radWidth / 2);
    return linWidth;
}

double getHorizontalOffset(int angle, double dist){
    double radAng = angle * (3.14 / 180);
    double offset = cos(radAng) * dist;
    return offset;
}

double getVerticalOffset(int angle, double dist){
    double radAng = angle * (3.14 / 180);
    double linWidth = 2* dist * tan(radAng / 2);
    return linWidth;
}
