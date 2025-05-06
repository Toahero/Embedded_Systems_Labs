/*
 * mapScanning.c
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
#include "line_following.h"
#include "scanFunctions.h"
#include "sharedStructs.h"

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

void printObsData(struct fieldObs* obstacle);

int scanEdge(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int maxToAdd){

    int numObs = 0;

    int roombaOffset = 300;
    int intervalSize = 500;

    int expectedSide;
    if(botPos->direction %2 == 0){
        expectedSide = 4270 - roombaOffset;
    }
    else{
        expectedSide = 2440 - roombaOffset;
    }

    int addLimit = maxToAdd;
    int distToTravel;
    int followResult = 0;
    struct obSide obsData;
    int sweepCount;

    while(1){
        if(addLimit <= 0){
            return numObs;
        }

        distToTravel = intervalSize;
        lcd_printf("Running");
        followResult = scanLine(sensor_data, botPos, &obsData, addLimit, &distToTravel);
        updateBotPos(botPos, intervalSize - distToTravel);

        switch(followResult){
        case 4: //If an object is detected
            //Replace this later
            lcd_printf("Object found: Size %d", obsData.size);
            timer_waitMillis(1000);

            //Decrease the number that can be added.
            addLimit--;

            //Increase number of obs, add it to the array.
            numObs++;
            lineScanToObs(obsLoc, botPos, &obsData);
            obsLoc++;
            break;

        case 3: //Hole detected
            lcd_printf("Hole Detected");
            timer_waitMillis(1000);
            addOnEdge(0, botPos, obsLoc);
            numObs++;
            addLimit--;
            break;

        case 2: //Short Object Detected.
            lcd_printf("Short Object Detected");
            timer_waitMillis(1000);
            addOnEdge(1, botPos, obsLoc);
            numObs++;
            addLimit--;
            break;

        case 1: //Cybot has reached end of edge
            lcd_printf("End of edge");
            timer_waitMillis(1000);
            return numObs;

        case 0: //Cybot completed the interval without issue.
            lcd_printf("End of interval");
            timer_waitMillis(1000);
            //Add function to scan for objects
            sweepCount += sweepRange(45, 135, botPos, obsLoc, addLimit);
            numObs += sweepCount;
            obsLoc += sweepCount;
            addLimit -= sweepCount;
            break;

        default://If a button is pressed, immediately terminate
            return 0;
        }
    }





    return numObs;
}

void scanPerimeter(oi_t *sensor_data){
    int totalDist;
    int numSides = 0;
    int rightCliff, leftCliff;

    int roombaOffset = 300;

    //temporary array for storing IR values;
    int irArray[180];
    int arraySize;
    int numObjects;

    //Length of the area in mm
    int expectedLength = 4270 - roombaOffset;
    int expectedWidth = 2440 - roombaOffset;

    int xCoord = roombaOffset * .5;
    int yCoord = roombaOffset * .5;

    int currentSide = 0;

    int forwardsMove = 600;
    int sidewaysMove = 400;

    int sideDist;
    int travelDist;
    int followResult;

    timer_waitMillis(500);

    followResult = 0;

    int maxObjects = 100;

    int intervalSize = 500;

    char output[100];

    int i;

    while(followResult != 1){
        travelDist = intervalSize;
        followResult = followLine(sensor_data, &travelDist);

        //lcd_printf("Returned result %d", followResult);

        if(followResult == -1){
            return;
        }

        sideDist = intervalSize - travelDist;
        totalDist += sideDist;
        if(currentSide == 0){
            xCoord += sideDist;
        }
        lcd_printf("X:%d Y:%d", xCoord, yCoord);



        /*if(numObjects != 0){
            lcd_printf("%d Objects detected", numObjects);
            timer_waitMillis(5000);
        }*/

        switch(followResult){
        case -1:
            return;

        case 0:
            arraySize = ir_scanRange(irArray, 45, 135, 2);
            numObjects = scan_containsObject(irArray, arraySize, 90);

            if(numObjects){
                /*numObjects = locateObjects(45, 135, objectSizes, horizOffsets, vertOffsets, maxObjects);
                for(i = 0; i < numObjects; i++){
                    lcd_printf("X Offset:%.2f\nY Offset:%.2f\nWidth:%.2f", horizOffsets[i], vertOffsets[i], objectSizes[i]);
                    timer_waitMillis(500);
                }*/
            }

            break;

        case 1:
            turn_left(sensor_data, 90);
            currentSide++;

        case 2:
            sprintf(output, "Object %d,%d %d\n", xCoord, yCoord, 140);
            uart_sendStr(output);

            if(totalDist + forwardsMove * 2 + 100> expectedLength){
                cutCorner(sensor_data, 0, sidewaysMove);
                currentSide++;
            }
            else{
                move_aroundObject(sensor_data, 0, sidewaysMove, forwardsMove);
            }
            break;

        case 3:
            sprintf(output, "Hole %d,%d %d\n", xCoord, yCoord, 140);
            uart_sendStr(output);

            if(totalDist + forwardsMove + 100> expectedLength){
                cutCorner(sensor_data, 0, sidewaysMove * 2);
                currentSide++;
            }
            else{
                move_aroundObject(sensor_data, 0, sidewaysMove * 2, forwardsMove * 2);
            }
            break;
        }


    }


    rightCliff = sensor_data->cliffRightSignal;
    leftCliff = sensor_data->cliffLeftSignal;
    if(followResult != 0){
        /*if(rightCliff > leftCliff){
            turn_right(sensor_data, 90);
        }
        else{

            turn_left(sensor_data, 90);
        }*/


    }

    numSides++;

    lcd_printf("Total Distance: %d", totalDist);
}

int sweepRange(int startAng, int endAng, struct robotCoords* botPosition, struct fieldObs* obsLoc, int maxToAdd){
    int currentAngle = startAng;
    int obsCount = 0;
    struct obstacle obsArray[maxToAdd];
    char output[100];

    uart_sendStr("Starting Scan\n");

    while(currentAngle < endAng && obsCount < maxToAdd){
        obsCount += sweepNextObs(&obsArray[obsCount], &currentAngle, endAng);
    }

    int i;
    for(i = 0; i < obsCount; i++){
        obsLoc->itemType = 3;
        obsLoc->sizeMM = obsArray[i].sizeMM;

        switch(botPosition->direction){
        case 3:

            //The robot is facing west
            obsLoc->xCoord = botPosition->xCoord - obsArray[i].vertOffsetMM;
            obsLoc->yCoord = botPosition->yCoord - obsArray[i].horiOffsetMM;
            break;
        case 2:
            //The robot is facing south
            obsLoc->xCoord = botPosition->xCoord - obsArray[i].horiOffsetMM;
            obsLoc->yCoord = botPosition->yCoord - obsArray[i].vertOffsetMM;
            break;

        case 1:
            //The robot is facing east
            obsLoc->xCoord = botPosition->xCoord + obsArray[i].vertOffsetMM;
            obsLoc->yCoord = botPosition->yCoord + obsArray[i].horiOffsetMM;
            break;

        default:
            //The robot is facing north
            obsLoc->xCoord = botPosition->xCoord + obsArray[i].horiOffsetMM;
            obsLoc->yCoord = botPosition->yCoord + obsArray[i].vertOffsetMM;
        }
        obsLoc++;
    }


    sprintf("Preconversion Data: %d new objects found\n", obsCount);
    uart_sendStr(output);
    lcd_printf("%s", output);

    for(i = 0; i < obsCount; i++){
        sprintf(output, "Object %d: x: %d   y: %d  size:%d\n", i, obsArray[i].horiOffsetMM, obsArray[i].vertOffsetMM, obsArray[i].sizeMM);
        uart_sendStr(output);
        lcd_printf("%s", output);
    }

    return obsCount;
}

void testSweep(void){
    struct robotCoords testCoords;
    testCoords.xCoord = 100;
    testCoords.yCoord = 100;
    testCoords.direction = 0;

    int maxObjects = 20;
    int numObjects = 0;
    int obsAdded;

    struct fieldObs obsArray[maxObjects];

    numObjects += sweepRange(0, 180, &testCoords, &obsArray[0], maxObjects - numObjects);

    uart_sendStr("\nPostconversion data: True coordinates\n");
    int i;
    for(i = 0; i < numObjects; i++){
        printObsData(&obsArray[i]);
    }
}

void lineScanTest(oi_t *sensor_data){
    struct robotCoords testCoords;
    testCoords.xCoord = 100;
    testCoords.yCoord = 100;
    testCoords.direction = 0;

    int maxObjects = 20;
    int numObjects = 0;
    int travelDist = 1000;
    int obsAdded;

    char output[100];

    struct obSide foundObs;
    int result;
    result = scanLine(sensor_data, &testCoords, &foundObs, maxObjects, &travelDist);

    lcd_printf("Result: %d", result);
    timer_waitMillis(1000);

    if(result == 4){
        sprintf(output, "EdgePos: %d Size: %d Dist: %.2f\n", testCoords.yCoord, foundObs.size, foundObs.midDist);
        uart_sendStr(output);
        lcd_printf("%s", output);
    }
}

void edgeScanTest(oi_t *sensor_data){


    struct robotCoords testCoords;
    testCoords.xCoord = 100;
    testCoords.yCoord = 100;
    testCoords.direction = 0;

    int maxObjects = 20;
    int numObjects = 0;
    int travelDist = 1000;
    int obsAdded;

    char output[100];

    struct fieldObs obsArray[maxObjects];

    int result;
    result = scanEdge(sensor_data, &testCoords, &obsArray[0], maxObjects);
    int i;
    sprintf(output, "\n%d Objects Found.\n", result);
    uart_sendStr(output);
    for(i = 0; i < result; i++){
        sprintf(output, "Object %d: X:%d Y:%d Size:%d\n", i, obsArray[i].xCoord, obsArray[i].yCoord, obsArray[i].sizeMM);
        uart_sendStr(output);
    }
}

void printObsData(struct fieldObs* obstacle){
    char output[100];
    sprintf(output,  "Item Type: %d, Coords: (%d,%d), Size: %d\n", obstacle->itemType, obstacle->xCoord, obstacle->yCoord, obstacle->sizeMM);
    uart_sendStr(output);
}
