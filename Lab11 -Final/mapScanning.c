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
#include <math.h>

#define MAP_LENGTH 4270
#define MAP_WIDTH 2440

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
void scanPerimeter(oi_t *sensor_data);

int middleScan(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int addLimit){
    int offsetMove = 500;

    int turnDir;

    switch(botPos->direction){
    //Turn direction depends on robot direction and position

    case 3: //Robot is pointing west
        if(botPos->xCoord > MAP_LENGTH/2){
            turnDir = 0;
        }
        else{
            turnDir = 1;
        }
        break;

    case 2: //Robot is pointing south
        if(botPos->xCoord > MAP_WIDTH/2){
            turnDir = 1;
        }
        else{
            turnDir = 0;
        }
        break;

    case 1: //Robot is pointing east
        if(botPos->yCoord > MAP_LENGTH/2){
            turnDir = 1;
        }
        else{
            turnDir = 0;
        }
        break;

    default: //Robot is pointing north
        if(botPos->xCoord > MAP_WIDTH/2){
            turnDir = 0;
        }
        else{
            turnDir = 1;
        }
    }

    if(turnDir == 0){
        turn_bot_left(sensor_data, botPos);
    }
    else{
        turn_bot_right(sensor_data, botPos);
    }

    int sweepCount = sweepRange(45, 135, botPos, obsLoc, addLimit);
    int result;

    if(sweepCount < 1){
        result = move_bot_forward(sensor_data, botPos, offsetMove);
        sweepCount = sweepRange(0, 180, botPos, obsLoc, addLimit);
        turn_bot_right(sensor_data, botPos);
        turn_bot_right(sensor_data, botPos);
        move_bot_forward(sensor_data, botPos, offsetMove * 0.9);

        if(turnDir == 0){
            turn_bot_left(sensor_data, botPos);
        }
        else{
            turn_bot_right(sensor_data, botPos);
        }
    }
    else{
        if(turnDir == 0){
            turn_bot_right(sensor_data, botPos);
        }
        else{
            turn_bot_left(sensor_data, botPos);
        }
    }




    obsLoc += sweepCount;
    return sweepCount;
}

int scanEdge(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int maxToAdd){


    int numObs = 0;

    int roombaOffset = 300;
    int intervalSize = 400;

    int expectedSide;
    if(botPos->direction %2 == 0){
        expectedSide = MAP_LENGTH - roombaOffset;
    }
    else{
        expectedSide = MAP_WIDTH - roombaOffset;
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

        sweepCount += sweepRange(70, 120, botPos, obsLoc, addLimit);
        numObs += sweepCount;
        obsLoc += sweepCount;
        addLimit -= sweepCount;

        if(sweepCount > 0){
            oi_play_song(2);
            lcd_printf("%d objects found", sweepCount);
            move_Around_Object(sensor_data, botPos, 200);
        }

        distToTravel = intervalSize;
        lcd_printf("Running");
        followResult = scanLine(sensor_data, botPos, &obsData, addLimit, &distToTravel);
        updateBotPos(botPos, intervalSize - distToTravel);



        switch(followResult){
        case 4: //If an object is detected
            //Replace this later
            oi_play_song(2);
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
            oi_play_song(0);
            lcd_printf("Hole Detected");
            timer_waitMillis(1000);
            addOnEdge(0, botPos, obsLoc);
            numObs++;
            addLimit--;

            move_Around_Object(sensor_data, botPos, 610);
            break;

        case 2: //Short Object Detected.
            oi_play_song(1);
            lcd_printf("Short Object Detected");
            timer_waitMillis(1000);
            addOnEdge(1, botPos, obsLoc);
            numObs++;
            addLimit--;

            move_Around_Object(sensor_data, botPos, 130);

            break;

        case 1: //Cybot has reached end of edge
            lcd_printf("End of edge");
            timer_waitMillis(1000);
            return numObs;

        case 0: //Cybot completed the interval without issue.
            lcd_printf("End of interval");
            timer_waitMillis(1000);
            middleScan(sensor_data, botPos, obsLoc, addLimit);


        default://If a button is pressed, immediately terminate
            return 0;
        }
    }
    return numObs;
}

void scanPerimeter(oi_t *sensor_data){
    struct robotCoords botCoords;
    botCoords.xCoord = 2400;
    botCoords.yCoord = 100;
    botCoords.direction = 0;

    int maxObjects = 20;
    int numObjects = 0;
    int travelDist = 1000;
    int obsAdded;

    char output[100];

    struct fieldObs obsArray[maxObjects];

    int result;

    int i;

    for(i = 0; i < 1; i++){
        numObjects += scanEdge(sensor_data, &botCoords, &obsArray[0], maxObjects);
        turn_bot_left(sensor_data, &botCoords);
    }

    int maxObjectSize = 0;
    struct fieldObs largestObs;

    for(i = 0; i < result; i++){
        if(obsArray[i].sizeMM > maxObjectSize && obsArray[i].itemType != 0){
            largestObs = obsArray[i];
            maxObjectSize = obsArray[i].sizeMM;
        }

        sprintf(output, "Object %d: X:%d Y:%d Size:%d\n", i, obsArray[i].xCoord, obsArray[i].yCoord, obsArray[i].sizeMM);
        uart_sendStr(output);
    }

    sprintf(output, "\nThe largest object is %d mm, located at (%d, %d)\n", largestObs.sizeMM, largestObs.xCoord, largestObs.yCoord);
    uart_sendStr(output);

    int xDist, yDist;
    do{
        xDist = (botCoords.xCoord - largestObs.xCoord);
        yDist = (botCoords.yCoord - largestObs.yCoord);

        int moveDir;

        if(abs(xDist) > 100){
            if(xDist > 0){
                moveDir = 1;
            }
            else{
                moveDir = 3;
            }
        }

        while(botCoords.direction != moveDir){
            turn_bot_left(sensor_data, &botCoords);
        }

        move_bot_forward(sensor_data, &botCoords, xDist/2);

        if(abs(yDist) > 100){
            if(yDist > 0){
                moveDir = 0;
            }
            else{
                moveDir = 1;
            }
        }

        while(botCoords.direction != moveDir){
            turn_bot_left(sensor_data, &botCoords);
        }

        move_bot_forward(sensor_data, &botCoords, xDist/2);
    }while(xDist > 100 || yDist > 100);
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

    /*
    char noteArray[2];
    char noteLengths[2];
    noteArray[0] = 48;
    noteArray[1] = 30;
    noteLengths[0] = 32;
    noteLengths[1] = 32;

    oi_loadSong(0, 2, noteArray, noteLengths);*/

    struct robotCoords testCoords;
    testCoords.xCoord = 2400;
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
