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
#define BOT_OFFSET 300
#define NAV_THRESHOLD 20

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
int navigateToCoords(oi_t *sensor_data, struct robotCoords* botPos, struct robotCoords* goalPos, struct fieldObs* obsLoc, int addLimit);

int scanEdge(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int maxToAdd);

int sweepRange(int startAng, int endAng, struct robotCoords* botPosition, struct fieldObs* obsLoc, int maxToAdd, int thresh);

void testSweep(void);

void lineScanTest(oi_t *sensor_data);

void printObsData(struct fieldObs* obstacle);
int deleteDuplicates(struct fieldObs obArray[], int arraySize);

int moveAroundObs(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int addLimit, int frontSize, int sideSize){

    turn_bot_left(sensor_data, botPos);
    int moveSide = sideSize * 1.25 + BOT_OFFSET;;
    int numSide = navigateDistance(sensor_data, botPos, obsLoc, addLimit, moveSide);

    lcd_printf("SideMove completed");
    //timer_waitMillis(3000);

    turn_bot_right(sensor_data, botPos);
    int movefront = frontSize * 2 + BOT_OFFSET;
    int numFront = navigateDistance(sensor_data, botPos, obsLoc, addLimit, movefront);



    turn_bot_right(sensor_data, botPos);
    int numBack = navigateDistance(sensor_data, botPos, obsLoc, addLimit, moveSide);

    turn_bot_left(sensor_data, botPos);

    return 0;
}

int navigateDistance(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int addLimit, int goalDist){

    char output[100];

    int goal;
    int goalDir = botPos->direction;

    switch(goalDir){
    case 3:
        goal = botPos->xCoord - goalDist;
        if (goal < 0){
            goal = 0;
        }
        break;

    case 2:
        goal = botPos->yCoord - goalDist;
        if(goal < 0){
            goal = 0;
        }
        break;

    case 1:
        goal = botPos->xCoord + goalDist;
        if(goal > MAP_WIDTH){
            goal = MAP_WIDTH;
        }
        break;

    default:
        goal = botPos->yCoord + goalDist;
        if(goal > MAP_LENGTH){
            goal = MAP_LENGTH;
        }
        break;
    }

    struct fieldObs obArray[5];

    int sweepCount;
    int moveProgress;
    int moveResult;
    int moveInterval = 200;

    do{
        lcd_printf("Coords: (%d,%d)\nGoalDir:%d GoalDist:%d", botPos->yCoord, botPos->xCoord, goalDir, goal);


        sweepCount = sweepRange(45, 135, botPos, &obArray[0], 5, 500);

        if(sweepCount > 0){
            oi_play_song(2);
            //lcd_printf("Scan found item");
            turn_bot_left(sensor_data, botPos);
            //timer_waitMillis(1000);
        }
        else{
            moveResult = move_bot_forward(sensor_data, botPos, moveInterval);
            if(moveResult == 0){
                if(botPos->direction != goalDir){
                    turn_bot_right(sensor_data, botPos);
                }
            }
            else{
                //lcd_printf("Bump/Cliff found item");
                //timer_waitMillis(1000);
                turn_bot_left(sensor_data, botPos);
            }
        }

        if(goalDir % 2 == 0){
            moveProgress = botPos->yCoord;
        }
        else{
            moveProgress = botPos->xCoord;
        }

    }while(abs(moveProgress - goal) > 200);

    lcd_printf("Goal of %d reached\n", goal);
    //timer_waitMillis(1000);

    while(botPos->direction != goalDir){
        turn_bot_right(sensor_data, botPos);
    }
}

int middleScan(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int addLimit){
    int offsetMove = 500;

    int turnDir;

    turn_bot_left(sensor_data, botPos);

    int sweepCount = sweepRange(70, 110, botPos, obsLoc, addLimit, 200);
    int result;

    if(sweepCount < 1){
        result = move_bot_forward(sensor_data, botPos, offsetMove);
        sweepCount = sweepRange(0, 180, botPos, obsLoc, addLimit, 90);
        turn_bot_right(sensor_data, botPos);
        turn_bot_right(sensor_data, botPos);
        move_bot_forward(sensor_data, botPos, offsetMove * 0.9);

        turn_bot_left(sensor_data, botPos);
    }
    else{
        turn_bot_right(sensor_data, botPos);
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
    int sideSize;
    int frontSize;

    while(1){
        if(addLimit <= 0){
            return numObs;
        }

        sweepCount = sweepRange(70, 120, botPos, obsLoc, addLimit, 90);
        numObs += sweepCount;
        obsLoc += sweepCount;
        addLimit -= sweepCount;

        if(sweepCount > 0){
            oi_play_song(2);
            //lcd_printf("%d objects found", sweepCount);
            //move_Around_Object(sensor_data, botPos, 200, obsLoc, addLimit);


            obsLoc--;
            lcd_printf("Object Coords: (%d, %d)", obsLoc->xCoord, obsLoc->yCoord);
            timer_waitMillis(2000);
            switch(botPos->direction){
            case 3://West
                sideSize = MAP_LENGTH - obsLoc->yCoord;
                break;
            case 2://South
                sideSize = obsLoc->xCoord;
                break;
            case 1://East
                sideSize = obsLoc->yCoord;
                break;
            case 0://North
                sideSize = MAP_WIDTH - obsLoc->xCoord;
                break;
            }

            if(botPos->direction % 2 == 0){
                frontSize = abs(botPos->yCoord - obsLoc->yCoord);
            }
            else{
                frontSize = abs(botPos->xCoord - obsLoc->xCoord);
            }

            obsLoc++;


            numObs += moveAroundObs(sensor_data, botPos, obsLoc, addLimit, frontSize, sideSize);
        }

        distToTravel = intervalSize;
        lcd_printf("Running");
        followResult = scanLine(sensor_data, botPos, &obsData, addLimit, &distToTravel);
        updateBotPos(botPos, intervalSize - distToTravel);



        switch(followResult){
        case 4: //If an object is detected
            oi_play_song(2);
            //lcd_printf("Object found: Size %d", obsData.size);
            //timer_waitMillis(1000);

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

            //move_Around_Object(sensor_data, botPos, 610);
            move_bot_backward(sensor_data, botPos, 50);
            sideSize = 610;
            frontSize = 610;
            numObs += moveAroundObs(sensor_data, botPos, obsLoc, addLimit, frontSize, sideSize);
            break;

        case 2: //Short Object Detected.
            oi_play_song(1);
            lcd_printf("Short Object Detected");
            timer_waitMillis(1000);
            addOnEdge(1, botPos, obsLoc);
            numObs++;
            addLimit--;

            //moveAroundObs(sensor_data, botPos, 130);
            move_bot_backward(sensor_data, botPos, 50);
            frontSize = 260;
            sideSize = 130;
            numObs += moveAroundObs(sensor_data, botPos, obsLoc, addLimit, frontSize, sideSize);

            break;

        case 1: //Cybot has reached end of edge
            lcd_printf("End of edge");
            timer_waitMillis(1000);
            return numObs;

        case 0: //Cybot completed the interval without issue.
            lcd_printf("End of interval");
            timer_waitMillis(1000);
            //middleScan(sensor_data, botPos, obsLoc, addLimit);
            break;

        default://If a button is pressed, immediately terminate
            return 0;
        }
    }
    lcd_printf("End of Edge");
    timer_waitMillis(1000);
    return numObs;
}

void scanPerimeter(oi_t *sensor_data){
    struct robotCoords botCoords;
    botCoords.xCoord = 2400;
    botCoords.yCoord = 100;
    botCoords.direction = 0;

    int maxObjects = 30;
    int numObjects = 0;


    char output[100];

    struct fieldObs obsArray[maxObjects];
    int i;

    for(i = 0; i < 4; i++){
        numObjects += scanEdge(sensor_data, &botCoords, &obsArray[0], maxObjects);
        turn_bot_left(sensor_data, &botCoords);
    }

    int maxObjectSize = 0;
    struct fieldObs largestObs;

    for(i = 0; i < numObjects; i++){
        if(obsArray[i].sizeMM > maxObjectSize && obsArray[i].itemType != 0){
            largestObs = obsArray[i];
            maxObjectSize = obsArray[i].sizeMM;
        }

        sprintf(output, "Object %d: X:%d Y:%d Size:%d\n", i, obsArray[i].xCoord, obsArray[i].yCoord, obsArray[i].sizeMM);
        uart_sendStr(output);
    }

    sprintf(output, "\nThe largest object is %d mm, located at (%d, %d)\n", largestObs.sizeMM, largestObs.xCoord, largestObs.yCoord);
    uart_sendStr(output);

    int xGoal = largestObs.xCoord;
    int yGoal = largestObs.yCoord;
    int navDir;
    while(1){

        if(abs(botCoords.xCoord - xGoal) < 100 && abs(botCoords.yCoord - yGoal) < 100){
            return;
        }
        if(abs(botCoords.xCoord - xGoal) > 100){
            if(botCoords.xCoord - xGoal > 0){
                navDir = 3;
            }
            else{
                navDir = 1;
            }
            while(navDir != botCoords.direction){
                turn_bot_left(sensor_data, &botCoords);
            }
            navigateDistance(sensor_data, &botCoords, &obsArray[0],  5, abs(botCoords.xCoord - xGoal));
        }

        if(abs(botCoords.yCoord - yGoal) > 100){
            if(botCoords.yCoord - yGoal > 0){
                navDir = 2;
            }
            else{
                navDir = 0;
            }
            while(navDir != botCoords.direction){
                turn_bot_left(sensor_data, &botCoords);
            }
            navigateDistance(sensor_data, &botCoords, &obsArray[0],  5, abs(botCoords.yCoord - yGoal));
        }

    }
}

int sweepRange(int startAng, int endAng, struct robotCoords* botPosition, struct fieldObs* obsLoc, int maxToAdd, int thresh){
    int currentAngle = startAng;
    int obsCount = 0;
    struct obstacle obsArray[maxToAdd];
    //char output[100];

    //uart_sendStr("Starting Scan\n");

    while(currentAngle < endAng && obsCount < maxToAdd){
        obsCount += sweepNextObs(&obsArray[obsCount], &currentAngle, endAng, thresh);
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

    numObjects += sweepRange(0, 180, &testCoords, &obsArray[0], maxObjects - numObjects, 90);

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


    char output[100];

    int maxObjects = 20;
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

int deleteDuplicates(struct fieldObs obArray[], int arraySize){
    int i;
    int distThresh = 100;
    int sizeThresh = 50;

    int arrSize = arraySize;

    for(i = 0; i < arrSize - 1; i++){
        struct fieldObs firstObs = obArray[i];
        int j;
        for(j = i+1; j < arrSize; j++){
            struct fieldObs secObs = obArray[j];

            int xDiff = abs(firstObs.xCoord - secObs.xCoord);
            int yDiff = abs(firstObs.yCoord - secObs.yCoord);
            int sizeDiff = abs(firstObs.sizeMM - secObs.sizeMM);

            if(xDiff < distThresh && yDiff < distThresh && sizeDiff < sizeThresh){
                int k;
                for(k = j; k < arraySize; k++){
                    obArray[k] = obArray[k+1];
                }
                arrSize--;
            }
        }
    }
    return arrSize;
}
