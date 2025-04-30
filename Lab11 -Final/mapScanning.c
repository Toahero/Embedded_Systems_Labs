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

    int maxObjects = 10;
    float objectSizes[maxObjects];
    float horizOffsets[maxObjects];
    float vertOffsets[maxObjects];

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
                numObjects = locateObjects(45, 135, objectSizes, horizOffsets, vertOffsets, maxObjects);
                for(i = 0; i < numObjects; i++){
                    lcd_printf("X Offset:%.2f\nY Offset:%.2f\nWidth:%.2f", horizOffsets[i], vertOffsets[i], objectSizes[i]);
                    timer_waitMillis(500);
                }
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
