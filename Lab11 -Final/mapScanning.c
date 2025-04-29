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


    int forwardsMove = 600;
    int sidewaysMove = 400;

    int travelDist;
    int followResult;

    timer_waitMillis(500);

    followResult = 0;


    int intervalSize = 500;



    while(followResult == 0){
        travelDist = intervalSize;
        followResult = followLine(sensor_data, &travelDist);

        if(followResult == -1){
            return;
        }

        totalDist += intervalSize - travelDist;

        arraySize = ir_scanRange(irArray, 0, 90, 2);
        numObjects = scan_containsObject(irArray, arraySize, 200);
        if(numObjects != 0){
            lcd_printf("%d Objects detected", numObjects);
            timer_waitMillis(5000);
        }

        if(followResult == 2){
            if(totalDist + forwardsMove + 100> expectedLength){
                cutCorner(sensor_data, 0, sidewaysMove);
            }
            else{
                move_aroundObject(sensor_data, 0, sidewaysMove, forwardsMove);
            }

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

        turn_left(sensor_data, 90);
    }

    numSides++;

    lcd_printf("Total Distance: %d", totalDist);
}
