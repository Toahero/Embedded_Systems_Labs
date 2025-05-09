/*
 * line_following.c
 *
 *  Created on: Apr 26, 2025
 *      Author: jagaul
 */
#include <inc/tm4c123gh6pm.h>
#include <open_interface.h>
#include <lcd.h>
#include "Timer.h"
#include "button.h"
#include "movement.h"
#include "scanFunctions.h"
#include "sharedStructs.h"


#define IR_BUFFER_SIZE 5
#define IR_OBJECT_THRESHOLD 45
#define MIN_OBJECT_SIZE 50
/*2041-06
 * L-  U:2757 L:1616
 * FL- U:2836 L:2343
 * FR- U:2779 L:1699
 * R-  U:2790 L:1999
 */

/*2041-09
 * L-  U:2816 L:2197
 * FL- U:2758 L:1460
 * FR- U:2752 L:1543
 * R-  U:2761 L:1766
 */

/*2041-10
 * L-  U:2838 L:2192
 * FL- U:2763 L:1378
 * FR- U:2851 L:2145
 * R-  U:2733 L:1273
 */

/*2041-12
 * L-  U:2811 L:2103
 * FL- U:2863 L:1460
 * FR- U:2841 L:2133
 * R-  U:2866 L:2310
 */




volatile uint16_t cl_upper = 2811;
volatile uint16_t cl_lower = 2103;

volatile uint16_t cfl_upper = 2863;
volatile uint16_t cfl_lower = 2145;

volatile uint16_t cfr_upper = 2841;
volatile uint16_t cfr_lower = 2133;

volatile uint16_t cr_upper = 2866;
volatile uint16_t cr_lower = 2310;

#define MOVE_SPEED 75

void followPerimeter(oi_t *sensor_data){
    int totalDist;
    int numSides = 0;
    int rightCliff, leftCliff;
    int expectedDist = 2000;
    int travelDist;
    int followResult;

    timer_waitMillis(500);

    while(numSides < 4){

        followResult = 0;

        while(followResult != 1){
            travelDist = expectedDist;
            followResult = followLine(sensor_data, &travelDist);

            if(followResult == -1){
                return;
            }

            totalDist += expectedDist - travelDist;
            if(followResult == 2){
                //move_aroundObject(sensor_data, 0, 400, 600);
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
    }

    lcd_printf("Total Distance: %d", totalDist);
}

int scanLine(oi_t *sensor_data, struct robotCoords* botPos, struct obSide* foundObs, int maxToAdd, int* distance_mm)  {
    int adjust = 0;
    int numScans = 3;

    int button = 0;
    int cliffR, cliffFR, cliffFL;
    int scanAngle = 180;

    float boundMod = 0.25;

    //char tempOUT[100];


    //Set boundary values
    int cr_UpperBound = cr_upper - ((cr_upper - cr_lower) * boundMod);
    int cr_LowerBound = cr_lower + ((cr_upper -cr_lower) * boundMod);

    int cfr_UpperBound = cfr_upper - ((cfr_upper - cfr_lower) * boundMod);
    int cfr_LowerBound = cfr_lower + ((cfr_upper -cfr_lower) * boundMod);
    int cfr_cliffBound = cfr_lower * 0.5;

    int cfl_UpperBound = cfl_upper - ((cfl_upper - cfl_lower) * boundMod);
    int cfl_LowerBound = cfl_lower + ((cfl_upper -cfl_lower) * boundMod);
    int cfl_cliffBound = cfl_lower * 0.5;

    int cl_UpperBound = cl_upper - ((cl_upper - cl_lower) * boundMod);
    int cl_LowerBound = cl_lower + ((cl_upper -cl_lower) * boundMod);

    int leftSpeed, rightSpeed;

    //Before starting to follow the line, turn the sensor rightward and fill the buffer
    int scanBuffer[IR_BUFFER_SIZE];
    int i;
    for(i = 0; i < IR_BUFFER_SIZE; i++){
       scanBuffer[i] = multiScanIR(scanAngle, numScans);
    }

    //Set variables for the object search
    int irScan;
    int scanSum;
    int scanAvg;
    int startPos;
    int endPos;
    int obsFound = 0;


    while(1){
        oi_update(sensor_data);

        *distance_mm -= sensor_data->distance;

        oi_setWheels(MOVE_SPEED, MOVE_SPEED);
        scanSum = 0;
        for(i = 0; i < IR_BUFFER_SIZE; i++){
            scanSum += scanBuffer[i];
        }
        scanAvg = scanSum / IR_BUFFER_SIZE;
        irScan = multiScanIR(scanAngle, numScans);

        if(irScan >= scanAvg + IR_OBJECT_THRESHOLD && !obsFound){
            startPos = *distance_mm;
            obsFound = 1;
        }

        if(irScan <= scanAvg - IR_OBJECT_THRESHOLD && obsFound){
            obsFound = 0;
            if(startPos - *distance_mm > MIN_OBJECT_SIZE){
                //Record the start and end positions of the object

                endPos = *distance_mm;


                foundObs->size = startPos - endPos;

                //Move backwards to the object midpoint
                //TODO: Possibly create backwards line following
                move_backward(sensor_data, foundObs->size/2);

                //Get the distance to the object.
                foundObs->midDist = pingAt(scanAngle) * 10;
                move_forward(sensor_data, foundObs->size/2);
                oi_setWheels(0, 0);
                //Return indicator that another object has been found.
                return 4;
            }
        }

        button = button_getButton();
        cliffR = sensor_data->cliffRightSignal;
        cliffFR = sensor_data->cliffFrontRightSignal;
        cliffFL = sensor_data->cliffFrontLeftSignal;

        if(button != 0){
            oi_setWheels(0, 0);
            return -1;
        }

        if(*distance_mm < 0 && !obsFound){
            oi_setWheels(0, 0);
            return 0;
        }

        if(*distance_mm < -100){
            oi_setWheels(0, 0);
            return 0;
        }

        if(cliffFR > cfr_UpperBound && cliffFL > cfl_UpperBound){
            oi_setWheels(0, 0);
            return 1;
        }
        if(sensor_data->bumpLeft || sensor_data-> bumpRight){
            oi_setWheels(0, 0);
            return 2;
        }

        if(cliffFR < cfr_cliffBound || cliffFL < cfl_cliffBound){
            oi_setWheels(0, 0);
            return 3;
        }

        //If sensor FR senses an edge and R does not, the cybot is likely approaching the edge at a high angle. Immediately turn away.
        if(cliffFR > cfr_UpperBound && !(cliffR > cr_UpperBound)){
            turn_left(sensor_data, 30);
        }

        if(cliffR < cr_LowerBound){
            adjust = 25;
        }
        else if(cliffR > cr_UpperBound){
            adjust = -25;
        }
        else{
            adjust = 0;
        }

        leftSpeed = MOVE_SPEED - adjust;
        rightSpeed = MOVE_SPEED + adjust;

        oi_setWheels(leftSpeed, rightSpeed);
        //lcd_printf("L: %d  R: %d \nA: %d", leftSpeed, rightSpeed, adjust);
        //timer_waitMillis(50);
    }
}

int followLine(oi_t *sensor_data, int* distance_mm){
    int adjust = 0;

    oi_setWheels(MOVE_SPEED, MOVE_SPEED);
    int button;
    int cliffR, cliffFR, cliffFL;


    float boundMod = 0.25;

    //Set boundary values
    int cr_UpperBound = cr_upper - ((cr_upper - cr_lower) * boundMod);
    int cr_LowerBound = cr_lower + ((cr_upper -cr_lower) * boundMod);

    int cfr_UpperBound = cfr_upper - ((cfr_upper - cfr_lower) * boundMod);
    int cfr_LowerBound = cfr_lower + ((cfr_upper -cfr_lower) * boundMod);
    int cfr_cliffBound = cfr_lower * 0.5;

    int cfl_UpperBound = cfl_upper - ((cfl_upper - cfl_lower) * boundMod);
    int cfl_LowerBound = cfl_lower + ((cfl_upper -cfl_lower) * boundMod);
    int cfl_cliffBound = cfl_lower * 0.5;

    int cl_UpperBound = cl_upper - ((cl_upper - cl_lower) * boundMod);
    int cl_LowerBound = cl_lower + ((cl_upper -cl_lower) * boundMod);

    int leftSpeed, rightSpeed;
    while(1){
        button = button_getButton();
        if(button != 0){
            break;
        }

        oi_update(sensor_data);

        *distance_mm -= sensor_data->distance;

        cliffR = sensor_data->cliffRightSignal;
        cliffFR = sensor_data->cliffFrontRightSignal;
        cliffFL = sensor_data->cliffFrontLeftSignal;

        if(button != 0){
            oi_setWheels(0, 0);
            return -1;
        }

        if(*distance_mm < 0){
            oi_setWheels(0, 0);
            return 0;
        }

        if(cliffFR > cfr_UpperBound && cliffFL > cfl_UpperBound){
            oi_setWheels(0, 0);
            return 1;
        }
        if(sensor_data->bumpLeft || sensor_data-> bumpRight){
            oi_setWheels(0, 0);
            return 2;
        }

        if(cliffFR < cfr_cliffBound || cliffFL < cfl_cliffBound){
            oi_setWheels(0, 0);
            return 3;
        }

        //If sensor FR senses an edge and R does not, the cybot is likely approaching the edge at a high angle. Immediately turn away.
        if(cliffFR > cfr_UpperBound && !(cliffR > cr_UpperBound)){
            turn_left(sensor_data, 30);
        }

        if(cliffR < cr_LowerBound){
            adjust = 25;
        }
        else if(cliffR > cr_UpperBound){
            adjust = -25;
        }
        else{
            adjust = 0;
        }

        leftSpeed = MOVE_SPEED - adjust;
        rightSpeed = MOVE_SPEED + adjust;

        oi_setWheels(leftSpeed, rightSpeed);
        //lcd_printf("L: %d  R: %d \nA: %d", leftSpeed, rightSpeed, adjust);
        //timer_waitMillis(50);
    }
}

void calibrate_CliffValue(oi_t *sensor_data){
    int button = 0;
    lcd_printf("Place the Cybot on a white surface, then press button 1");
    while(button != 1){
        button = button_getButton();
    }
    oi_update(sensor_data);

    cr_upper = sensor_data->cliffRightSignal;
    cfr_upper = sensor_data->cliffFrontRightSignal;
    cfl_upper = sensor_data->cliffFrontLeftSignal;
    cl_upper = sensor_data->cliffLeftSignal;


    button = 0;

    lcd_printf("Place the Cybot on the floor, then press button 1");
    timer_waitMillis(500);

    while(button != 1){
        button = button_getButton();
    }
    oi_update(sensor_data);
    cr_lower = sensor_data->cliffRightSignal;
    cfr_lower = sensor_data->cliffFrontRightSignal;
    cfl_lower = sensor_data->cliffFrontLeftSignal;
    cl_lower = sensor_data->cliffLeftSignal;

}

void displayCliffVals(){
    lcd_printf("L- U:%d L:%d\nFL- U: %d L:%d\nFR- U:%d L:%d\nR- U:%d L:%d", cl_upper, cl_lower, cfl_upper, cfl_lower, cfr_upper, cfr_lower, cr_upper, cr_lower);
    //lcd_printf("R- U:%d L:%d", cr_upper, cr_lower);
    timer_waitMillis(1000);
    int button = 0;
    while(button == 0){
        button = button_getButton();
    }
}
