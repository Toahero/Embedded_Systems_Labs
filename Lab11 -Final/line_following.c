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

/*2041-09
 * L-  U:2816 L:2197
 * FL- U:2758 L:1460
 * FR- U:2752 L:1543
 * R-  U:2761 L:1766
 */

/*2041-12
 * L-  U:2811 L:2103
 * FL- U:2863 L:1460
 * FR- U:2841 L:2133
 * R-  U:2866 L:2310
 */




volatile uint16_t cl_upper = 2816;
volatile uint16_t cl_lower = 2197;

volatile uint16_t cfl_upper = 2758;
volatile uint16_t cfl_lower = 2452;

volatile uint16_t cfr_upper = 2752;
volatile uint16_t cfr_lower = 1543;

volatile uint16_t cr_upper = 2761;
volatile uint16_t cr_lower = 1766;

#define MOVE_SPEED 100

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
                move_aroundObject(sensor_data, 0, 400, 600);
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

int scanLine(oi_t *sensor_data, struct robotCoords* botPos, struct fieldObs* obsLoc, int maxToAdd, int* distance_mm)  {
    int adjust = 0;
    int numScans = 3;
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

    //Before starting to follow the line, turn the sensor rightward and fill the buffer
    int scanBuffer[IR_BUFFER_SIZE];
    int i;
    for(i = 0; i < IR_BUFFER_SIZE; i++){
       scanBuffer[i] = multiScanIR(0, numScans);
    }

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
