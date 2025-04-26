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

#define CR_LOWER_AVG 1400

#define CR_UPPER_AVG 2500

#define CFR_UPPER_AVG 2800
#define CFL_UPPER_AVG 2800

volatile uint16_t cr_upper = 2759;
volatile uint16_t cr_lower = 1239;

volatile uint16_t cfr_upper = 2873;
volatile uint16_t cfr_lower = 1948;

volatile uint16_t cfl_upper = 2778;
volatile uint16_t cfl_lower = 1327;

volatile uint16_t cl_upper = 2758;
volatile uint16_t cl_lower = 2214;

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

int followLine(oi_t *sensor_data, int* distance_mm){
    int adjust = 0;

    oi_setWheels(MOVE_SPEED, MOVE_SPEED);
    int button;
    int cliffR, cliffFR, cliffFL;

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
            return -1;
        }



        if(*distance_mm < 0){
            return 0;
        }

        if((cliffFR > cfr_upper - 400 && cliffFL > cfl_upper - 400)){
            oi_setWheels(0, 0);
            return 1;
        }
        if(sensor_data->bumpLeft || sensor_data-> bumpRight){
            return 2;
        }

        //If sensor FR senses an edge and R does not, the cybot is likely approaching the edge at a high angle. Immediately turn away.
        if(cliffFR > cfr_upper - 400 && !(cliffR > cr_upper - 400)){
            turn_left(sensor_data, 30);
        }

        if(cliffR < cr_lower + 400){
            adjust = 25;
        }
        else if(cliffR > cr_upper - 400){
            adjust = -25;
        }
        else{
            adjust = 0;
        }

        leftSpeed = MOVE_SPEED - adjust;
        rightSpeed = MOVE_SPEED + adjust;

        oi_setWheels(leftSpeed, rightSpeed);
        lcd_printf("L: %d  R: %d \nA: %d", leftSpeed, rightSpeed, adjust);
        timer_waitMillis(50);
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
    timer_waitMillis(50);
}
