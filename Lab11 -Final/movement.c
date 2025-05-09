/*
 * movement.c
 *
 *  Created on: Feb 6, 2025
 *      Author: jagaul
 */

#include <open_interface.h>
#include <lcd.h>
#include "Timer.h"
#include "uart-interrupt.h"
#include "sharedStructs.h"

#define MOVE_SPEED 200
#define FORWARD_ADJUST 0.97
#define BACKWARD_ADJUST 0.97
#define RIGHT_ADJUST 0.962
#define LEFT_ADJUST 0.962

#define LEFT_MOTOR_ADJ 1.0
#define RIGHT_MOTOR_ADJ 1.0

#define MAP_LENGTH 4270
#define MAP_WIDTH 2440
#define BOT_OFFSET 300

double move_forward(oi_t *sensor_data, double distance_mm);

int move_bot_forward(oi_t *sensor_data, struct robotCoords* botPos, int distance_mm);

int move_bot_backward(oi_t *sensor_data, struct robotCoords* botPos, int distance_mm);

void right_angle_turn(int turnDir, oi_t *sensor_data, struct robotCoords* botPos);

void turn_bot_right(oi_t *sensor_data, struct robotCoords* botPos);

void turn_bot_left (oi_t *sensor_data, struct robotCoords* botPos);

double move_backward(oi_t *sensor_data, double distance_mm);

double move_forward_bumpInt(oi_t *sensor_data, double distance_mm);

double turn_right (oi_t *sensor, double degrees);

double turn_left (oi_t *sensor, double degrees);

void forward_mm_redirect(oi_t *sensor_data, int distance_mm);

void forward_mm_detours(oi_t *sensor_data, int distance_mm);

int forward_mm_nav(oi_t *sensor_data, int* distance_mm);

int cutCorner(oi_t *sensor_data, int turnDirection, int sidewaysMove);

void right_angle_turn(int turnDir, oi_t *sensor_data, struct robotCoords* botPos){
    if(turnDir){
        turn_bot_right(sensor_data, botPos);
    }
    else{
        turn_bot_left(sensor_data, botPos);
    }
}

int move_bot_forward(oi_t *sensor_data, struct robotCoords* botPos, int distance_mm){

    int result = 0;
    double currDist = 0.0;

    oi_setWheels(MOVE_SPEED * LEFT_MOTOR_ADJ, MOVE_SPEED * RIGHT_MOTOR_ADJ);
    while (currDist/FORWARD_ADJUST < distance_mm){
        if(sensor_data -> bumpLeft){
            oi_setWheels(0, 0);
            result = 1;
            break;
        }

        if(sensor_data->bumpRight){
            result = 4;
            break;
        }

        if(sensor_data->cliffFrontLeftSignal < 1000 || sensor_data->cliffFrontRightSignal < 1000){
            result = 2;
            break;
        }

        if(sensor_data->cliffFrontLeftSignal > 2500){
            oi_setWheels(0, 0);
            result = 2;
            break;
        }

        if(sensor_data->cliffFrontRightSignal > 2500){
            oi_setWheels(0, 0);
            result = 3;
            break;
        }

        oi_update(sensor_data);
        currDist += sensor_data -> distance;
    }
    oi_setWheels(0,0);
    updateBotPos(botPos, (int) currDist/FORWARD_ADJUST);
    return result;
}

int move_bot_backward(oi_t *sensor_data, struct robotCoords* botPos, int distance_mm){

    double currDist = 0.0;

    oi_setWheels(-MOVE_SPEED * LEFT_MOTOR_ADJ, -MOVE_SPEED * RIGHT_MOTOR_ADJ);
    while (currDist/FORWARD_ADJUST > -distance_mm){
        oi_update(sensor_data);
        currDist += sensor_data -> distance;
    }
    oi_setWheels(0,0);
    updateBotPos(botPos, (int) currDist/FORWARD_ADJUST);
    return 0;
}

void turn_bot_right(oi_t *sensor_data, struct robotCoords* botPos){
    double currAng = 0.0;
    oi_setWheels(-MOVE_SPEED, MOVE_SPEED);

    //Move until distance reaches assigned distance
    while (currAng > -90 * RIGHT_ADJUST){
        oi_update(sensor_data);
        currAng += sensor_data -> angle;
    }
    oi_setWheels(0,0);
    botPos->direction = (botPos->direction + 1) % 4;
}

void turn_bot_left (oi_t *sensor_data, struct robotCoords* botPos){
    double currAng = 0.0;
    oi_setWheels(MOVE_SPEED, -MOVE_SPEED);


    //Move until distance reaches assigned distance
    while (currAng < 90 * LEFT_ADJUST){
        oi_update(sensor_data);
        currAng += sensor_data -> angle;
    }
    oi_setWheels(0,0);
    //To reduce the angle, add 3 then take the factorial
    botPos->direction = (botPos->direction + 3) % 4;
}



double move_forward(oi_t *sensor_data, double distance_mm){

    double currDist = 0.0;

    //lcd_init();
    oi_setWheels(MOVE_SPEED * LEFT_MOTOR_ADJ, MOVE_SPEED * RIGHT_MOTOR_ADJ);

    //Move until distance reaches assigned distance
    while (currDist < distance_mm * FORWARD_ADJUST){
        oi_update(sensor_data);
        currDist += sensor_data -> distance;
        //lcd_printf("Distance: %lf", currDist);
    }
    oi_setWheels(0,0);
    return currDist;
}

double move_backward(oi_t *sensor_data, double distance_mm){

    double currDist = 0.0;

    //lcd_init();
    oi_setWheels(-MOVE_SPEED, -MOVE_SPEED);

    //Move until distance reaches assigned distance
    while (currDist > -distance_mm * BACKWARD_ADJUST){
        oi_update(sensor_data);
        currDist += sensor_data -> distance;
        //lcd_printf("Distance: %lf", currDist);
    }
    oi_setWheels(0,0);
    return currDist;
}

double move_forward_bumpInt(oi_t *sensor_data, double distance_mm){
    double currDist = 0.0;
    oi_setWheels(MOVE_SPEED, MOVE_SPEED);

    //lcd_init();

    while(currDist < distance_mm * FORWARD_ADJUST){
        oi_update(sensor_data);
        currDist += sensor_data -> distance;
        //lcd_printf("Distance: %lf", currDist);
        if((sensor_data -> bumpLeft)||(sensor_data->bumpRight)){
            break;
        }
    }

    oi_setWheels(0,0);
    return currDist;
}

double turn_right (oi_t *sensor_data, double degrees){
    double currAng = 0.0;
    //lcd_init();

    oi_setWheels(-MOVE_SPEED, MOVE_SPEED);


    //Move until distance reaches assigned distance
    while (currAng > -degrees * RIGHT_ADJUST){
        oi_update(sensor_data);
        currAng += sensor_data -> angle;
        lcd_printf("Angle: %.2f\n Thresh: %.2f", currAng, -degrees * RIGHT_ADJUST);
    }
    oi_setWheels(0,0);
    return currAng;
}

double turn_left (oi_t *sensor_data, double degrees){
    double currAng = 0.0;
    //lcd_init();
            oi_setWheels(MOVE_SPEED, -MOVE_SPEED);


            //Move until distance reaches assigned distance
            while (currAng < degrees * LEFT_ADJUST){
                oi_update(sensor_data);
                currAng += sensor_data -> angle;
                //lcd_printf("%lf", currAng);
            }
            oi_setWheels(0,0);
            return currAng;
}

void forward_mm_redirect(oi_t *sensor_data, int distance_mm){
    double forwardMotion = 0.0;
    double distanceMoved = 0.0;


    //While the robot has not travelled the full distance
    while (forwardMotion < distance_mm){
        //Try to move forward the full distance.
        distanceMoved = forwardMotion;
        forwardMotion += move_forward_bumpInt(sensor_data, distance_mm - distanceMoved);

        //If the distance isn't far enough
        if(forwardMotion < distance_mm){
            //Check if the left bumper is triggered
            if(sensor_data -> bumpLeft){
                forwardMotion += move_backward(sensor_data, 50);
                turn_right(sensor_data, 90);
                move_forward(sensor_data, 100);
                turn_left(sensor_data, 135);
                return;
            }
            //Otherwise, assume the right sensor was triggered
            else{
                forwardMotion += move_backward(sensor_data, 50);
                turn_left(sensor_data, 90);
                move_forward(sensor_data, 100);
                turn_right(sensor_data, 135);
                return;
            }
        }
    }
}

void forward_mm_detours(oi_t *sensor_data, int distance_mm){
    double forwardMotion = 0.0;
    double distanceMoved = 0.0;

    int turnChange = 0;


    //While the robot has not travelled the full distance
    while (forwardMotion < distance_mm){
        //Try to move forward the full distance.
        distanceMoved = forwardMotion;
        forwardMotion += move_forward_bumpInt(sensor_data, distance_mm - distanceMoved);

        //If the distance isn't far enough
        if(forwardMotion < distance_mm){
            //Check if the left bumper is triggered
            if(sensor_data -> bumpLeft){
                forwardMotion += move_backward(sensor_data, 150);
                turn_right(sensor_data, 90);
                move_forward(sensor_data, 250);
                turn_left(sensor_data, 90);
                turnChange += 90;
            }
            //Otherwise, assume the right sensor was triggered
            else{
                forwardMotion += move_backward(sensor_data, 150);
                        turn_left(sensor_data, 90);
                        move_forward(sensor_data, 250);
                        turn_right(sensor_data, 90);
                        turnChange -= 90;
            }
        }
    }


    //Expirimental: remove if this doesn't work.
    if(turnChange > 0){
        turn_right(sensor_data, turnChange);
    }

    if(turnChange < 0){
        turn_left(sensor_data, -turnChange);
    }
}

int forward_mm_nav(oi_t *sensor_data, int* distance_mm){
    double currDist = 0;
    oi_setWheels(MOVE_SPEED, MOVE_SPEED);

    while(*distance_mm > 0){
        oi_update(sensor_data);
        *distance_mm -= sensor_data -> distance * FORWARD_ADJUST;
        lcd_printf("Distance: %d", *distance_mm);

        if(sensor_data -> bumpLeft){
            oi_setWheels(0, 0);
            return 1;
        }
        if(sensor_data->bumpRight){
            oi_setWheels(0, 0);
            return 2;
        }

        if(sensor_data->cliffFrontLeftSignal > 2500){
            return 3;
        }

        if(sensor_data->cliffFrontRightSignal > 2500){
            return 4;
        }
    }
    oi_setWheels(0, 0);
    return 0;
}
