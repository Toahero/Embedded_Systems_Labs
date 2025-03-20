/*
 * movement_functions.c
 *
 *  Created on: Feb 12, 2025
 *      Author: jagaul
 */

#include <movement.h>

void driveSquareRight(oi_t *sensor_data, int sideLengthCm){

    int i = 0;
    for(i = 0; i < 4; i++){
        move_forward(sensor_data, sideLengthCm * 10);
        turn_right(sensor_data, 90);
    }
}

void driveSquareLeft(oi_t *sensor_data, int sideLengthCm){
    int i = 0;
        for(i = 0; i < 4; i++){
            move_forward(sensor_data, sideLengthCm * 10);
            turn_left(sensor_data, 90);
        }
}
/*
void forward_mm_detours(oi_t *sensor_data, int distance_mm){
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
                        forwardMotion += move_backward(sensor_data, 150);
                        turn_right(sensor_data, 90);
                        move_forward(sensor_data, 250);
                        turn_left(sensor_data, 90);
            }
            //Otherwise, assume the right sensor was triggered
            else{
                forwardMotion += move_backward(sensor_data, 150);
                        turn_left(sensor_data, 90);
                        move_forward(sensor_data, 250);
                        turn_right(sensor_data, 90);
            }
        }
    }
}*/
