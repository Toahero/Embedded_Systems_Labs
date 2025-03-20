

/**
 * main.c
 */
#include <open_interface.h>
#include <movement.h>

void driveSquareRight(oi_t *sensor_data, int sideLengthCm);
void driveSquareLeft(oi_t *sensor_data, int sideLengthCm);

void forward_mm_detours(oi_t *sensor_data, int distance_mm);

void main() {
        oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
        oi_init(sensor_data); // do this only once at start of main()

        //driveSquareLeft(sensor_data, 50);
        //move_forward(sensor_data, 1000);
        forward_mm_detours(sensor_data, 2000);

        oi_free(sensor_data); // do this once at end of main()
}

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
}
