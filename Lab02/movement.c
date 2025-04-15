/*
 * movement.c
 *
 *  Created on: Feb 6, 2025
 *      Author: jagaul
 */

#include <open_interface.h>
#include <lcd.h>

#define MOVE_SPEED 100
#define FORWARD_ADJUST 1.01
#define BACKWARD_ADJUST 1.01
#define RIGHT_ADJUST 0.90
#define LEFT_ADJUST 0.90



double move_forward(oi_t *sensor_data, double distance_mm){

    double currDist = 0.0;

    lcd_init();
    oi_setWheels(MOVE_SPEED, MOVE_SPEED);

    //Move until distance reaches assigned distance
    while (currDist < distance_mm * FORWARD_ADJUST){
        oi_update(sensor_data);
        currDist += sensor_data -> distance;
        lcd_printf("Distance: %lf", currDist);
    }
    oi_setWheels(0,0);
    return currDist;
}

double move_backward(oi_t *sensor_data, double distance_mm){

    double currDist = 0.0;

    lcd_init();
    oi_setWheels(-MOVE_SPEED, -MOVE_SPEED);

    //Move until distance reaches assigned distance
    while (currDist > -distance_mm * BACKWARD_ADJUST){
        oi_update(sensor_data);
        currDist += sensor_data -> distance;
        lcd_printf("Distance: %lf", currDist);
    }
    oi_setWheels(0,0);
    return currDist;
}

double move_forward_bumpInt(oi_t *sensor_data, double distance_mm){
    double currDist = 0.0;
    oi_setWheels(MOVE_SPEED, MOVE_SPEED);

    lcd_init();

    while(currDist < distance_mm * FORWARD_ADJUST){
        oi_update(sensor_data);
        currDist += sensor_data -> distance;
        lcd_printf("Distance: %lf", currDist);
        if((sensor_data -> bumpLeft)||(sensor_data->bumpRight)){
            break;
        }
    }

    oi_setWheels(0,0);
    return currDist;
}

double turn_right (oi_t *sensor_data, double degrees){
    double currAng = 0.0;
        lcd_init();

        oi_setWheels(-MOVE_SPEED, MOVE_SPEED);


        //Move until distance reaches assigned distance
        while (currAng > -degrees * RIGHT_ADJUST){
            oi_update(sensor_data);
            currAng += sensor_data -> angle;
            lcd_printf("Angle: %lf", currAng);
        }
        oi_setWheels(0,0);
        return currAng;
}

double turn_left (oi_t *sensor_data, double degrees){
    double currAng = 0.0;
    lcd_init();
            oi_setWheels(MOVE_SPEED, -MOVE_SPEED);


            //Move until distance reaches assigned distance
            while (currAng < degrees * LEFT_ADJUST){
                oi_update(sensor_data);
                currAng += sensor_data -> angle;
                lcd_printf("%lf", currAng);
            }
            oi_setWheels(0,0);
            return currAng;
}
