/*
 * movementCalib.c
 *
 *  Created on: Apr 25, 2025
 *      Author: jagaul
 */

#include "adc.h"
#include "servo.h"
#include <open_interface.h>
#include <lcd.h>
#include "Timer.h"
#include "uart-interrupt.h"
#include "ping.h"

#define MOVE_SPEED 100

double getBackwardAdjust(oi_t *sensor_data){
    oi_setWheels(MOVE_SPEED, MOVE_SPEED);

    while(1){
        oi_update(sensor_data);
        if((sensor_data -> bumpLeft)&&(sensor_data->bumpRight)){
            break;
        }
    }
    oi_setWheels(0, 0);

    servo_move(90);
    double pingMin = ping_getDistance();

    oi_setWheels(-MOVE_SPEED, -MOVE_SPEED);

    double currDist = 0.0;
    while (currDist > -1000.0){
        oi_update(sensor_data);
        currDist += sensor_data -> distance;
        lcd_printf("Moved back: %.2f", currDist);
    }
    oi_setWheels(0,0);

    double pingMax = ping_getDistance();

    double pingChange = pingMax-pingMin;

    lcd_printf("Min: %.5f\nMax: %.5f\nDist: %.5f", pingMin, pingMax, pingChange);

    return 100/pingChange;
}

void adjMotorModifiers(double* leftAdj, double* rightAdj){
    servo_move(0);
}
