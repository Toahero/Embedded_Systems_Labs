/*
 * moveAndScan.c
 *
 *  Created on: Apr 25, 2025
 *      Author: jagaul
 */
#include <open_interface.h>
#include <lcd.h>
#include "Timer.h"
#include "uart-interrupt.h"

#define MOVE_SPEED 100
#define FORWARD_ADJUST 1.01
#define IR_OBJECT_THRESHOLD 90

int move_forward_detect(oi_t *sensor_data, int* distance_mm){
    oi_setWheels(MOVE_SPEED, MOVE_SPEED);

    uint16_t i;
    int irValue;
    int prevIR;

    while(*distance_mm > 0){
        oi_update(sensor_data);
        *distance_mm -= sensor_data -> distance * FORWARD_ADJUST;
        lcd_printf("%.2f", *distance_mm);

        servo_move(45);
        irValue = adc_read();
        for(i = 45; i < 135; i++){
            prevIR = irValue;
            servo_move(i);
            irValue = adc_read();

            if(prevIR - irValue > IR_OBJECT_THRESHOLD){
                oi_setWheels(0,0);
                return 5;
            }

            if(sensor_data -> bumpLeft){
                oi_setWheels(0, 0);
                return 1;
            }
            if(sensor_data->bumpRight){
                oi_setWheels(0, 0);
                return 2;
            }

            /*if(sensor_data->cliffFrontLeftSignal > 2500){
                oi_setWheels(0, 0);
                return 3;
            }

            if(sensor_data->cliffFrontRightSignal > 2500){
                oi_setWheels(0, 0);
                return 4;
            }*/
        }


    }
    oi_setWheels(0, 0);
    return 0;
}
