/*
 * dataCollection.c
 *
 *  Created on: Apr 23, 2025
 *      Author: jagaul
 */
#include <open_interface.h>
#include <lcd.h>
#include "Timer.h"
#include "uart-interrupt.h"


void collect_cliffSignals(oi_t *sensor_data, int distance_mm){
    double currDist = 0.0;

    //lcd_init();
    char output[50];
    command_byte0 = ' ';

    int leftCliff;
    int rightCliff;

    uart_get();

    while(1){
        if(command_flag0){
            command_flag0 = 0;
            break;
        }

        oi_setWheels(100, 100);
        //Move forward
        while (currDist < distance_mm){
            oi_update(sensor_data);
            currDist += sensor_data -> distance;

            leftCliff = sensor_data->cliffFrontLeftSignal;
            rightCliff = sensor_data->cliffFrontRightSignal;

            //lcd_printf("Distance: %lf", currDist);
            sprintf(output, "%.2f, %d, %d\n", currDist, leftCliff, rightCliff);
            uart_sendStr(output);
        }

        //Move Backwards
        oi_setWheels(-100,-100);
        while (currDist > 0){
            oi_update(sensor_data);
            currDist += sensor_data -> distance;

            leftCliff = sensor_data->cliffFrontLeftSignal;
            rightCliff = sensor_data->cliffFrontRightSignal;

            //lcd_printf("Distance: %lf", currDist);
            sprintf(output, "%.2f, %d, %d\n", currDist, leftCliff, rightCliff);
            uart_sendStr(output);
        }
        oi_setWheels(0,0);
    }
}
