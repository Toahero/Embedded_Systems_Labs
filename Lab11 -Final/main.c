/*
 * main10.c
 *
 *  Created on: Apr 15, 2025
 *      Author: jagaul
 */


#include "Timer.h"
#include "lcd.h"
#include "servo.h"
#include "button.h"
#include "ping.h"
#include "adc.h"
#include "uart-interrupt.h"
#include <stdlib.h>

//1318-09: Right: 284480 Left: 312000
#define RIGHT_CALIB 284480
#define LEFT_CALIB 312000

void sensorTests(void);

void ir_scanRange(int scanVals[], int startDeg, int endDeg);
float pingAt(int angle);

int main(void) {

    timer_init();
    lcd_init();
    uart_interrupt_init();
    adc_init();
    ping_init();
    button_init();
    servo_init();

    lcd_printf("init finished");
    servo_setLeft(LEFT_CALIB);
    servo_setRight(RIGHT_CALIB);

    sensorTests();
}

void sensorTests(){
    int angle = 0;
    float dist;

    dist = pingAt(angle);

    lcd_printf("Angle: %d\nDistance:%.3f", angle, dist);

    int degVals[180];

    ir_scanRange(degVals, 0, 180);
    int i;
    for(i = 0; i < 180; i++){
        lcd_printf("%d", degVals[i]);
        timer_waitMillis(100);
    }
}

void ir_scanRange(int scanVals[], int startDeg, int endDeg){
    int i;
    for(i = startDeg; i < endDeg; i++){
        servo_move(i);
        scanVals[i] = adc_read();
    }
}

float pingAt(int angle){
    servo_move(angle);
    return ping_getDistance();
}
