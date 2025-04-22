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

#include <stdlib.h>

//1318: Right: 284480 Left: 312000
#define RIGHT_CALIB 284480
#define LEFT_CALIB 312000

void checkpoint1(void);

void checkpoint2(void);

void calibrationTest(void);

void testCalibrate(void);

int main(void) {

    //testCalibrate();

    checkpoint1();
    //checkpoint2();

}

void testCalibrate(){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    servo_init();
    button_init();

    servo_calibrate();
}



void checkpoint2(void){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    servo_init();
    button_init();

}

void checkpoint1(){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    servo_init();


    servo_setLeft(LEFT_CALIB);
    servo_setRight(RIGHT_CALIB);

    servo_move(90);
    lcd_printf("90 degrees");
    timer_waitMillis(1000);

    servo_move(30);
    lcd_printf("30 degrees");
    timer_waitMillis(1000);

    servo_move(150);
    lcd_printf("150 degrees");
    timer_waitMillis(1000);

    servo_move(90);
    lcd_printf("90 degrees");
    timer_waitMillis(1000);
}
