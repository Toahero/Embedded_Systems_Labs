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


void checkpoint1(void);

void checkpoint2(void);

int main(void) {

    //test();

    checkpoint1();
    //checkpoint2();


}


void checkpoint2(void){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    servo_init();
    button_init();

    int currAng = 90;
    int currMatch;
    int rightButton;
    int currDir = 1;



    servo_move(90);

    while(1){
        rightButton = button_getButton();
        switch(rightButton){
        case 1:
            currAng += 1 * currDir;
            break;

        case 2:
            currAng += 5* currDir;
            break;

        case 3:
            currDir *= -1;
            break;

        case 4:
            if(currDir == 1){
                currAng = 5;
            }
            else{
                currAng = 175;
            }
            break;

        default:
            break;
        }

        if(currAng > 180){
            currAng = 180;
        }
        if(currAng < 0){
            currAng = 0;
        }


        servo_move(currAng);
        currMatch = get_match();
        lcd_printf("Angle: %d\nMatch: %d\n Direction: %d", currAng, currMatch, currDir);
    }
}

void checkpoint1(){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    servo_init();


    servo_calibrate();

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
