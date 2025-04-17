/*
 * servo.c
 *
 *  Created on: Apr 15, 2025
 *      Author: jagaul
 */


#include "servo.h"

#define CYCLES_IN_20MS 320000

//Default calibration value in cycles
static unsigned int deg_0;
static unsigned int deg_180;

static void set_angle(unsigned int angle_deg){
    unsigned int matchVal = deg_0 - (angle_deg * (deg_0 - deg_180)) / 180;

    servo_setMatch(matchVal);
}

void servo_setMatch(unsigned int match){
    //Split the values across the match and prescaler registers
    TIMER1_TBMATCHR_R = (match & 0x00FFFF);
    TIMER1_TBPMR_R = (match & 0xFF0000) >> 16;
}

int servo_getMatch(void){
    //Recombine the values using bitwise operations
    return((TIMER1_TBPMR_R & 0xFF) << 16) | (TIMER1_TBMATCHR_R & 0xFFFF);
}

void servo_init(void){
    //For this function, we use Timer 1B (connected to GPIO pin PB5) in PWM mode
    //The timer must be configured as a 24 bit count-down counter, using a prescaler

    //Initialize GPIO pin B3

    deg_0 = CYCLES_IN_20MS - 1000 * 16;
    deg_180 = CYCLES_IN_20MS - 2000 * 16;;

    SYSCTL_RCGCGPIO_R |= 0b00000010; //enable clock to GPIO port B
    while ((SYSCTL_PRGPIO_R & 0b00000010) == 0) {}; //Wait for GPIOB to be ready


    GPIO_PORTB_DEN_R |= 0b00100000; //Enable digital functionality on pin B3

    GPIO_PORTB_AFSEL_R |= 0b00100000; //Enable alternate functionality on pin B3

    GPIO_PORTB_PCTL_R |= 0x700000; //Configure B5 to respond to timer 1


    //Initialize the Timer

    SYSCTL_RCGCTIMER_R |= 0b10; //Initialize the clock for the timer

    TIMER1_CTL_R &= ~0x100; // Disable timer 1B (by clearing pin 8) while configuring

    TIMER1_CFG_R =(0b1111 & 0x4); //Set the last four pins to 0x4 (masking others), to configure the timer in 16 bit mode

    /*Three Changes are made: Set bit 3 TBAMS to 1 (PWM Mode),  Set bit 2 TBCMR to 0 (edge time mode), set bits 1&0 TBMR to 0x2 (Periodic Timer mode)*/
    TIMER1_TBMR_R = 0b00001011;

    TIMER1_CTL_R &= ~0x4000; //Set timer 1B to noninverted

    TIMER1_TBILR_R = (CYCLES_IN_20MS & 0x00FFFF); //Sets the timer start value to 20ms (in cycles)
    TIMER1_TBPR_R = (CYCLES_IN_20MS & 0xFF0000) >> 16; //Sets the prescaler so that the timer functions as a 24 bit counter



    set_angle(90);


    TIMER1_CTL_R |= 0x100; //Re-enable timer 1B by setting pin 8 to 1

    timer_waitMillis(100);
}

void servo_setEdges(unsigned int deg0, unsigned int deg180){
    deg_0 = deg0;
    deg_180 = deg180;
}

void servo_setLeft(unsigned int leftCoord){
    deg_0 = leftCoord;
}

void servo_setRight(unsigned int rightCoord){
    deg_180 = rightCoord;
}

void servo_move(uint16_t angle_deg)
{
    int turnAng;
    if (angle_deg > 180)
    {
        turnAng = 180;
    }
    else{
        turnAng = angle_deg;
    }

    set_angle(turnAng);
    timer_waitMillis(50); // Dumb wait for the servo to move, tune this
}

void servo_calibrate(void){
    int keepGoing = 1;
    int button;
    int prevButton;
    int speed = 20;
    unsigned int minVal;
    unsigned int maxVal;
    unsigned int match = 320000 - 1500 * 16;

    button = button_getButton();
    while (keepGoing){
        prevButton = button;
        button = button_getButton();

        if(button == prevButton){
            switch(button){
            case 1:
                match -= (speed * 16);
                break;

            case 2:
                match += (speed * 16);
                break;

            case 4:
                maxVal = match;
                keepGoing = 0;
                break;

            default:
                break;
            }
        }

        lcd_printf("Match Val: %d\nSet to the rightmost value (180)", match);
        servo_setMatch(match);
        timer_waitMillis(50);
    }
    lcd_printf("Right Value Set");
    timer_waitMillis(1000);

    keepGoing = 1;
    while (keepGoing){
            prevButton = button;
            button = button_getButton();

            if(button == prevButton){
                switch(button){
                case 1:
                    match -= (speed * 16);
                    break;

                case 2:
                    match += (speed * 16);
                    break;

                case 4:
                    minVal = match;
                    keepGoing = 0;
                    break;

                default:
                    break;
                }
            }

            lcd_printf("Match Val: %d\nSet to the leftmost value (0)", match);
            servo_setMatch(match);
            timer_waitMillis(50);
        }

    lcd_printf("Left(0):%d\nRight(180): %d", minVal, maxVal);
}
