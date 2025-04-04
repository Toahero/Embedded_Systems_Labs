/*
 * main8.c
 *
 *  Created on: Apr 2, 2025
 *      Author: jagaul
 */


#include "Timer.h"
#include "lcd.h"
#include "adc.h"

#include<stdlib.h>
#include "cyBot_Scan.h"
#include "uart-interrupt.h"
#include <math.h>

//#include "cyBot_Scan.h"  // For scan sensors
//#include "uart-interrupt.h"
//#include "movement.h"

// Uncomment or add any include directives that are needed
//#include "open_interface.h"

//Calibration Values:
//1318-3: Right: 243250   Left: 1177750
//1318-6  Right: 348250   Left: 1356250
//1318-5: Right: 332500   Left: 1309000
//2041-07 Right: 295750   Left: 1230250
//2041-12 Right: 243250   Left: 1188250
//2041-15 Right: 327250   Left: 1303750

#define RIGHT_CALIB 332500
#define LEFT_CALIB 1309000

void calibrateServos(void);

void checkpoint1(void);
void checkpoint2(void);

double ir_to_dist(int ir);

int main(void) {
    //calibrateServos();
    //checkpoint2();
    IR_conversion();
}

void IR_conversion(void){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    adc_init();
    uart_interrupt_init();

    cyBOT_init_Scan(0b0111);
    cyBOT_Scan_t scan;

    right_calibration_value = RIGHT_CALIB;
    left_calibration_value = LEFT_CALIB;

    timer_waitMillis(250);
    cyBOT_Scan(90, &scan);

    command_byte0 = 'q';

    uart_sendStr("starting\n");

    int irVal;
    float pingDist;
    double irDist;
    char output[100];
    uart_receive();

    while(1){
        cyBOT_Scan(90, &scan);

        pingDist = scan.sound_dist;
        irVal = adc_read();
        irDist = ir_to_dist(irVal);
        sprintf(output, "%.2f, %.2f\n", pingDist, irDist);
        uart_sendStr(output);
        lcd_printf("%s", output);
        timer_waitMillis(50);

        if(command_flag0){
            command_flag0 = 0;
            break;
        }
    }
}

void checkpoint2(void){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    adc_init();
    uart_interrupt_init();

    cyBOT_init_Scan(0b0111);
    cyBOT_Scan_t scan;

    right_calibration_value = RIGHT_CALIB;
    left_calibration_value = LEFT_CALIB;

    timer_waitMillis(250);
    cyBOT_Scan(90, &scan);

    command_byte0 = 'q';

    uart_sendStr("starting\n");

    int irVal;
    float pingDist;
    char output[100];
    uart_receive();

    while(1){
        cyBOT_Scan(90, &scan);

        pingDist = scan.sound_dist;
        irVal = adc_read();

        sprintf(output, "%.2f, %d\n", pingDist, irVal);
        uart_sendStr(output);
        lcd_printf("%s", output);
        timer_waitMillis(50);

        if(command_flag0){
            command_flag0 = 0;
            break;
        }
    }
}

void checkpoint1(void){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    adc_init();


    lcd_printf("Init finished");
    timer_waitMillis(250);
    int reading = 0;

    while(1){
        reading = adc_read();
        lcd_printf("%d", reading);
        timer_waitMillis(100);
        }
}

double ir_to_dist(int ir){
    double result = 124725 * pow(ir, -1.182);
    return result;
}

void calibrateServos(void){

    timer_init();
    lcd_init();

    cyBOT_init_Scan(0b0111);
    cyBOT_SERVO_cal();

}
