/**
 * @file lab9_template.c
 * @author
 * Template file for CprE 288 Lab 9
 */

#include "Timer.h"
#include "lcd.h"
#include "ping.h"
#include "uart-interrupt.h"
#include "adc.h"

#include<stdlib.h>
// Uncomment or add any include directives that are needed

//#warning "Possible unimplemented functions"
//#define REPLACEME 0

void checkpoint2(void);
void checkpoint3(void);

void calibratePing();

void dualScan();

int main(void) {
	//checkpoint2();
    checkpoint3();

    //calibratePing();

    //dualScan();
}

void checkpoint3(void){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    ping_init();
    uart_interrupt_init();

    command_byte0 = 'q';
    //uart_receive();

    char output[100];

    int delay;
    double delayMS;
    float dist;
    int numOverflows = 0;

    //Based on the given processor speed of 80 MHz
    int commands_per_ms = 80000;

    while(1){
        if(command_flag0){
        command_flag0 = 0;
        break;
        }

        delay = ping_getDelay();

        if(delay < 1){
            numOverflows++;
        }
        else{
            delayMS = (double) delay / commands_per_ms;
            dist = 0.001071875 * delay;
            //dist = 0.001 * delay + 0.4275;
            sprintf(output, "Step Delay: %d\nTime Delay: %.2f ms\nDistance: %.2f cm \nTotal Overflows: %d\n", delay, delayMS, dist, numOverflows);
            lcd_printf("%s", output);
            uart_sendStr(output);
        }



    }
}

void checkpoint2(void){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    //uart_interrupt_init();
    ping_init();


    // YOUR CODE HERE
    lcd_printf("Init completed");
    timer_waitMillis(500);


    int delay;
    int numOverflows = 0;

    while(1)
    {
        //uart_sendStr("Test");

        ping_trigger();
        timer_waitMillis(100);
        delay = ping_getDelay();

        if(delay < 1){
            numOverflows++;
        }
        else{
            lcd_printf("Delay: %d, Total Overflows: %d", delay, numOverflows);
        }



    timer_waitMillis(50);

    }
}

void calibratePing(void){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    uart_interrupt_init();
    ping_init();

    int i;
    int distance = 5;
    int numValues = 10;
    int interval = 10;
    int delay;

    char output[100];

    int distArray[numValues];
    int intervalArray[numValues];

    for(i = 0; i < numValues; i++){
        sprintf(output, "move the cybot %d cm away: ", distance);
        lcd_printf("%s", output);
        uart_sendStr(output);

        distArray[i] = distance;
        distance += interval;

        //uart_receive();
        //ping_trigger();
        //timer_waitMillis(500);
        delay = ping_getDelay();
        sprintf(output, "%d \n", delay);
        uart_sendStr(output);

        intervalArray[i] = delay;
    }

    for(i = 0; i < numValues; i++){
        sprintf(output, "%d, %d\n", distArray[i], intervalArray[i]);
        uart_sendStr(output);
    }
}

void dualScan(){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    uart_interrupt_init();
    ping_init();
    adc_init();


    char output[100];

    double distance;
    int interval;

    command_byte0 = 'q';
    uart_receive();
    while(1){
        if(command_flag0){
        command_flag0 = 0;
        break;
        }

        distance = adc_dist();
        interval = ping_getDelay();

        sprintf(output, "%.2f, %d\n", distance, interval);
        uart_sendStr(output);
    }
}

