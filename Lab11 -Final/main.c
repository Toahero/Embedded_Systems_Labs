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
#include <open_interface.h>

#include "dataCollection.h"

//1318-09: Right: 284480 Left: 312000
//1318-01: Right: 286720 Left 312640
#define RIGHT_CALIB 286720
#define LEFT_CALIB 312640


#define IR_MINIMUM 300
#define IR_RESCAN_THRESH 100
#define OBJ_THRESH 90

struct cell{
    int mapped;
    int hole;
    int occupied;

    int objects[];

    //float objCoords[][]; ;
};


void sensorTests(void);

void ir_scanRange(int scanVals[], int startDeg, int endDeg);
float pingAt(int angle);

int multiScanIR(int angle, int numScans);

//void oI_test(void);

void moveTest(void);
void getData(void);

int main(void) {

    //sensorTests();
    //oI_test();
    moveTest();

}

void getData(void){
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    ping_init();
    button_init();
    servo_init();

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    collect_cliffSignals(sensor_data, 500);

    oi_free(sensor_data); // do this once at end of main()
}

void moveTest(void){
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    ping_init();
    button_init();
    servo_init();

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    double dist = 10000.0;

    int result;

    result = forward_mm_nav(sensor_data, &dist);

    switch(result){
    case 0:
        lcd_printf("Movement completed.");
        break;

    case 1:
        lcd_printf("Left Sensor Bumped");
        break;

    case 2:
        lcd_printf("Right Sensor Bumped");
        break;

    case 3:
        lcd_printf("Front Left Cliff");
        break;

    case 4:
        lcd_printf("Front Right Cliff");
        break;
    }

    turn_right(sensor_data, 90.0);
    dist = 50.0;
    forward_mm_nav(sensor_data, dist);
    turn_right(sensor_data, 90.0);
    dist = 10000.0;
    forward_mm_nav(sensor_data, dist);

    oi_free(sensor_data); // do this once at end of main()
}

/*void oI_test(void){
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    ping_init();
    button_init();
    servo_init();

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    char output[50];
    command_byte0 = 'q';


    lcd_printf("Press any key to start");
    uart_get();

    int leftCliff;
    int rightCliff;
    while(1){
        oi_update(sensor_data);

        leftCliff = sensor_data->cliffFrontLeftSignal;
        rightCliff = sensor_data->cliffFrontRightSignal;

        sprintf(output, "%d, %d\n", leftCliff, rightCliff);
        uart_sendStr(output);

        lcd_printf(output);
        timer_waitMillis(100);


        if(command_flag0){
            command_flag0 = 0;
            break;
        }
    }

    lcd_printf("Testing concluded");
    oi_free(sensor_data); // do this once at end of main()
}*/

void sensorTests(){
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    ping_init();
    button_init();
    servo_init();

    lcd_printf("init finished");
    servo_setLeft(LEFT_CALIB);
    servo_setRight(RIGHT_CALIB);

    int angle = 0;
    //float dist;

    char outStr[100];
    //dist = pingAt(angle);

    //lcd_printf("Angle: %d\nDistance:%.3f", angle, dist);

    int degVals[200];



    int i;
    int j;
    for(j = 0; j < 3; j++){
        ir_scanRange(degVals, 0, 180);

        for(i = 0; i < 180; i++){
            sprintf(outStr, "%d, %d\n", i, degVals[i]);
            uart_sendStr(outStr);
            //timer_waitMillis(100);
        }
    }

}



void ir_scanRange(int scanVals[], int startDeg, int endDeg){
    int i;
    for(i = startDeg; i < endDeg; i++){
        servo_move(i);
        scanVals[i] = multiScanIR(i, 3);
    }
}

int multiScanIR(int angle, int numScans){

    int scanVal;
    int scanSum = 0.0;
    int i = 0;
    int change = 0;
    int currAvg = 0;

    int failCount = 0;
    int failMax = 5;

    while(i < numScans){

        scanVal = adc_read();

        if(scanVal < IR_MINIMUM){
            scanVal = IR_MINIMUM;
        }

        scanSum += scanVal;
        currAvg = scanSum / (i+1);
        change = abs(currAvg - scanVal);

        if(change > IR_RESCAN_THRESH && failCount < failMax){
            i = 0;
            scanSum = 0;
            failCount++;
        }
        else {
            i++;
        }
    }

    return scanSum /numScans;

}

float pingAt(int angle){
    servo_move(angle);
    return ping_getDistance();
}
