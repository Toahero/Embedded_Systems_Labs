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
#include "moveAndScan.h"
#include "movementCalib.h"
//#include "movement.h"

#include "line_following.h"

#include "scanFunctions.h"

//1318-09: Right: 284480 Left: 312000
//1318-01: Right: 286720 Left 312640
#define RIGHT_CALIB 286720
#define LEFT_CALIB 312640




struct Cell{
    int mapped;
    int hole;
    int occupied;

    int objects[];

    //float objCoords[][]; ;
};

void generateMap(void);

void sensorTests(void);

void moveAndScanTests(void);

void ir_scanRange(int scanVals[], int startDeg, int endDeg);
float pingAt(int angle);

int multiScanIR(int angle, int numScans);

oi_t* initSensors(void);

void moveTest(void);
void getData(void);

void calibrateTest();

void lineFollowTest();

int main(void) {

    //sensorTests();;
    //moveTest();
    //moveAndScanTests();
    generateMap();
    //calibrateTest();


    //lineFollowTest();
}

void calibrateTest(){
    oi_t *sensor_data = initSensors();

    double calDistance = getBackwardAdjust(sensor_data);

    timer_waitMillis(1000);
    lcd_printf("Modifier:\n%.3f", calDistance);
    oi_free(sensor_data); // do this once at end of main()
}

void moveAndScanTests(void){
    oi_t *sensor_data = initSensors();



    int distance = 1000;
    int result;
    result = move_forward_detect(sensor_data, &distance);

    lcd_printf("distance left: %d\nMovement stop value: %d", distance, result);

    oi_free(sensor_data); // do this once at end of main()
}

void lineFollowTest(){
    oi_t *sensor_data = initSensors();


    //calibrate_CliffValue(sensor_data);
    //displayCliffVals();
    //timer_waitMillis(1000);
    int button = 0;
    lcd_printf("Press any button to start");
    while(button == 0){
        button = button_getButton();
    }

    followPerimeter(sensor_data);
    //followLine(sensor_data);
    //collect_lineEdge(sensor_data);
    oi_free(sensor_data); // do this once at end of main()
}

void generateMap(void){

    oi_t *sensor_data = initSensors();

    int button = 0;
    lcd_printf("Press any button to start");
    while(button == 0){
        button = button_getButton();
    }

    scanPerimeter(sensor_data);


    oi_free(sensor_data); // do this once at end of main()
}


void getData(void){
    oi_t *sensor_data = initSensors();

    collect_cliffSignals(sensor_data, 500);

    oi_free(sensor_data); // do this once at end of main()
}

/*void moveTest(void){
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


oi_t* initSensors(void){
    timer_init();
    lcd_init();
    uart_init();
    adc_init();
    ping_init();
    button_init();
    servo_init();

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    return sensor_data;
}
