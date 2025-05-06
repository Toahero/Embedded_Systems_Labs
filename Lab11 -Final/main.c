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

#include "movementCalib.h"
//#include "movement.h"

#include "line_following.h"

#include "scanFunctions.h"

//1318-02: Right: 312320 Left: 286400
//1318-09: Right: 284480 Left: 312000
//1318-10: Right: 311040 Left: 283200
//1318-01: Right: 286720 Left 312640
//2041-12: Right: 312320 Left: 275760
#define RIGHT_CALIB 311040
#define LEFT_CALIB 283200


void scanTests(void);

void generateMap(void);

void sensorTests(void);



oi_t* initSensors(void);

void moveTest(void);
void getData(void);

void calibrateTest();

void lineFollowTest();

int main(void) {

    //sensorTests();
    //moveTest();
    //calibrateTest();

    //generateMap();
    scanTests();
    //lineFollowTest();

    //getData();
}

void moveTest(void){
    oi_t *sensor_data = initSensors();

    double moveDist = 2000.0;

    lcd_printf("Move forward %.2f", moveDist);
    move_forward(sensor_data, moveDist);

    oi_free(sensor_data);
}

void calibrateTest(){
    oi_t *sensor_data = initSensors();

    double calDistance = getBackwardAdjust(sensor_data);

    timer_waitMillis(1000);
    lcd_printf("Modifier:\n%.3f", calDistance);
    oi_free(sensor_data); // do this once at end of main()
}

void scanTests(){
    oi_t *sensor_data = initSensors();

    //servo_calibrate();
    //testSweep();
    //lineScanTest(sensor_data);
    edgeScanTest(sensor_data);
    oi_free(sensor_data);
}


void lineFollowTest(){
    oi_t *sensor_data = initSensors();


    calibrate_CliffValue(sensor_data);
    displayCliffVals();
    timer_waitMillis(1000);
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
    int keepGoing = 1;

    while (keepGoing){
        oi_setWheels(0, 0);
        lcd_printf("Press 1 to scan map\nPress 2 to calibrate sensors\nPress 3 to quit");

        button = 0;
        while(button == 0){
            button = button_getButton();
        }

        switch(button){
        case 1:
            scanPerimeter(sensor_data);
            break;

        case 2:
            calibrate_CliffValue(sensor_data);
            displayCliffVals();
            timer_waitMillis(1000);
            break;

        case 3:
            keepGoing = 0;
            lcd_printf("Exiting program");
            break;

        default:
            break;
        }


    }

    oi_free(sensor_data); // do this once at end of main()
}


void getData(void){
    oi_t *sensor_data = initSensors();

    //collect_lineEdge(sensor_data);

    collectIR();
    oi_free(sensor_data); // do this once at end of main()
}

void sensorTests(){
    oi_t *sensor_data = initSensors();

    int scanArray[180];
    int arraySize;
    int numObjects;

    //servo_calibrate();


    arraySize = ir_scanRange(scanArray, 45, 135, 2);
    numObjects = scan_containsObject(scanArray, arraySize, 90);
    lcd_printf("%d objects found", numObjects);

    oi_free(sensor_data);
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

    servo_setLeft(LEFT_CALIB);
    servo_setRight(RIGHT_CALIB);

    return sensor_data;
}
