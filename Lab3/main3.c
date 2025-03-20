

/**
 * main.c
 */
#include <stdlib.h>
#include <open_interface.h>
#include <movement.h>
#include <cyBot_uart.h>
#include <lcd.h>
#include<Timer.h>
#include <string.h>
#include <cyBot_Scan.h>
#include <math.h>


#define DETECT_SCAN_COUNT 3
#define DIST_SCAN_COUNT 5
#define OBJ_THRESH 10.0
#define RESCAN_THRESH 0.40
#define MAX_SCAN_DIST 150

/*Calibration Scan for CyBOT 2041-06
 * right(0) is given as 290500
 * left(180) is given as 1209250
 */

/*Calibration Scan for CyBOT 2041-14
 * right(0) is given as 306250
 * left(180) is given as 1293250
 */

/*Calibration Scan for CyBOT 2041-2:
right(0) is given as 238000
left(180) is given as 1214500
*/

/*Calibration Scan for CyBOT 1318-07
 * right(0) is given as 232750
 * left(180) is given as 1251250
 */

/*Calibration Scan for CyBOT 1318-10
 * right(0) is given as 316750
 * left(180) is given as 1303750
 */

struct obstacle{
    int firstDeg;
    int lastDeg;
    float midDist;
    float linWidth;
};

void stringToPutty(char inputString[]);

void checkpoint1(void);
void checkpoint2(void);
void checkpoint3(void);
void bonusCredit(void);

void calibrateServos(void);
void scanTest(void);
int nextObject(int* angle, int interval, struct obstacle* nextObs);
float multiScan(int angle, int numScans);

int getMidDeg(struct obstacle* obs);
int getAngWidth(struct obstacle* obs);

float getLinWidth(float dist, int degWidth);

int main(void) {


    //calibrateServos();

    //bonusCredit();
    //checkpoint2();

    checkpoint3();

    return 0;
}

void bonusCredit(void){
    timer_init();
    lcd_init();
    cyBot_uart_init();
    cyBOT_init_Scan(0b0111);
    right_calibration_value = 290500;
    left_calibration_value = 1209250;

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    //struct obstacle nextObstacle;
    struct obstacle obstacleList[10];

    int currAngle = 0;
    int numObjects = 0;

    //char outputString[50];

    while(currAngle < 180){
        numObjects += nextObject(&currAngle, 2, &obstacleList[numObjects]);
        currAngle += 2;
    }

    lcd_printf("%d objects found.", numObjects);
    //timer_waitMillis(2000);

    int i;
    int angWidth;
    int midAng;
    float minWidth = 0.0;
    int thinnestObj = 0;

    obstacleList[0].linWidth = getLinWidth(obstacleList[0].midDist, getAngWidth(&obstacleList[0]));
    minWidth = obstacleList[0].linWidth;

    for(i = 1; i < numObjects; i++){
        angWidth = getAngWidth(&obstacleList[i]);
        midAng = getMidDeg(&obstacleList[i]);
        obstacleList[i].linWidth = getLinWidth(obstacleList[i].midDist, midAng);

        if(obstacleList[i].linWidth < minWidth){
            minWidth = obstacleList[i].linWidth;
            thinnestObj = i;
        }

    }

    lcd_printf("Thinnest Object %d:\n Angle Width: %d\nLin Width: %.5f", i, getAngWidth(&obstacleList[thinnestObj]), obstacleList[thinnestObj].linWidth);

    midAng = getMidDeg(&obstacleList[thinnestObj]);

    if(midAng < 90){
        turn_right(sensor_data, 90-midAng);
    }
    else{
        turn_left(sensor_data, midAng - 90);
    }

    forward_mm_detours(sensor_data, obstacleList[thinnestObj].midDist);

    oi_free(sensor_data); // do this once at end of main()
}

void checkpoint3(void){
    timer_init();
    lcd_init();
    cyBot_uart_init();
    cyBOT_init_Scan(0b0111);
    right_calibration_value = 290500;
    left_calibration_value = 1209250;

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    //struct obstacle nextObstacle;
    struct obstacle obstacleList[10];

    int currAngle = 0;
    int numObjects = 0;


    int scanInt = 2;

    char outputString[100];

    while(currAngle < 180){
        numObjects += nextObject(&currAngle, scanInt, &obstacleList[numObjects]);
        currAngle += scanInt;
    }

    sprintf(outputString, "%d objects found.\n\r", numObjects);
    stringToPutty(outputString);
    //timer_waitMillis(2000);

    int i;
    int angWidth;
    int midAng;
    int minWidth;
    int thinnestObj = 0;


    minWidth = getAngWidth(&obstacleList[0]);


    for(i = 0; i < numObjects; i++){
        angWidth = getAngWidth(&obstacleList[i]);
        midAng = getMidDeg(&obstacleList[i]);
        obstacleList[i].linWidth = getLinWidth(obstacleList[i].midDist, midAng);

        if(angWidth < minWidth){
            minWidth = angWidth;
            thinnestObj = i;
        }
    }

    midAng = getMidDeg(&obstacleList[thinnestObj]);
    multiScan(midAng, 1);

    //Display Results:
    for(i = 0; i < numObjects; i++){
        //lcd_printf("Object %d:\nS:%d F:%d M:%d\nAngle Width: %d\nMiddle Dist: %5f", i, obstacleList[i].firstDeg, obstacleList[i].lastDeg, midAng, angWidth, obstacleList[i].midDist);
        sprintf(outputString, "Object %d: Angle Width: %d Middle Dist: %.5f Lin Width: %.5f\n\r", i, getAngWidth(&obstacleList[i]), obstacleList[i].midDist, obstacleList[i].linWidth);
        stringToPutty(outputString);
        //timer_waitMillis(5000);
    }

    oi_free(sensor_data); // do this once at end of main()
}

void stringToPutty(char inputString[]){
    int i;
    for(i = 0; i < strlen(inputString); i++){
        char nextByte = inputString[i];
        cyBot_sendByte(nextByte);
    }
}
void checkpoint1(void){
           lcd_init();
           cyBot_uart_init();

           char charRecieved = 'e';
           while (charRecieved != 'z'){
               charRecieved = cyBot_getByte();
               lcd_printf("Got an %c", charRecieved);

               stringToPutty("Message Recieved\n\r");
           }


}

void checkpoint2(void){
    timer_init();
    lcd_init();
    cyBot_uart_init();

    cyBOT_init_Scan(0b0111);
    right_calibration_value = 232750;
    left_calibration_value = 1251250;

    int i;
    cyBOT_Scan_t scan;


    //Wait for Putty to send an 'm' character
    char inputChar = 'e';
    while (inputChar != 'm'){
        inputChar = cyBot_getByte();
        lcd_printf("Got an %c", inputChar);
    }

    //clear

    stringToPutty("\rDegrees     PING Distance (cm)\n\r");

    int scanAngle;
    float scanDist;
    char outputString[50];

    for(i=45; i<= 135; i+=2){
        cyBOT_Scan(i, &scan);
        scanAngle = i;
        scanDist = scan.sound_dist;
        sprintf(outputString, "%d         %f\n\r", scanAngle, scanDist);
        stringToPutty(outputString);
    }

}

void calibrateServos(void){

    timer_init();
    lcd_init();

    cyBOT_init_Scan(0b0111);
    cyBOT_SERVO_cal();

}

void scanTest(void){
    timer_init();
    lcd_init();
    cyBOT_init_Scan(0b0111);
    right_calibration_value = 238000;
    left_calibration_value = 1214500;

    int i;
    cyBOT_Scan_t scan;

    for(i=0; i<= 180; i+=2){
        cyBOT_Scan(i, &scan);
    }
}

int nextObject(int* angle, int interval, struct obstacle* nextObs){
    //double objThreshold = 100.0;

    char outputString[50];

    int currObj = 0;
    int j;
    cyBOT_Scan_t scan;
    double scanSum = 0.0;
    double avgDist;
    double prevDist;
    float objDist;
    int midDeg;
    int angleWidth;

    avgDist = multiScan(*angle, 3);

    while(*angle <= 180){

        prevDist = avgDist;
        scanSum = 0.0;


        avgDist = multiScan(*angle, 3);

        sprintf(outputString, "Dist: %.4f Dist Change: %.4f\n\r", avgDist, avgDist - prevDist);
        stringToPutty(outputString);


        if(((avgDist - prevDist) < (OBJ_THRESH * -1)) && currObj == 0){
            currObj = 1;
             nextObs->firstDeg = *angle;
        }

        if(((avgDist - prevDist) > OBJ_THRESH) && currObj == 1){
            nextObs->lastDeg = *angle;
            angleWidth = ((nextObs->lastDeg) - (nextObs -> firstDeg));
            if(angleWidth < (2 * interval) + 1){
                currObj = 0;
            }
            else{
                midDeg =  ((angleWidth/2)  + (nextObs -> firstDeg));
                objDist = multiScan(midDeg, 5);
                nextObs-> midDist = objDist;
                return 1;
            }

        }
        *angle += interval;
    }

    if(currObj == 1){
        nextObs-> lastDeg = 180;
        angleWidth = ((nextObs->lastDeg) - (nextObs -> firstDeg));
        midDeg =  ((angleWidth/2)  + (nextObs -> firstDeg));
        objDist = multiScan(midDeg, 5);
        nextObs-> midDist = multiScan(nextObs -> lastDeg - nextObs -> firstDeg, 5);
        return 1;
    }
    return 0;
}

float multiScan(int angle, int numScans){
    float scanSum = 0.0;
    cyBOT_Scan_t scan;


    int i;
    for(i = 1; i <= numScans; i++){

        cyBOT_Scan(angle, &scan);

        //If the distance is beyond the max, assign it the limit value
        if(scan.sound_dist > MAX_SCAN_DIST){
            scanSum += MAX_SCAN_DIST;
        }
        else{
            scanSum += scan.sound_dist;
        }
        timer_waitMillis(10);


        //Check if the new scan is too different
        if(fabs(1- (scan.sound_dist / (scanSum/i))) > RESCAN_THRESH){
            return multiScan(angle, numScans);
        }
    }
    //sprintf(outputString, "%.4f\n\r", scanSum/numScans);
    //stringToPutty(outputString);
    return scanSum / numScans;
}

int getMidDeg(struct obstacle* obs){
    int angWidth = getAngWidth(obs);
    int midDeg =  ((angWidth/2)  + (obs -> firstDeg));
    return midDeg;
}

int getAngWidth(struct obstacle* obs){
    int angWidth = ((obs->lastDeg) - (obs -> firstDeg));
    return angWidth;
}

float getLinWidth(float dist, int degWidth){

    float radAng = (180 / 3.14);
    float linWidth = 2 * dist * tan(radAng / 2);
    return linWidth;
}
