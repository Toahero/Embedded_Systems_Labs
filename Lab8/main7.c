/**
 * lab6_template.c
 *
 * Template file for CprE 288 Lab 6
 *
 * @author Diane Rover, 2/15/2020
 *
 */

#include "Timer.h"
#include "lcd.h"
#include "cyBot_Scan.h"  // For scan sensors
#include "uart-interrupt.h"
#include "movement.h"
#include "adc.h"

// Uncomment or add any include directives that are needed
#include "open_interface.h"
// #include "movement.h"
// #include "button.h"
#include<stdlib.h>
#include <math.h>
//Calibration Values:
//1318-2: Right: 269500   Left: 1177750
//1318-3: Right: 243250   Left: 1177750
//1318-4: Right: 301000   Left: 1246000
//1318-6  Right: 348250   Left: 1356250
//1318-5: Right: 316750   Left: 1272250
//2041-07 Right: 295750   Left: 1230250
//2041-12 Right: 243250   Left: 1188250
//2041-15 Right: 327250   Left: 1303750

#define RIGHT_CALIB 316750
#define LEFT_CALIB 1272250

#define IR_MINIMUM 400
#define IR_RESCAN_THRESH 100
#define OBJ_THRESH 90

#define MAX_OBSTACLES 15

#define PING_RESCAN_THRESH 0.40

struct obstacle{
    int firstDeg;
    int lastDeg;
    int midDeg;

    float midDist;
    float linWidth;
};

void manAndAuto(void);
void moveToSmallest(void);

void nextObjTest(void);
void IRmultiScanTest(void);
void PingMultiScanTest(void);
void scanTest(void);
void checkpoint1(void);
void sensorTest(void);

void displayObstacles(int numObjects, struct obstacle obsList[]);
int getObstacles(int minAngle, int maxAngle, int interval, struct obstacle obsList[]);
int nextObjEdges(int* angle, int endAng, int interval, int numScans, struct obstacle* nextObs);

void pingObstacle(struct obstacle* currObs, int numPings);

int multiScanIR(int angle, int numScans);

float multiScanPing(int angle, int numScans);

float getLinWidth(int degWidth, float dist);


void calibrateServos(void);
void calibrateMovement(void);

int bufferAvg(int buffer[], int bufferSize);

int main(void) {
    //calibrateServos();
    //scanTest();
    //PingMultiScanTest();
    //nextObjTest();
    moveToSmallest();
    //manAndAuto();
    //calibrateMovement();

}



void moveToSmallest(){
   timer_init();
   lcd_init();
   uart_interrupt_init();
   cyBOT_init_Scan(0b0111);
   right_calibration_value = RIGHT_CALIB;
   left_calibration_value = LEFT_CALIB;
   adc_init();

   oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
   oi_init(sensor_data); // do this only once at start of main()




   int maxObs = 10;

   struct obstacle obstacleList[maxObs];
   int angle = 5;
   int obsCount = 0;
   int nextObs;
   int smallestDistMM = 1000;
   char outputLine[100];



   uart_sendStr("\n\rStarting\n\r");

   int i;
  int minObstacle = 0;
  float minWidth;
  int moveDist;
  double turnAng;

   while(smallestDistMM > 100){

       //Get obstacles and display them to putty
       obsCount = getObstacles(0, 180, 1, obstacleList);

       sprintf(outputLine, "\n\r%d obstacles found.\n\r", obsCount);
       uart_sendStr(outputLine);

       displayObstacles(obsCount, obstacleList);


       minObstacle = 0;
       minWidth = obstacleList[0].linWidth;
       for(i = 1; i < obsCount; i++){

           if(obstacleList[i].linWidth < minWidth){
               minWidth = obstacleList[i].linWidth;
               minObstacle = i;
           }
       }

       sprintf(outputLine, "\n\rObstacle %d is the thinnest\n\r", minObstacle);
       uart_sendStr(outputLine);

       turnAng = obstacleList[minObstacle].midDeg - 90.0;
       sprintf(outputLine, "Turning %.1f degrees\n\r", turnAng);
       uart_sendStr(outputLine);


       if(obstacleList[minObstacle].midDist > 50){
           smallestDistMM = obstacleList[minObstacle].midDist * 9;
       }
       else{
           smallestDistMM = obstacleList[minObstacle].midDist * 5;
       }

       sprintf(outputLine, "Moving forward %d mm", smallestDistMM);
       uart_sendStr(outputLine);

       lcd_printf("Turning %.1f degrees\nAdvancing %d mm", turnAng, smallestDistMM);

       if(turnAng > 0){
           turn_left(sensor_data, turnAng);
       }
       else{

           turn_right(sensor_data, -turnAng);
       }
       if(smallestDistMM > 100){
           forward_mm_detours(sensor_data, smallestDistMM);
       }

   }


   lcd_printf("Complete");
   oi_free(sensor_data);
}



int getObstacles(int minAngle, int maxAngle, int interval, struct obstacle obsList[]){


    int angle = minAngle + interval;
    int max = maxAngle - interval;
    int obsCount = 0;
    int nextObs;
    int numScans = 1;




    while(obsCount < MAX_OBSTACLES && angle < maxAngle - interval){

        //lcd_printf("Scanning %d", angle);
        nextObs = nextObjEdges(&angle, max, 1, numScans, &obsList[obsCount]);

        if(nextObs > 0){
            pingObstacle(&obsList[obsCount], 1);
            obsCount++;
        }
    }

    return obsCount;
}

void displayObstacles(int numObjects, struct obstacle obsList[]){
    int i;
    int firstAng, lastAng, midAng;
    float dist, width;
    char outputLine[100];

    for(i = 0; i < numObjects; i++){
        firstAng = obsList[i].firstDeg;
        lastAng = obsList[i].lastDeg;
        midAng = obsList[i].midDeg;
        dist = obsList[i].midDist;
        width = obsList[i].linWidth;

        lcd_printf("start: %d\nend: %d\nmid: %d", firstAng, lastAng, midAng);

        sprintf(outputLine, "\nObstacle %d\n\rStart: %d, End: %d, Mid: %d\n\r", i, firstAng, lastAng, midAng);
        uart_sendStr(outputLine);

        sprintf(outputLine, "Distance: %.5f  Linear Width: %.5f\n\r", dist, width);
        uart_sendStr(outputLine);
    }
}


int nextObjEdges(int* angle, int endAng, int interval, int numScans, struct obstacle* nextObs){
    int avgDist;
    int buffAvg;
    int currObj = 0;
    int distChange;

    int bufferSize = 5;

    int buffer[bufferSize];
    int i;

    //TEMP: Line for putty output
    //char outputLine[50];

    *angle = *angle - (bufferSize * interval);
    for(i = 0; i < bufferSize; i++){
        buffer[i] = multiScanIR(*angle, numScans);
        *angle += interval;
    }
    i = 0;
    avgDist = multiScanIR(*angle, numScans);

    while(*angle < endAng){


        *angle += interval;

        avgDist = multiScanIR(*angle, numScans);

        buffer[i % bufferSize] = avgDist;
        buffAvg = bufferAvg(buffer, bufferSize);
        i++;

        distChange = avgDist - buffAvg;
        lcd_printf("%d: objPres: %d\n Change: %d", *angle, currObj, distChange);

        //If value increases substantially and no object currently found, record the first degree and object present.
        if((distChange > (OBJ_THRESH)) && currObj == 0){
            currObj = 1;
            nextObs->firstDeg = *angle;
        }

        //If the value increases substantially and an object is currently found, mark the end of the current object and prepare to scan a new one.
        /*if(distChange > OBJ_THRESH){
            nextObs->lastDeg = *angle;
            return 1;
        }*/


        //TEMP: Send the Data to Putty
        //sprintf(outputLine, "%d, %d, %d, %d\n\r", *angle, currObj, avgDist, buffAvg);
        //uart_sendStr(outputLine);

        //If the value decreases substantially and an object is currently found, mark the end point;
        if((distChange < OBJ_THRESH * -1) && currObj == 1){
            nextObs->lastDeg = *angle;
            int angleWidth = ((nextObs->lastDeg) - (nextObs -> firstDeg));

            //If the object is smaller than two interval movements, assume it was a false reading
            if(angleWidth < (3 * interval) + 1){
                currObj = 0;
                //*angle = startVal;
            }
            //Otherwise, return that it's a true reading.
            else{
                int midDegree =  ((angleWidth/2)  + (nextObs -> firstDeg));
                nextObs -> midDeg = midDegree;
                return 1;
            }

        }


    }

    if(currObj == 1){
        nextObs->lastDeg = *angle;
        int angleWidth = ((nextObs->lastDeg) - (nextObs -> firstDeg));
        nextObs -> midDeg = ((angleWidth/2)  + (nextObs -> firstDeg));
    }

    return currObj;
}


void pingObstacle(struct obstacle* currObs, int numPings){
    int midPoint = currObs->midDeg;
    int degWidth = (currObs->lastDeg - currObs->midDeg) * 2;

    float dist = multiScanPing(midPoint, numPings);
    currObs -> midDist = dist;
    currObs -> linWidth = getLinWidth(degWidth, dist);

}

int multiScanIR(int angle, int numScans){

    int scanVal;
    int scanSum = 0.0;
    int i = 0;
    int change = 0;
    int currAvg = 0;

    int failCount = 0;
    int failMax = 5;

    cyBOT_Scan_t scan;
    while(i < numScans){

        cyBOT_Scan(angle, &scan);
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

float multiScanPing(int angle, int numScans){

    float scanSum = 0.0;
    int i = 0;
    float change = 0.0;
    float currAvg = 0;

    int failCount = 0;
    int failMax = 5;

    cyBOT_Scan_t scan;

    while(i < numScans){
        cyBOT_Scan(angle, &scan);
        timer_waitMillis(10);
        scanSum += scan.sound_dist;


        currAvg = scanSum / (i+1);
        change = fabs(currAvg - scan.sound_dist);

        if(change > PING_RESCAN_THRESH && failCount < failMax){
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

int bufferAvg(int buffer[], int bufferSize){
    int buffSum = 0;
    int i;
    for(i = 0; i < bufferSize; i++){
        buffSum += buffer[i];
    }
    return buffSum / bufferSize;
}

float getLinWidth(int degWidth, float dist){

    float radAng = degWidth * (3.14 / 180);
    float linWidth = 2* dist * tan(radAng / 2);
    return linWidth;
}


void calibrateServos(void){

    timer_init();
    lcd_init();

    cyBOT_init_Scan(0b0111);
    cyBOT_SERVO_cal();

}


//Finished Test Functions
void nextObjTest(void){
    timer_init();
    lcd_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);
    right_calibration_value = RIGHT_CALIB;
    left_calibration_value = LEFT_CALIB;

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    char outputLine[100];


    int maxObs = 10;

    struct obstacle obstacleList[maxObs];
    int angle = 5;
    int obsCount = 0;
    int nextObs;

    uart_sendStr("\n\rStarting\n\r");


    while(obsCount < maxObs && angle < 180){

        //lcd_printf("Scanning %d", angle);
        nextObs = nextObjEdges(&angle, 180, 1, 2, &obstacleList[obsCount]);

        if(nextObs > 0){

            pingObstacle(&obstacleList[obsCount], 1);


            lcd_printf("start: %d\nend: %d\nmid: %d", obstacleList[obsCount].firstDeg, obstacleList[obsCount].lastDeg, obstacleList[obsCount].midDeg);
            timer_waitMillis(250);

            sprintf(outputLine, "Obstacle %d\n\rStart: %d, End: %d, Mid: %d\n\r", obsCount, obstacleList[obsCount].firstDeg, obstacleList[obsCount].lastDeg, obstacleList[obsCount].midDeg);
            uart_sendStr(outputLine);

            sprintf(outputLine, "Distance: %.5f  Linear Width: %.5f\n\r", obstacleList[obsCount].midDist, obstacleList[obsCount].linWidth);
            uart_sendStr(outputLine);

            obsCount++;
        }


    }

    if(obsCount > 0){
        sprintf(outputLine, "%d objects were found.\n\r", obsCount);
        uart_sendStr(outputLine);
    }
    else{
        uart_sendStr("No objects found.");
    }

    oi_free(sensor_data);
}

void scanTest(void){
    timer_init();
    lcd_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);
    right_calibration_value = RIGHT_CALIB;
    left_calibration_value = LEFT_CALIB;

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    char outputLine[50];

    int numScans = 2;

    int i, j;
    int buffSum;
    int buffAvg;
    int currScan;
    int change;

    int bufferSize = 5;

    int buffer[bufferSize];

    for(i = 0; i < bufferSize; i++){
        buffer[i] = multiScanIR(i, numScans);
    }

    for(i = bufferSize; i < 180; i += 1){
        buffSum = buffer[0];
        for(j = 1; j < bufferSize; j++){
            buffSum += buffer[j];
        }
        buffAvg = buffSum / bufferSize;


        currScan = multiScanIR(i, 2);

        change = currScan - buffAvg;

        buffer[i % bufferSize] = currScan;
        sprintf(outputLine, "%d, %d, %d, %d\n\r", i, currScan, buffAvg, change);
        uart_sendStr(outputLine);
    }

    oi_free(sensor_data);
}

void IRmultiScanTest(void){
    timer_init();
    lcd_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);
    right_calibration_value = RIGHT_CALIB;
    left_calibration_value = LEFT_CALIB;

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    char outputLine[50];



    int i;
    int currScan;
    for(i = 0; i < 180; i +=1){
        currScan = multiScanIR(i, 3);
        sprintf(outputLine, "%d, %d\n\r", i, currScan);
        uart_sendStr(outputLine);
    }

    oi_free(sensor_data);
}

void PingMultiScanTest(void){
    timer_init();
    lcd_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);
    right_calibration_value = RIGHT_CALIB;
    left_calibration_value = LEFT_CALIB;

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    char outputLine[50];



    int i;
    float currScan;
    for(i = 0; i < 180; i +=2){
        currScan = multiScanPing(i, 3);
        sprintf(outputLine, "%d: %f\n\r", i, currScan);
        uart_sendStr(outputLine);
    }

    oi_free(sensor_data);
}

void checkpoint1(void){
    timer_init();
    lcd_init();
    uart_interrupt_init();
    cyBOT_init_Scan(0b0111);
    right_calibration_value = RIGHT_CALIB;
    left_calibration_value = LEFT_CALIB;

    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    char outputLine[50];



    int i;
    cyBOT_Scan_t scan;
    for(i = 0; i < 180; i++){
        cyBOT_Scan(i, &scan);
        sprintf(outputLine, "%d: %d\n\r", i, scan.IR_raw_val);
        uart_sendStr(outputLine);
    }

    oi_free(sensor_data);
}

void sensorTest(){
    timer_init();
   lcd_init();
   uart_interrupt_init();
   cyBOT_init_Scan(0b0111);
   right_calibration_value = RIGHT_CALIB;
   left_calibration_value = LEFT_CALIB;

   oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
   oi_init(sensor_data); // do this only once at start of main()

   char outputLine[100];
   int i;
   float distance;
   for(i = 0; i < 50; i++){
       distance = multiScanPing(90, 3);
       sprintf(outputLine, "%.5f\n\r", distance);
       uart_sendStr(outputLine);
       timer_waitMillis(250);
   }

   oi_free(sensor_data);
}

void calibrateMovement(void){
   timer_init();
   lcd_init();
   uart_interrupt_init();
   cyBOT_init_Scan(0b0111);
   right_calibration_value = RIGHT_CALIB;
   left_calibration_value = LEFT_CALIB;

   oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
   oi_init(sensor_data); // do this only once at start of main()

   lcd_printf("Turning 90 degrees");
   turn_right(sensor_data, 90);

   oi_free(sensor_data);
}
/*void manAndAuto(){
    timer_init();
   lcd_init();
   uart_interrupt_init();
   cyBOT_init_Scan(0b0111);
   right_calibration_value = RIGHT_CALIB;
   left_calibration_value = LEFT_CALIB;

   oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
   oi_init(sensor_data); // do this only once at start of main()

   char outputLine[100];

   int maxObs = 10;

  struct obstacle obstacleList[MAX_OBSTACLES];
  int angle = 5;
  int obsCount = 0;
  int nextObs;

  command_byte0 = 'q';
  command_byte1 = 't';

  uart_sendStr("\n\rStarting\n\r");

  int manual = 0;
  int exit = 0;

  while(exit == 0){
      uart_sendStr("\rAutoMode\n\r");

      while(manual == 0){
          if(command_flag0){
                command_flag0 = 0;
                exit = 1;
                break;
            }

          if(command_flag1){
              command_flag1 = 0;
              manual = 1;
              break;
          }


      }

      uart_sendStr("\rManual\n\r");

      while (manual == 1){

          if(command_flag0){
              command_flag0 = 0;
              exit = 1;
              break;
          }
          if(command_flag1){
                command_flag1 = 0;
                manual = 0;
                break;
            }


      }
  }

  uart_sendStr("\n\rExiting");
}*/
