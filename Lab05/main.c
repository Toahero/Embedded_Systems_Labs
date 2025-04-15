/**
 * lab5_template.c
 *
 * Template file for CprE 288 Lab 5
 *
 * @author Zhao Zhang, Chad Nelson, Zachary Glanz
 * @date 08/14/2016
 *
 * @author Phillip Jones, updated 6/4/2019
 * @author Diane Rover, updated 2/25/2021, 2/17/2022
 */

#include "button.h"
#include "timer.h"
#include "lcd.h"

#include "uart.h"

#include "cyBot_uart.h"  // Functions for communicating between CyBot and Putty (via UART1)
                         // PuTTy: Baud=115200, 8 data bits, No Flow Control, No Parity, COM1

//#include "cyBot_Scan.h"  // Scan using CyBot servo and sensors


//#warning "Possible unimplemented functions"
//#define REPLACEME 0

void checkpoint1();
void checkpoint2();
void checkpoint3();
void checkpoint4();

int main(void) {
    checkpoint2();
}

void checkpoint1(){
    button_init();
        timer_init(); // Must be called before lcd_init(), which uses timer functions
        lcd_init();

        lcd_printf("Starting");


      // initialize the cyBot UART1 before trying to use it

      // (Uncomment ME for UART init part of lab)
        cyBot_uart_init_clean();  // Clean UART1 initialization, before running your UART1 GPIO init code
        //cyBot_uart_init_PHJ_first_half();

        // Complete this code for configuring the GPIO PORTB part of UART1 initialization (your UART1 GPIO init code)
         SYSCTL_RCGCGPIO_R |= 0b00000010;
           while ((SYSCTL_PRGPIO_R & 0b00000010) == 0) {};
             GPIO_PORTB_DEN_R |= 0b00000011;
             GPIO_PORTB_AFSEL_R |= 0b00000011;
         GPIO_PORTB_PCTL_R &= 0x00000000;     // Force 0's in the desired locations
         GPIO_PORTB_PCTL_R |= 0x00000011;     // Force 1's in the desired locations
             // Or see the notes for a coding alternative to assign a value to the PCTL field*/


        // (Uncomment ME for UART init part of lab)
            cyBot_uart_init_last_half();  // Complete the UART device configuration
            lcd_printf("Init Completed");
            // Initialize the scan
          // cyBOT_init_Scan();
            // Remember servo calibration function and variables from Lab 3

        // YOUR CODE HERE

        while(1)
        {

          cyBot_sendByte('F');

          timer_waitMillis(1000);

        }
}

void checkpoint2(){
    button_init();
        timer_init(); // Must be called before lcd_init(), which uses timer functions
        lcd_init();



        uart_init();

        char nextChar;
        char charBuffer[20];
        int numChars = 0;
        int i;

        lcd_clear();
        while(1){
            nextChar = uart_receive();
            numChars++;
            lcd_printf("\n%d", numChars);
            lcd_home();

            if((numChars >= 20) || (nextChar == '\r')){

                uart_sendChar('\n');
                uart_sendChar('\r');
                lcd_clear();
                for(i = 0; i < numChars; i++){
                    uart_sendChar(charBuffer[i]);
                    lcd_putc(charBuffer[i]);
                }
                numChars = 0;
                timer_waitMillis(1000);
            }
            else{
                charBuffer[numChars] = nextChar;
                lcd_putc(nextChar);
            }
        }
}

void checkpoint3(){
    button_init();
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();



    uart_init();

    char nextChar;
    char charBuffer[20];
    int numChars = 0;
    int i;

    lcd_clear();
    while(1){
        nextChar = uart_receive();
        if(nextChar == '\r'){
            uart_sendChar('\n');
            uart_sendChar('\r');
        }
        else{
            uart_sendChar(nextChar);
        }
    }
    return 0;
}

void checkpoint4(){
    button_init();
        timer_init(); // Must be called before lcd_init(), which uses timer functions
        lcd_init();

        uart_init();

        char nextChar;
        char message1[20] = "message 1\n\r";
        int numChars = 0;
        int i;

        lcd_clear();
        while(1){
            lcd_printf("%d", numChars);
            nextChar = uart_receive();
            numChars++;

            if(numChars > 20){
                uart_sendChar('\n');
                uart_sendChar('\r');
                numChars = 0;
            }

            if(nextChar == '1'){
                uart_sendStr(message1);
            }
            else if(nextChar == '\r'){
                uart_sendChar('\n');
                uart_sendChar('\r');
                numChars = 0;
            }
            else{
                uart_sendChar(nextChar);
            }
        }
        return 0;
}
