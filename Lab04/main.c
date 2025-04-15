/**
 * lab4_template.c
 *
 * Template file for CprE 288 lab 4
 *
 * @author Zhao Zhang, Chad Nelson, Zachary Glanz
 * @date 08/14/2016
 */

#include "button.h"
#include "Timer.h"
#include "lcd.h"
#include "cyBot_uart.h"  // Functions for communicating between CyBot and Putty (via UART)
                         // PuTTy: Baud=115200, 8 data bits, No Flow Control, No Parity, COM1

//#warning "Possible unimplemented functions"
//#define REPLACEME 0

void sendString(char inputString[]);

int main(void) {
	button_init();
	timer_init(); // Must be called before lcd_init(), which uses timer functions
	lcd_init();
	cyBot_uart_init();
	            // Don't forget to initialize the cyBot UART before trying to use it

	char messages[4][50];


	strcpy(messages[0], "First Pressed\n\r");
	strcpy(messages[1], "Second Pressed\n\r");
	strcpy(messages[2], "Third Pressed\n\r");
	strcpy(messages[3], "Fourth Pressed\n\r");



	int highestButton = 0;
	int prevButton = 0;
	
	sendString("Beginning Button Test\n\r");

	while(1)
	{

	    highestButton = button_getButton();
	    lcd_printf("%d", highestButton);


	    if((highestButton != prevButton) && (highestButton != 0)){
	        sendString(messages[highestButton - 1]);
	        prevButton = highestButton;
	    }


	}

}

void sendString(char inputString[]){
    int i;
    for(i = 0; i < strlen(inputString); i++){
        char nextByte = inputString[i];
        cyBot_sendByte(nextByte);
    }
}
