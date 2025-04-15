/// Simple 'Hello, world' program
/**
 * This program prints "Hello, world" to the LCD screen
 * @author Chad Nelson
 * @date 06/26/2012
 *
 * updated: phjones 9/3/2019
 * Description: Added timer_init call, and including Timer.h
 */

#include "Timer.h"
#include "lcd.h"

#define LCD_WIDTH 20

void lcd_rotatingBanner(char data[]);

int main (void) {

	timer_init(); // Initialize Timer, needed before any LCD screen functions can be called 
	              // and enables time functions (e.g. timer_waitMillis)

	lcd_init();   // Initialize the LCD screen.  This also clears the screen. 

	// Print "Hello, world" on the LCD
	//lcd_printf("Hello, world");

	//lcd_puts("Hello, world"); // Replace lcd_printf with lcd_puts
        // step through in debug mode and explain to TA how it works
	lcd_rotatingBanner("Test");

	// NOTE: It is recommended that you use only lcd_init(), lcd_printf(), lcd_putc, and lcd_puts from lcd.h.
       // NOTE: For time functions, see Timer.h

	return 0;
}

void lcd_rotatingBanner(char data[]){

    int combinedLength = LCD_WIDTH + strlen(data)+1;
    char combinedString[50] = "";

    int i;
    for(i = 0; i < LCD_WIDTH; i++){
        strcat(combinedString, " ");
    }

    strcat(combinedString, data);
    char outputString[LCD_WIDTH];

    char currChar;
    int startPos = 0;
    int currPos = 0;

    while(1){
        currPos = startPos;
        for(i = 0; i < LCD_WIDTH-1; i++){
            if(combinedString[currPos] == '\n'){
                currPos = 0;
            }
            currChar = combinedString[currPos];
            outputString[i] = currChar;

            currPos++;
        }
        lcd_printf(outputString);

        startPos = (startPos + 1) % strlen(combinedString);
        timer_waitMillis(300);
    }

}
