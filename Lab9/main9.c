/**
 * @file lab9_template.c
 * @author
 * Template file for CprE 288 Lab 9
 */

#include "Timer.h"
#include "lcd.h"
#include "ping.h"

// Uncomment or add any include directives that are needed

//#warning "Possible unimplemented functions"
//#define REPLACEME 0

int main(void) {
	timer_init(); // Must be called before lcd_init(), which uses timer functions
	lcd_init();
	ping_init();

	// YOUR CODE HERE
	lcd_printf("Init completed");

	while(1)
	{
	    ping_trigger();
	    timer_waitMillis(500);
	    lcd_printf("%d - %d", ping_getStartTime(), ping_getEndTime());
	    timer_waitMillis(500);

	}

}
