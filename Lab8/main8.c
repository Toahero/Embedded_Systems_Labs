/*
 * main8.c
 *
 *  Created on: Apr 2, 2025
 *      Author: jagaul
 */


#include "Timer.h"
#include "lcd.h"
#include "adc.h"

//#include "cyBot_Scan.h"  // For scan sensors
//#include "uart-interrupt.h"
//#include "movement.h"

// Uncomment or add any include directives that are needed
//#include "open_interface.h"

int main(void) {
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    adc_init();

    int reading = 0;

    while(1){
        reading = adc_read();
        lcd_printf("%d", reading);
    }
}
