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

// Uncomment or add any include directives that are needed
#include "open_interface.h"
// #include "movement.h"
// #include "button.h"


//#warning "Possible unimplemented functions"
//#define REPLACEME 0

void checkpoint1(void);

int main(void) {
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    uart_interrupt_init();

    cyBOT_init_Scan(0b0111);
    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()

    command_byte = 's';

    cyBOT_Scan_t scan;
    int i;
    char inputVal;

    while(1){
        inputVal = uart_receive();

        if(inputVal == 'g'){
            for(i=0; i<= 180; i+=2){
                cyBOT_Scan(i, &scan);

                if(command_flag == 1){
                    command_flag = 0;
                    break;
                }
            }
        }

    }

}

void checkpoint1(void){
    timer_init(); // Must be called before lcd_init(), which uses timer functions
    lcd_init();
    uart_interrupt_init();

    cyBOT_init_Scan(0b0111);
    oi_t *sensor_data = oi_alloc(); // do this only once at start of main()
    oi_init(sensor_data); // do this only once at start of main()


    // YOUR CODE HERE

    cyBOT_Scan_t scan;
    int i;
    char inputVal;

    for(i=0; i<= 180; i+=2){
        cyBOT_Scan(i, &scan);
        inputVal = uart_recieve_nonblocking();

        if(inputVal == 's'){
            break;
        }
    }
    oi_free(sensor_data); // do this once at end of main()
}
