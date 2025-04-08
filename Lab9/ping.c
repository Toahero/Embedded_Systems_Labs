/**
 * Driver for ping sensor
 * @file ping.c
 * @author
 */

#include "ping.h"
#include "Timer.h"

// Global shared variables
// Use extern declarations in the header file

volatile uint32_t g_start_time = 0;
volatile uint32_t g_end_time = 0;
volatile enum{LOW, HIGH, DONE} g_state = LOW; // State of ping echo pulse

void ping_init (void){

  // YOUR CODE HERE

    IntRegister(INT_TIMER3B, TIMER3B_Handler);

    IntMasterEnable();

    // Disable timer 3 while setting values
    TIMER3_CTL_R &= ~0x80;

    //Set the last four pins to 0x4, to configure the timers in 16 bit mode
    (TIMER3_CFG_R & 0b1111)= 0x4;

    //Set timer 3b to edge count mode
    (TIMER3_TBMR_R & 0b0011) = 0x1;

    //Set the timer to count down
    TIMER3_TBMR_R &= ~0x10;

    //Set the event to both rising and falling edges
    TIMER3_CTL_R |= 0xC00;

    //Sets the prescaler so that the timer functions as a 24 bit mode
    TIMER3_TBPR_R |= 0xFF;

    //Sets the timer start value
    TIMER3_TBILR_R |= 0xFFFF;

    //Enable Interrupts
    TIMER3_IMR_R |= 0x200;


}

void ping_trigger (void){
    g_state = LOW;
    // Disable timer and disable timer interrupt
    TIMER3_CTL_R ???;
    TIMER3_IMR_R ???;
    // Disable alternate function (disconnect timer from port pin)
    GPIO_PORTB_AFSEL_R ???;

    // YOUR CODE HERE FOR PING TRIGGER/START PULSE

    // Clear an interrupt that may have been erroneously triggered
    TIMER3_ICR_R ???
    // Re-enable alternate function, timer interrupt, and timer
    GPIO_PORTB_AFSEL_R ???;
    TIMER3_IMR_R ???;
    TIMER3_CTL_R ???;
}

void TIMER3B_Handler(void){

  // YOUR CODE HERE
  // As needed, go back to review your interrupt handler code for the UART lab.
  // What are the first lines of code in the ISR? Regardless of the device, interrupt handling
  // includes checking the source of the interrupt and clearing the interrupt status bit.
  // Checking the source: test the MIS bit in the MIS register (is the ISR executing
  // because the input capture event happened and interrupts were enabled for that event?
  // Clearing the interrupt: set the ICR bit (so that same event doesn't trigger another interrupt)
  // The rest of the code in the ISR depends on actions needed when the event happens.

}

float ping_getDistance (void){

    // YOUR CODE HERE

}
