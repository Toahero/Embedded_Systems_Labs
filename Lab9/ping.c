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
    //Initialize GPIO pin B3

    SYSCTL_RCGCGPIO_R |= 0b00000010; //enable clock to GPIO port B
    while ((SYSCTL_PRGPIO_R & 0b00000010) == 0) {}; //Wait for GPIOB to be ready


    GPIO_PORTB_DEN_R |= 0b00001000; //Enable digital functionality on pin B3

    GPIO_PORTB_AFSEL_R |= 0b00001000; //Enable alternate functionality on pin B3

    GPIO_PORTB_PCTL_R = 0x7; //Configure B3 to respond to timer 3

    //Timer Initialization

    SYSCTL_RCGCTIMER_R |= 0x08; //enable clock to Timer 3
    while((SYSCTL_PRTIMER_R & 0x08) == 0) {}; //wait for Timer 3 to be ready


    TIMER3_CTL_R &= ~0x100; // Disable timer 3 while setting values


    TIMER3_CFG_R =(0b1111 & 0x4); //Set the last four pins to 0x4, to configure the timers in 16 bit mode


    TIMER3_TBMR_R = 0b00000111; //Set timer 3b to edge time, and time capture mode


    TIMER3_TBMR_R &= ~0x10; //Set the timer to count down


    TIMER3_CTL_R |= 0xC00; //Set the event to both rising and falling edges


    TIMER3_TBPR_R |= 0xFF; //Sets the prescaler so that the timer functions as a 24 bit mode


    TIMER3_TBILR_R |= 0xFFFF; //Sets the timer start value



    //Set up timer interrupts
    TIMER3_ICR_R  |= 0x400;//Clear capture interrupt flag

    TIMER3_IMR_R |= 0x400; //Enable Event Capture Interrupt

    //Set up NVIC

    NVIC_PRI9_R = (NVIC_PRI9_R & 0xFFFFFF0F) | 0x20; //Sets priority to timer 3b, masking other values

    NVIC_EN1_R |= 0x10; //Set bit for, to enable timer 3b interrupts

    IntRegister(INT_TIMER3B, TIMER3B_Handler);

    IntMasterEnable();

    TIMER3_CTL_R |= 0x100;//Re-enable the timer
}

void ping_trigger (void){
    g_state = LOW;
    // Disable timer and disable timer interrupt
    TIMER3_CTL_R  &= ~0x100;
    TIMER3_IMR_R &= ~0x400;;
    // Disable alternate function (disconnect timer from port pin)
    GPIO_PORTB_AFSEL_R &= ~0b00001000;

    // YOUR CODE HERE FOR PING TRIGGER/START PULSE
    GPIO_PORTB_DIR_R |= 0b00001000; //Set pin 3 as output
    GPIO_PORTB_DATA_R &= 0xF7;//Set pin 3 Low
    GPIO_PORTB_DATA_R &= 0x8;//Set pin3 High
    timer_waitMicros(5); //Wait 5 microseconds
    GPIO_PORTB_DATA_R &= 0xF7;//Set pin 3 Low

    GPIO_PORTB_DIR_R &= ~0b00001000; //Set pin 3 as input

    // Clear an interrupt that may have been erroneously triggered
    TIMER3_ICR_R  |= 0x400;//Clear capture interrupt flag
    // Re-enable alternate function, timer interrupt, and timer
    GPIO_PORTB_AFSEL_R |= 0b00001000;
    TIMER3_IMR_R |= 0x400;
    TIMER3_CTL_R |= 0x100;//Re-enable the timer
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

    if(g_state == DONE){
        TIMER3_ICR_R  |= 0x400;
        return;
    }

    if(g_state == LOW){
        g_start_time =  TIMER3_TBR_R;
        g_state = HIGH;
        TIMER3_ICR_R  |= 0x400;
    }
    else{
        g_end_time = TIMER3_TBR_R;
        TIMER3_ICR_R  |= 0x400;
        g_state = DONE;
    }

}

float ping_getDistance (void){

    // YOUR CODE HERE

}

int ping_getStartTime(void){
    return g_start_time;
}

int ping_getEndTime(void){
    return g_end_time;
}
