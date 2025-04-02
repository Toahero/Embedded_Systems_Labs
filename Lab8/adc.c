/*
 * adc.c
 *
 *  Created on: Mar 31, 2025
 *      Author: jagaul
 */

#include "adc.h"

void adc_init (void){

    //Enabling the module

    //Set pin 1 to high, enabling clock for the ADC 0
    SYSCTL_RCGCADC_R |= 0b00000001;

    //Enable Clock for GPIO port B
    SYSCTL_RCGCGPIO_R |= 0b00000010;

    //Wait for GPIO B to finish setting up
    while ((SYSCTL_PRGPIO_R & 0b00000010) == 0) {};

    //Pin GPIO B4 is ADC compatible, corresponds to AIN10

    //enable alternate functionality on b4
    GPIO_PORTE_AFSEL_R |= 0b00010000;

    //disable digital functionality on b4
    GPIO_PORTE_DEN_R &= ~0b00010000;

    //Enable analog function of b4
    GPIO_PORTB_AMSEL_R |= 0b00010000;


    //Enabling the Sample Sequencer

    //Ensure that sample sequencer 0 is disabled before changing settings
    ADC0_ACTSS_R &= ~0b0001;

    //Use the GPIO input pin's ainx functionality as an interupt.
    ADC0_EMUX_R |= 0b1000;

    //Set all samples to come from ADC pin 10 (b4)
    ADC0_SSMUX10_R = 0xAAAAAAAA;

    //Re-enable SS0
    ADC0_ACTSS_R |= 0b0001;
}


uint16_t adc_read (void){

}
