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
    SYSCTL_RCGCADC_R |= 0x01;

    //Enable Clock for GPIO port B
    SYSCTL_RCGCGPIO_R |= 0x02;

    //Wait for GPIO B to finish setting up
    while ((SYSCTL_PRGPIO_R & 0x02) == 0) {};



    //Pin GPIO B4 is ADC compatible, corresponds to AIN10

    //disable digital functionality on b4
    GPIO_PORTB_DEN_R &= ~0x10;

    //enable alternate functionality on b4
    GPIO_PORTB_AFSEL_R |= 0x10;

    //Enable analog function of b4
    GPIO_PORTB_AMSEL_R |= 0x10;




    GPIO_PORTB_ADCCTL_R |= 0b00010000;

    //Enabling the Sample Sequencer

    //Ensure that sample sequencer 0 is disabled before changing settings
    ADC0_ACTSS_R &= ~0b0001;

    //Set the trigger to software based
    ADC0_EMUX_R |= 0x0;

    //begin sampling
    ADC0_PSSI_R |= 0x1;

    //Assign ADC input 10
    ADC0_SSMUX0_R &= ~0b1001;
    ADC0_SSMUX0_R |= 0xA;

    //Enable the first sample and end bits
    ADC0_SSCTL0_R |= 0x6;

    ADC0_IM_R |= 0x01;

    //Activate averaging
    ADC0_SAC_R = 0x02;

    //Reenable sammple sequencer
    ADC0_ACTSS_R |= 0b0001;
}


uint16_t adc_read (void){

    ADC0_PSSI_R = 0x0001;

    while((ADC0_RIS_R & 0x01) == 0){};

    ADC0_ISC_R |= 0x0001;

    uint16_t result = ADC0_SSFIFO0_R;

    return result;
}

double adc_dist(void){

    ADC0_PSSI_R = 0x0001;

    while((ADC0_RIS_R & 0x01) == 0){};

    ADC0_ISC_R |= 0x0001;

    uint16_t reading = ADC0_SSFIFO0_R;

    double dist = 124725 * pow(reading, -1.182);

    return dist;
}
