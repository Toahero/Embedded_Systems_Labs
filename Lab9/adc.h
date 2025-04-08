/*
 * adc.h
 *
 *  Created on: Mar 31, 2025
 *      Author: jagaul
 */

#ifndef ADC_H_
#define ADC_H_

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>

void adc_init (void);

uint16_t adc_read (void);

#endif /* ADC_H_ */
