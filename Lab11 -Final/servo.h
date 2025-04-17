/*
 * servo.h
 *
 *  Created on: Apr 15, 2025
 *      Author: jagaul
 */

#ifndef SERVO_H_
#define SERVO_H_

#include <inc/tm4c123gh6pm.h>
#include <stdint.h>
#include "Timer.h"
#include "button.h"
#include "lcd.h"

void servo_setMatch(unsigned int match);

void servo_setLeft(unsigned int leftCoord);
void servo_setRight(unsigned int rightCoord);

int get_match(void);

void servo_init(void);

void servo_move(uint16_t angle_deg);

void servo_setEdges(unsigned int deg0, unsigned int deg180);

void servo_calibrate(void);

#endif /* SERVO_H_ */
