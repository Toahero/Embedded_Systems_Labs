/*
 * movment.h
 *
 *  Created on: Feb 6, 2025
 *      Author: jagaul
 */

#ifndef MOVEMENT_H_
#define MOVEMENT_H_

#include <open_interface.h>
#include <lcd.h>

double move_forward(oi_t *sensor_data, double distance_mm);

double move_backward(oi_t *sensor_data, double distance_mm);

double move_forward_bumpInt(oi_t *sensor_data, double distance_mm);

double turn_right (oi_t *sensor, double degrees);

double turn_left (oi_t *sensor, double degrees);

#endif /* MOVEMENT_H_ */
