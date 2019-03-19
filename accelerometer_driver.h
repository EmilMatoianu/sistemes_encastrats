/*
 * accelerometer_driver.h
 *
 *  Created on: 26 oct. 2017
 *      Author: toni
 */

#ifndef ACCELEROMETER_DRIVER_H_
#define ACCELEROMETER_DRIVER_H_

#define CONVERSION_SCALE    3250.0
#define CONVERSION_OFFSET   8150.0

void init_Accel(void);

void Accel_read(float *values);

#endif /* ACCELEROMETER_DRIVER_H_ */
