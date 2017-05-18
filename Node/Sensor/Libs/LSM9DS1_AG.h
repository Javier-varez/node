/*
 * LSM9DS1_AG.h
 *
 *  Created on: Apr 13, 2017
 *      Author: javier
 */

#ifndef LSM9DS1_AG_H_
#define LSM9DS1_AG_H_

#include "sensor.h"

#define LSM9DS1_AG_ID			0x03

typedef struct {
	int16_t accelerometer[3]; // Acelerometro (x,y,z): g
	int16_t gyroscope[3];		// Giroscopo (x,y,z): g
} LSM9DS1_AG_Out_Data;

int LSM9DS1_AG_init(Sensor *sensor, I2C_HandleTypeDef *hi2c, uint16_t sampling_period_s);
int LSM9DS1_AG_remove(Sensor *sensor);

extern Sensor_I2C_Probe_Intf LSM9DS1_AG_intf;

#endif /* LSM9DS1_AG_H_ */
