/*
 * LSM9DS1_M.h
 *
 *  Created on: Apr 15, 2017
 *      Author: javier
 */

#ifndef LSM9DS1_M_H_
#define LSM9DS1_M_H_

#include "sensor.h"

typedef struct {
	int16_t magnetometer[3]; // Acelerometro (x,y,z): g
} LSM9DS1_M_Out_Data;

int LSM9DS1_M_init(Sensor *sensor, I2C_HandleTypeDef *hi2c);
int LSM9DS1_M_remove(Sensor *sensor);

extern Sensor_I2C_Probe_Intf LSM9DS1_M_intf;

#endif /* LSM9DS1_M_H_ */
