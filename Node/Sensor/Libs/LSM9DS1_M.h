/*
 * LSM9DS1_M.h
 *
 *  Created on: Apr 15, 2017
 *      Author: javier
 */

#ifndef LSM9DS1_M_H_
#define LSM9DS1_M_H_

#include "sensor.h"

#define LSM9DS1_M_ID			0x04

typedef struct {
	int16_t magnetometer[3]; // Acelerometro (x,y,z): g
} LSM9DS1_M_Out_Data;

int LSM9DS1_M_init(struct sensor_struct *sensor, void* handler, uint16_t sampling_period_s);
int LSM9DS1_M_remove(Sensor *sensor);

extern Sensor_Probe_Intf LSM9DS1_M_intf;

#endif /* LSM9DS1_M_H_ */
