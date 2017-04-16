/*
 * HTS221.h
 *
 *  Created on: Apr 11, 2017
 *      Author: javier
 */

#ifndef HTS221_H_
#define HTS221_H_

#include "sensor.h"

typedef struct {
	float humidity;		// Humedad relativa: %
	float temperature;	// Temperatura: °C
} HTS221_Out_Data;

int HTS221_init(Sensor *sensor, I2C_HandleTypeDef *hi2c);
int HTS221_remove(Sensor *sensor);

Sensor_I2C_Probe_Intf HTS221_intf;

#endif /* HTS221_H_ */
