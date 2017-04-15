/*
 * LPS25H.h
 *
 *  Created on: Apr 12, 2017
 *      Author: javier
 */

#ifndef LPS25H_H_
#define LPS25H_H_

#include "sensor.h"

typedef struct {
	float pressure;		// Presion Atmosferica: hPa
	float temperature;	// Temperatura: °C
} LPS25H_Out_Data;

int LPS25H_init(Sensor *sensor, I2C_HandleTypeDef *hi2c);
int LPS25H_remove(Sensor *sensor);

extern Sensor_I2C_Probe_Intf LPS25H_intf;

#endif /* LPS25H_H_ */
