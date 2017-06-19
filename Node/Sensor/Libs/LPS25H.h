/*
 * LPS25H.h
 *
 *  Created on: Apr 12, 2017
 *      Author: javier
 */

#ifndef LPS25H_H_
#define LPS25H_H_

#include "sensor.h"

#define LPS25H_ID				0x02

typedef struct {
	float pressure;		// Presion Atmosferica: hPa
	float temperature;	// Temperatura: °C
} LPS25H_Out_Data;

int LPS25H_init(struct sensor_struct *sensor, void *handler, uint16_t sampling_period_s);
int LPS25H_remove(Sensor *sensor);

extern Sensor_Probe_Intf LPS25H_intf;

#endif /* LPS25H_H_ */
