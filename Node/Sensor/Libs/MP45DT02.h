/*
 * MP45DT02.h
 *
 *  Created on: May 15, 2017
 *      Author: javier
 */

#ifndef SENSOR_LIBS_MP45DT02_H_
#define SENSOR_LIBS_MP45DT02_H_

#include "sensor.h"

#define MAX_BANDS 6

typedef struct {
	float energy[MAX_BANDS];
} MP45DT02_Out_Data;

int MP45DT02_init(Sensor *sensor);

#endif /* SENSOR_LIBS_MP45DT02_H_ */
