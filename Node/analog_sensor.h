/*
 * analog_sensor.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef ANALOG_SENSOR_H_
#define ANALOG_SENSOR_H_

#include "stm32f4xx.h"

typedef struct {
	ADC_HandleTypeDef *hadc;
	uint32_t channel;
} Sensor_Analog;

#endif /* ANALOG_SENSOR_H_ */
