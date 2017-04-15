/*
 * digital_sensor.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef DIGITAL_SENSOR_H_
#define DIGITAL_SENSOR_H_

#include "stm32f4xx.h"

typedef struct {
	GPIO_TypeDef * port;
	uint16_t pin;
} Sensor_Digital;

#endif /* DIGITAL_SENSOR_H_ */
