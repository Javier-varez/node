/*
 * VBAT.c
 *
 *  Created on: 22 may. 2017
 *      Author: David Llamazares, Javier Andrés
 */

#include "VBAT.h"
#include <stdlib.h>
#include <string.h>

#define 	VBAT_TIMEOUT 			1000

#define 	VBAT_ID					0x05
#define		DEFAULT_PERIOD_S		5

void VBAT_init_sensor_structure(Sensor *sensor, ADC_HandleTypeDef *hadc) {
	analog_sensor_init(sensor, "VBAT",VBAT_ID, NULL, DEFAULT_PERIOD_S, hadc, 'V');

	Sensor_Analog *VBAT = &sensor->data.analog;

	VBAT->channel.Rank = 1;
	VBAT->channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	VBAT->channel.Offset = 0;
	VBAT->channel.Channel = ADC_CHANNEL_12;
}


