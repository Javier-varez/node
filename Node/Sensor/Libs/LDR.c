/*
 * LDR.c
 *
 *  Created on: 22 may. 2017
 *      Author: David Llamazares, Javier Andrés
 */

#include "LDR.h"
#include <stdlib.h>
#include <string.h>

#define 	LDR_ID				0x06
#define		DEFAULT_PERIOD_S	5

void LDR_init_sensor_structure(Sensor *sensor, ADC_HandleTypeDef *hadc) {
	analog_sensor_init(sensor, "LDR",LDR_ID, NULL, DEFAULT_PERIOD_S, hadc, 'L');

	Sensor_Analog *LDR = &sensor->data.analog;

	LDR->channel.Rank = 1;
	LDR->channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	LDR->channel.Offset = 0;
	LDR->channel.Channel = ADC_CHANNEL_12;
}


