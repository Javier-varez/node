/*
 * LDR.c
 *
 *  Created on: 22 may. 2017
 *      Author: David Llamazares, Javier Andrés
 */

#include "LDR.h"
#include <stdlib.h>
#include <string.h>

#define		LDR_DEFAULT_PERIOD_S	5

Sensor_Probe_Intf LDR_intf = {
	.init = LDR_init_sensor_structure,
	.remove = LDR_remove
};

int LDR_init_sensor_structure(Sensor *sensor, void *handler, uint16_t sampling_period_s) {
	analog_sensor_init(sensor, "LDR", LDR_ID, NULL, sampling_period_s ? sampling_period_s : LDR_DEFAULT_PERIOD_S,
					   (ADC_HandleTypeDef*) handler, 'L');

	Sensor_Analog *LDR = &sensor->data.analog;

	LDR->channel.Rank = 1;
	LDR->channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	LDR->channel.Offset = 0;
	LDR->channel.Channel = ADC_CHANNEL_12;

	return 0;
}

int LDR_remove(Sensor *sensor) {
	return 0;
}
