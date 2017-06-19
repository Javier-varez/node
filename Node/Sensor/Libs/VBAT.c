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
#define		VBAT_DEFAULT_PERIOD_S		5

Sensor_Probe_Intf VBAT_intf = {
	.init = VBAT_init_sensor_structure,
	.remove = VBAT_remove
};

int VBAT_init_sensor_structure(Sensor *sensor, void *handler, uint16_t sampling_period_s) {
	analog_sensor_init(sensor, "VBAT", VBAT_ID, NULL, sampling_period_s ? sampling_period_s : VBAT_DEFAULT_PERIOD_S,
					   (ADC_HandleTypeDef*)handler, 'V');

	Sensor_Analog *VBAT = &sensor->data.analog;

	VBAT->channel.Rank = 1;
	VBAT->channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	VBAT->channel.Offset = 0;
	VBAT->channel.Channel = ADC_CHANNEL_11;

	return 0;
}

int VBAT_remove(Sensor *sensor) {
	return 0;
}



