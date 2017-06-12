/*
 * VBAT.c
 *
 *  Created on: 22 may. 2017
 *      Author: David Llamazares, Javier Andrés
 */

#include "VBAT.h"
#include <stdlib.h>
#include <string.h>
#include <init_periph.h>
#define TIMEOUT 1000

#define 	VBAT_ID			0x05 //TODO: check if 0x06 is not colliding with other sensors
#define		DEFAULT_PERIOD_S		5

static Sensor_Func_Table VBAT_func_tbl = {
	.init = VBAT_init,
	.read = VBAT_read,
	.packData = VBAT_packData,
};

Sensor_Analog VBAT;
ADC_ChannelConfTypeDef adcChannel;


int VBAT_init(Sensor *sensor) {


adc_init();
analog_sensor_init(sensor, "VBAT",VBAT_ID, &VBAT_func_tbl, DEFAULT_PERIOD_S);
return 0;
}

int VBAT_read(Sensor *sensor) {

	VBAT.channel=ADC_CHANNEL_11;
	adcChannel.Rank = 1;
	adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcChannel.Offset = 0;

	if (HAL_ADC_ConfigChannel(&VBAT.hadc, &adcChannel) != HAL_OK)
	{
		//error
	}
	uint32_t ADCValue_VBAT;
	float VBAT_Volts;
	//int MeasurementCounter_VBAT;
	if (HAL_ADC_PollForConversion(&VBAT.hadc, TIMEOUT) == HAL_OK)
	{
		HAL_ADC_Start(&VBAT.hadc);
		ADCValue_VBAT = HAL_ADC_GetValue(&VBAT.hadc);
		//VBAT_Volts = TODO
	//	MeasurementCounter_VBAT=ADCValue_VBAT;
	}
	return 0;
}

int VBAT_packData(Sensor *sensor, uint8_t *data, uint8_t data_len) {
	VBAT_Out_Data *out_data = (VBAT_Out_Data*)sensor->out_data;

	if (sizeof(VBAT_Out_Data) + 2 <= data_len ) {
		data[0] = 'V';
		data++;

		memcpy(data, &out_data->vbat, sizeof(out_data->vbat));
		data += sizeof(out_data->vbat);

		data[0] = '\0';

		return 1;
	}
	return 0;
}


