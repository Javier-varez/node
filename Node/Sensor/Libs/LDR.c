/*
 * LDR.c
 *
 *  Created on: 22 may. 2017
 *      Author: David Llamazares, Javier Andrés
 */

#include "LDR.h"
#include <stdlib.h>
#include <string.h>
#include <init_periph.h>
#define TIMEOUT 1000

#define 	LDR_ID			0x06 //TODO: check if 0x06 is not colliding with other sensors
#define		DEFAULT_PERIOD_S		5

static Sensor_Func_Table LDR_func_tbl = {
	.init = LDR_init,
	.read = LDR_read,
	.packData = LDR_packData,
};

Sensor_Analog LDR;
ADC_ChannelConfTypeDef adcChannel;


int LDR_init(Sensor *sensor) {


adc_init();
analog_sensor_init(sensor, "LDR",LDR_ID, &LDR_func_tbl, DEFAULT_PERIOD_S);
return 0;
}

int LDR_read(Sensor *sensor) {

	LDR.channel=ADC_CHANNEL_12;
	adcChannel.Rank = 1;
	adcChannel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	adcChannel.Offset = 0;

	if (HAL_ADC_ConfigChannel(&LDR.hadc, &adcChannel) != HAL_OK)
	{
		//error
	}
	uint32_t ADCValue_LDR;
	int LDR_Level;
	//int MeasurementCounter_LDR;
	if (HAL_ADC_PollForConversion(&LDR.hadc, TIMEOUT) == HAL_OK)
	{
		HAL_ADC_Start(&LDR.hadc);
		ADCValue_LDR = HAL_ADC_GetValue(&LDR.hadc);
		//LDR_Level = TODO
	//	MeasurementCounter_LDR=ADCValue_LDR;
	}
	return 0;
}

int LDR_packData(Sensor *sensor, uint8_t *data, uint8_t data_len) {
	LDR_Out_Data *out_data = (LDR_Out_Data*)sensor->out_data;

	if (sizeof(LDR_Out_Data) + 2 <= data_len ) {
		data[0] = 'L';
		data++;

		memcpy(data, &out_data->LDR, sizeof(out_data->LDR));
		data += sizeof(out_data->LDR);

		data[0] = '\0';

		return 1;
	}
	return 0;
}


