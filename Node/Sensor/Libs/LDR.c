/*
 * LDR.c
 *
 *  Created on: 22 may. 2017
 *      Author: David Llamazares, Javier Andrés
 */


#include "LDR.h"
#include <stdlib.h>
#include <string.h>

#define 	LDR_TIMEOUT 		1000

#define 	LDR_ID				0x06
#define		DEFAULT_PERIOD_S	5

int LDR_init(Sensor *sensor);
int LDR_packData(Sensor *sensor, uint8_t *data, uint8_t data_len);
int LDR_read(Sensor *sensor, void *out_data);

static Sensor_Func_Table LDR_func_tbl = {
	.init = LDR_init,
	.read = LDR_read,
	.packData = LDR_packData,
};

int LDR_init(Sensor *sensor) {
	return 0;
}

int LDR_read(Sensor *sensor, void *out_data) {

	Sensor_Analog *LDR = &sensor->data.analog;

	if (HAL_ADC_ConfigChannel(LDR->hadc, &LDR->channel) != HAL_OK)
	{
		//error
	}
	uint32_t ADCValue_LDR;
	if (HAL_ADC_PollForConversion(LDR->hadc, LDR_TIMEOUT) == HAL_OK)
	{
		HAL_ADC_Start(LDR->hadc);
		ADCValue_LDR = HAL_ADC_GetValue(LDR->hadc);
	}

	LDR_Out_Data *data = (LDR_Out_Data*) out_data;

	data->LDR = ADCValue_LDR;

	return 0;
}

int LDR_packData(Sensor *sensor, uint8_t *data, uint8_t data_len) {
	LDR_Out_Data *out_data = (LDR_Out_Data*)sensor->out_data;

	if (sizeof(LDR_Out_Data) + 2 <= data_len) {
		data[0] = 'L';
		data++;

		memcpy(data, &out_data->LDR, sizeof(out_data->LDR));
		data += sizeof(out_data->LDR);

		data[0] = '\0';

		return 1;
	}
	return 0;
}

void LDR_init_sensor_structure(Sensor *sensor, ADC_HandleTypeDef *hadc) {
	analog_sensor_init(sensor, "LDR",LDR_ID, &LDR_func_tbl, DEFAULT_PERIOD_S, hadc);

	Sensor_Analog *LDR = &sensor->data.analog;

	LDR->channel.Rank = 1;
	LDR->channel.SamplingTime = ADC_SAMPLETIME_480CYCLES;
	LDR->channel.Offset = 0;
	LDR->channel.Channel = ADC_CHANNEL_12;
}


