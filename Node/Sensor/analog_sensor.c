#include "sensor.h"
#include <string.h>

#define ANALOG_TIMEOUT 1000

int analog_func_init(Sensor *sensor) {
	return 0;
}

int analog_func_read(Sensor *sensor, void *out_data){

	Sensor_Analog *sensor_analog = &sensor->data.analog;

	if (HAL_ADC_ConfigChannel(sensor_analog->hadc, &sensor_analog->channel) != HAL_OK)
	{
		//error
	}
	uint32_t ADCValue; //12 bits (0 to 4095)
	HAL_ADC_Start(sensor_analog->hadc);
	if (HAL_ADC_PollForConversion(sensor_analog->hadc, ANALOG_TIMEOUT) == HAL_OK)
	{
		ADCValue = HAL_ADC_GetValue(sensor_analog->hadc);
	}

	uint16_t *data = (uint16_t*) out_data;

	*data = ADCValue;

	return 0;
}

int analog_func_packData(Sensor *sensor, uint8_t *data, uint8_t data_len){
	uint16_t *out_data = (uint16_t*)sensor->out_data;

	if (sizeof(uint16_t) + 2 <= data_len) {
		data[0] = sensor->data.analog.identifier;
		data++;

		memcpy(data, out_data, sizeof(uint16_t));
		data += sizeof(uint16_t);

		data[0] = '\0';

		return 1;
	}
	return 0;
}

static Sensor_Func_Table analog_func_tbl = {
	.init = analog_func_init,
	.read = analog_func_read,
	.packData = analog_func_packData
};

int analog_sensor_init(Sensor *sensor, char *name, uint8_t sensorID,
		Sensor_Func_Table *func_tbl, uint32_t sampling_period_s, ADC_HandleTypeDef *hadc,
		uint8_t identifier) {

	if (sensor != NULL) {
		if (strlen(name) < NAME_MAXLEN-1) strcpy(sensor->name, name);

		sensor->interface = SI_Analog;

		if (func_tbl != NULL) sensor->func_tbl = func_tbl;
		else sensor->func_tbl = &analog_func_tbl;

		sensor->sensorID = sensorID;
		sensor->sampling_period_s = sampling_period_s;

		sensor->data.analog.hadc = hadc;
		sensor->data.analog.identifier = identifier;

		sensor->out_data = (void*)malloc(sizeof(uint16_t));

		return 0;
	}

	return 1;
}
