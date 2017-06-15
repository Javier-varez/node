#include "sensor.h"
#include <string.h>

int analog_func_init(Sensor *sensor) {
	return 0;
}

int analog_func_read(Sensor *sensor, void *out_data){
	return 0;
}

int analog_func_packData(Sensor *sensor, uint8_t *data, uint8_t data_len){
	return 0;
}

static Sensor_Func_Table analog_func_tbl = {
	.init = analog_func_init,
	.read = analog_func_read,
	.packData = analog_func_packData
};

int analog_sensor_init(Sensor *sensor, char *name, uint8_t sensorID,
		Sensor_Func_Table *func_tbl, uint32_t sampling_period_s, ADC_HandleTypeDef *hadc) {

	if (sensor != NULL) {
		if (strlen(name) < NAME_MAXLEN-1) strcpy(sensor->name, name);

		sensor->interface = SI_Analog;

		if (func_tbl != NULL) sensor->func_tbl = func_tbl;
		else sensor->func_tbl = &analog_func_tbl;

		sensor->sensorID = sensorID;
		sensor->sampling_period_s = sampling_period_s;

		sensor->data.analog.hadc = hadc;

		return 0;
	}

	return 1;
}
