#include "sensor.h"
#include <string.h>

static Sensor_Func_Table analog_func_tbl = {
		.init = analog_sensor_init,
	};

int analog_sensor_init(Sensor *sensor, char *name, uint8_t sensorID,
		Sensor_Func_Table *func_tbl, uint32_t sampling_period_s) {
	if (sensor != NULL) {
		if (strlen(name) < NAME_MAXLEN-1) strcpy(sensor->name, name);

		sensor->interface = SI_Analog;

		if (analog_func_tbl != NULL) sensor->func_tbl = analog_func_tbl;
		else sensor->func_tbl = &analog_func_tbl;

		sensor->sensorID = sensorID;
		sensor->sampling_period_s = sampling_period_s;

		return 0;
	}

	return 1;
}
