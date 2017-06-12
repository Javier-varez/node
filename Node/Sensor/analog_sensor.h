/*
 * analog_sensor.h
 *

 */

#ifndef ANALOG_SENSOR_H_
#define ANALOG_SENSOR_H_

#include "sensor.h"
#include "stm32f4xx.h"


typedef struct {
	ADC_HandleTypeDef *hadc;
	ADC_ChannelConfTypeDef channel;
} Sensor_Analog;

typedef struct sensor_struct Sensor;
typedef struct sensor_fuct_tbl Sensor_Func_Table;

int analog_sensor_init(Sensor *sensor, char *name, uint8_t sensorID,
		Sensor_Func_Table *func_tbl, uint32_t sampling_period_s);

#endif /* ANALOG_SENSOR_H_ */
