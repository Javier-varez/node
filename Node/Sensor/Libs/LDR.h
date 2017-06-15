#ifndef LDR_H_
#define LDR_H_

#include "sensor.h"

typedef struct {
	uint16_t LDR; // Battery voltage in volts
} LDR_Out_Data;

Sensor_Func_Table LDR_Function_Table;

void LDR_init_sensor_structure(Sensor *sensor, ADC_HandleTypeDef *hadc);

#endif /* LDR_H_ */
