#ifndef VBAT_H_
#define VBAT_H_

#include "sensor.h"

Sensor_Func_Table VBAT_Function_Table;

void VBAT_init_sensor_structure(Sensor *sensor, ADC_HandleTypeDef *hadc);

#endif /* VBAT_H_ */
