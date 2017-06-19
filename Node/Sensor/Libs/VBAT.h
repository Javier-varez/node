#ifndef VBAT_H_
#define VBAT_H_

#include "sensor.h"

#define 	VBAT_ID					0x05

Sensor_Func_Table VBAT_Function_Table;

int VBAT_init_sensor_structure(Sensor *sensor, void *handler, uint16_t sampling_period_s);
int VBAT_remove(Sensor *sensor);

extern Sensor_Probe_Intf VBAT_intf;

#endif /* VBAT_H_ */
