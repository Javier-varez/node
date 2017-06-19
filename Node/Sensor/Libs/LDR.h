#ifndef LDR_H_
#define LDR_H_

#include "sensor.h"

#define 	LDR_ID				0x06

typedef struct {
	uint16_t LDR; // Battery voltage in volts
} LDR_Out_Data;

Sensor_Func_Table LDR_Function_Table;

int LDR_init_sensor_structure(Sensor *sensor, void *handler, uint16_t sampling_period_s);
int LDR_remove(Sensor *sensor);

extern Sensor_Probe_Intf LDR_intf;

#endif /* LDR_H_ */
