#ifndef VBAT_H_
#define VBAT_H_

#include "sensor.h"

typedef struct {
	float vbat; // Battery voltage in volts
} VBAT_Out_Data;

int VBAT_init(Sensor *sensor);
int VBAT_pack_data();
int VBAT_read();
//int VBAT_remove(Sensor *sensor);

Sensor_Func_Table VBAT_Function_Table;

#endif /* VBAT_H_ */
