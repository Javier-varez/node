#ifndef LDR_H_
#define LDR_H_

#include "sensor.h"

typedef struct {
	float LDR; // Battery voltage in volts
} LDR_Out_Data;

int LDR_init(Sensor *sensor);
int LDR_pack_data();
int LDR_read();

Sensor_Func_Table LDR_Function_Table;

#endif /* LDR_H_ */
