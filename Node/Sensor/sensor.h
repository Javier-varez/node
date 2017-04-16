/*
 * sensor.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "LList.h"

#include "analog_sensor.h"
#include "digital_sensor.h"
#include "i2c_sensor.h"
#include "sensor_list.h"

#define NAME_MAXLEN 15

typedef enum {
	SI_Analog,
	SI_Digital,
	SI_I2C,
	SI_Custom,
} Sensor_Interface;

struct sensor_struct;
typedef struct sensor_struct Sensor;

typedef struct sensor_fuct_tbl {
	int (*probe)(Sensor *);
	int (*init)(Sensor *);
	int (*read)(Sensor *, void *data);
} Sensor_Func_Table;

typedef union {
	Sensor_Analog analog;
	Sensor_Digital digital;
	Sensor_I2C i2c;
} Sensor_Data;

struct sensor_struct {
	char name[NAME_MAXLEN];
	Sensor_Interface interface;
	Sensor_Func_Table *func_tbl;
	Sensor_Data data;
	void * out_data; // Sensor values will be stored here.
	void * custom;
};

void sensor_addDiscoverableSensor(LListElement **head, I2C_HandleTypeDef *hi2c, uint8_t index);
void sensor_readSensors(LListElement *head);

#endif /* SENSOR_H_ */
