/*
 * sensor.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "analog_sensor.h"
#include "digital_sensor.h"
#include "i2c_sensor.h"
#include "spi_sensor.h"

#define NAME_MAXLEN 15

typedef enum {
	SI_Analog,
	SI_Digital,
	SI_I2C,
	SI_SPI,
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
	Sensor_SPI spi;
} Sensor_Data;

struct sensor_struct {
	char name[NAME_MAXLEN];
	Sensor_Interface interface;
	Sensor_Func_Table *func_tbl;
	Sensor_Data data;
	void * out_data; // Sensor values will be stored here.
	void * custom;
};

#endif /* SENSOR_H_ */
