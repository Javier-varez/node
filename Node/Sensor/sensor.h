/*
 * sensor.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef SENSOR_H_
#define SENSOR_H_

#include "FreeRTOS.h"
#include "task.h"

#include "LList.h"

#include "analog_sensor.h"
#include "digital_sensor.h"
#include "i2c_sensor.h"
#include "sensor_list.h"

#define NAME_MAXLEN 15
#define sensor_delay(x) (vTaskDelay(x/portTICK_PERIOD_MS))

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
	int (*read)(Sensor *, void *);
	int (*packData)(Sensor *, uint8_t *, uint8_t);
	int (*powerDown)(Sensor *);
	int (*powerUp)(Sensor *);
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
	uint32_t sampling_period_s;
	void * out_data; // Sensor values will be stored here.
	void * custom;
};

void sensor_addDiscoverableSensor(LListElement **head, I2C_HandleTypeDef *hi2c, uint8_t index);
void sensor_readSensors(LListElement *head, uint32_t id);
void sensor_setSamplingPeriod(Sensor *sensor, uint32_t sampling_time_s);

#endif /* SENSOR_H_ */
