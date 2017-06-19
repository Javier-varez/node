/*
 * sensor.c
 *
 *  Created on: Apr 16, 2017
 *      Author: javier
 */

#include "sensor.h"
#include "node.h"
#include <stdlib.h>

#include "FreeRTOS.h"
#include "task.h"

void sensor_addDiscoverableSensor(LListElement **head, I2C_HandleTypeDef *hi2c, uint8_t index) {
	Sensor *sensor = (Sensor*)malloc(sizeof(Sensor));
	if (sensor == NULL) return;

	if (discoverable_devices[index]->init(sensor, hi2c, 0) == 0) {
		sensor->func_tbl->init(sensor);

		if (*head == NULL) {
			*head = LList_CreateList((void *)sensor);
		} else {
			LList_AppendElement(*head, (void*) sensor);
		}
	}
}

void sensor_addSensor(LListElement **head, ADC_HandleTypeDef *hadc, I2C_HandleTypeDef *hi2c, struct_sensor_list *sensor_list, uint32_t sampling_period_s) {
	Sensor *sensor = (Sensor*)malloc(sizeof(Sensor));
	if (sensor == NULL) return;

	void * handler = (sensor_list->interface == SI_I2C) ? (void*) hi2c :
					 (sensor_list->interface == SI_Analog) ? (void*) hadc :
					 hadc;

	if (sensor_list->probe_intf->init(sensor, handler, sampling_period_s) == 0) {
		sensor->func_tbl->init(sensor);

		if (*head == NULL) {
			*head = LList_CreateList((void *)sensor);
		} else {
			LList_AppendElement(*head, (void*) sensor);
		}
	}
}

void sensor_readSensors(LListElement *head, uint32_t id) {
	while(head != NULL) {
		Sensor *sensor = (Sensor*)head->content;

		// If multiple of sampling period was found, update the measurement
		if (!(id % sensor->sampling_period_s)) {
			// Power up sensor
			if (sensor->func_tbl->powerUp)
				sensor->func_tbl->powerUp(sensor);
			// Read Sensor Data
			sensor->func_tbl->read(sensor, sensor->out_data);
			// Power Down sensor
			if (sensor->func_tbl->powerDown)
				sensor->func_tbl->powerDown(sensor);
		}

		head = head->nextElement;
	}
}

void sensor_setSamplingPeriod(Sensor *sensor, uint16_t sampling_period_s) {
	if(sampling_period_s >= 0x7FFF){
		sensor->sampling_period_s = 0xFFFFFFFF; // 136 years in seconds... should be safe.
	} else{
		sensor->sampling_period_s = sampling_period_s;
	}
}

Sensor* sensor_createSensor() {
	return (Sensor*) malloc(sizeof(Sensor));
}
