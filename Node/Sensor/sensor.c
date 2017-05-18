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

/*void sensor_addDiscoverableSensor(LListElement **head, I2C_HandleTypeDef *hi2c, uint8_t index) {
	Sensor *sensor = (Sensor*)malloc(sizeof(Sensor));
	if (sensor == NULL) return;

	if (discoverable_devices[index]->init(sensor, hi2c) == 0) {
		sensor->func_tbl->init(sensor);

		if (*head == NULL) {
			*head = LList_CreateList((void *)sensor);
		} else {
			LList_AppendElement(*head, (void*) sensor);
		}
	}
}*/

void sensor_addSensor(LListElement **head, I2C_HandleTypeDef *hi2c, Sensor_I2C_Probe_Intf *probe_intf, uint32_t sampling_period_s) {
	Sensor *sensor = (Sensor*)malloc(sizeof(Sensor));
	if (sensor == NULL) return;

	if (probe_intf->init(sensor, hi2c, sampling_period_s) == 0) {
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

void sensor_setSamplingPeriod(Sensor *sensor, uint32_t sampling_period_s) {
	sensor->sampling_period_s = sampling_period_s;
}
