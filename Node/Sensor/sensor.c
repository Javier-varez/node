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

	if (discoverable_devices[index]->init(sensor, hi2c) == 0) {
		sensor->func_tbl->init(sensor);

		if (*head == NULL) {
			*head = LList_CreateList((void *)sensor);
		} else {
			LList_AppendElement(*head, (void*) sensor);
		}
	}
}

void sensor_readSensors(LListElement *head) {
	while(head != NULL) {
		Sensor *sensor = (Sensor*)head->content;
		// Power up sensor
		if (sensor->func_tbl->powerUp)
			sensor->func_tbl->powerUp(sensor);
		// Read Sensor Data
		sensor->func_tbl->read(sensor, sensor->out_data);
		// Power Down sensor
		if (sensor->func_tbl->powerDown)
			sensor->func_tbl->powerDown(sensor);
		head = head->nextElement;
	}
}

void sensor_sendSensorData(LListElement *head, nRF24L01 *device, uint8_t node_id) {

	uint8_t data[32];
	data[0] = node_id;

	uint8_t index = 0;
	while (head != NULL) {
		Sensor *sensor = (Sensor*)head->content;

		data[1] = index;
		sensor->func_tbl->packData(sensor, &data[2]);

		nRF24L01_transmit(device, (uint8_t*)data);

		vTaskDelay(20/portTICK_PERIOD_MS);

		head = head->nextElement;
		index++;
	}
}
