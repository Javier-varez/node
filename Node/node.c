/*
 * node.c
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#include "stm32f4xx.h"
#include "node.h"

#include "FreeRTOS.h"
#include "task.h"

int Node_init(Node *node, uint32_t id, Sensor *sensor_array, Comms_module *comms_module) {
	if (node != NULL) {
		node->id = id;
		node->sensor_ptr = sensor_array;
		node->comms = comms_module;
		return 0;
	}

	return 1;
}


void Node_task(void *param) {
	LListElement *sensor_list = (LListElement*) param;

	TickType_t lastWakeTime = xTaskGetTickCount();
	while(1) {
		sensor_readSensors(sensor_list);

		vTaskDelayUntil(&lastWakeTime, 2000/portTICK_PERIOD_MS);
	}
}
