/*
 * node.c
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#include "stm32f4xx.h"
#include "node.h"

int Node_init(Node *node, uint32_t id, Sensor *sensor_array, Comms_module *comms_module) {
	if (node != NULL) {
		node->id = id;
		node->sensor_ptr = sensor_array;
		node->comms = comms_module;
		return 0;
	}

	return 1;
}
