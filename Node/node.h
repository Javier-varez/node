/*
 * node.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef NODE_H_
#define NODE_H_

#include "stm32f4xx.h"
#include "sensor.h"
#include "Comms_module.h"
#include "init_periph.h"

typedef struct {
	uint8_t id;
	LListElement *sensor_list; // Linked list
	Comms_module comms;
} Node;

// Utility functions
int Node_init(Node *node, uint32_t id);
void Node_task(void *param);

#endif /* NODE_H_ */
