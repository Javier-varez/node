/*
 * node.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef NODE_H_
#define NODE_H_

#include "stm32f4xx.h"
#include "node_configuration.h"
#include "sensor.h"
#include "Comms_module.h"
#include "init_periph.h"

typedef struct {
	Node_Configuration configuration;
	uint8_t id;
	LListElement *sensor_list; // Linked list
	Comms_module comms;
	uint8_t queued_packages;
	uint32_t current_sample;
} Node;

// Utility functions
int Node_init(Node *node, uint32_t id);
void Node_task(void *param);
int Node_loadConfiguration(Node* node);
int Node_storeConfiguration(Node* node, uint8_t addr);

#endif /* NODE_H_ */
