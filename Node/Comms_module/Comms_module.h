/*
 * Comms_module.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef COMMS_MODULE_H_
#define COMMS_MODULE_H_

#include "nRF24L01.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

typedef struct {
	nRF24L01 device;
	xQueueHandle payloadQueue;
	xQueueHandle ACKQueue;
	uint8_t address;
	uint8_t PID;
	uint8_t sent_packages;
} Comms_module;

typedef struct {
	uint8_t address;
	uint8_t PID;
	uint8_t data[30];
} Comms_Payload;

typedef nRF24L01_Mode Comms_module_Mode;

int comms_module_Init(Comms_module *module, Comms_module_Mode mode);
void comms_task(void *param);


#endif /* COMMS_MODULE_H_ */
