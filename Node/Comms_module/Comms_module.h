/*
 * Comms_module.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef COMMS_MODULE_H_
#define COMMS_MODULE_H_

#include "nRF24L01.h"

typedef struct {
	nRF24L01 device;
} Comms_module;

typedef nRF24L01_Mode Comms_module_Mode;

int comms_module_Init(Comms_module *module, Comms_module_Mode mode);

#endif /* COMMS_MODULE_H_ */
