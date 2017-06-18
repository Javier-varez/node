/*
 * node_configuration.h
 *
 *  Created on: May 14, 2017
 *      Author: javier
 */

#ifndef CONFIGURATION_NODE_CONFIGURATION_H_
#define CONFIGURATION_NODE_CONFIGURATION_H_

#include "eeprom.h"
#define SENSOR_NULL 0x00
typedef enum {
	C_NODE_ID = 0x16,
	C_S0_ID,
	C_S0_P,
	C_S1_ID,
	C_S1_P,
	C_S2_ID,
	C_S2_P,
	C_S3_ID,
	C_S3_P,
	C_S4_ID,
	C_S4_P,
	C_S5_ID,
	C_S5_P,
	C_S6_ID,
	C_S6_P,
	C_S7_ID,
	C_S7_P,
	C_S8_ID,
	C_S8_P,
	C_S9_ID,
	C_S9_P,
	C_S10_ID,
	C_S10_P,
	C_S11_ID,
	C_S11_P,
	C_MAX_DATA_LEN
} Config_addr;

#define MAX_CONF_SENSORS ((C_MAX_DATA_LEN - C_NODE_ID - 1)/2)

typedef struct {
	uint16_t Sensor_ID;
	uint16_t Sensor_period;
} Sensor_Configuration;

typedef struct {
	uint16_t node_id;
	Sensor_Configuration sensor_config[(C_MAX_DATA_LEN-C_NODE_ID-1)/2];
} Node_Configuration;

#endif /* CONFIGURATION_NODE_CONFIGURATION_H_ */
