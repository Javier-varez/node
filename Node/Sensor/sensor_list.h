/*
 * sensor_list.h
 *
 *  Created on: Apr 15, 2017
 *      Author: javier
 */

#ifndef SENSOR_LIST_H_
#define SENSOR_LIST_H_

#include <stdint.h>

struct sensor_struct;

typedef enum {
	SI_Analog,
	SI_Digital,
	SI_I2C,
	SI_Custom,
} Sensor_Interface;

typedef struct {
	int (*init)(struct sensor_struct *, void *, uint16_t);
	int (*remove)(struct sensor_struct *);
} Sensor_Probe_Intf;

typedef struct {
	uint8_t sensor_ID;
	Sensor_Interface interface;
	Sensor_Probe_Intf *probe_intf;
} struct_sensor_list;

#include "HTS221.h"
#include "LPS25H.h"
#include "LSM9DS1_AG.h"
#include "LSM9DS1_M.h"
#include "LDR.h"
#include "VBAT.h"

#include "LList.h"
#define NUM_SENSORS 4

extern Sensor_Probe_Intf *discoverable_devices[];



extern struct_sensor_list available_sensors[];

uint8_t sensorList_getIndex(uint8_t sensor_ID);
void sensor_discoverDevicesOnI2CBus(LListElement **head, I2C_HandleTypeDef *hi2c);
#endif /* SENSOR_LIST_H_ */
