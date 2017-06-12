/*
 * sensor_list.h
 *
 *  Created on: Apr 15, 2017
 *      Author: javier
 */

#ifndef SENSOR_LIST_H_
#define SENSOR_LIST_H_

#include "HTS221.h"
#include "LPS25H.h"
#include "LSM9DS1_AG.h"
#include "LSM9DS1_M.h"

#include "LList.h"
#define NUM_SENSORS 4

extern Sensor_I2C_Probe_Intf *discoverable_devices[];

typedef struct {
	uint8_t sensor_ID;
	Sensor_I2C_Probe_Intf *probe_intf;
} struct_sensor_list;

uint8_t sensorList_getIndex(uint8_t sensor_ID);
void sensor_discoverDevicesOnI2CBus(LListElement **head, I2C_HandleTypeDef *hi2c);
#endif /* SENSOR_LIST_H_ */
