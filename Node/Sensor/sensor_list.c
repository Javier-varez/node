/*
 * sensor_list.c
 *
 *  Created on: Apr 15, 2017
 *      Author: javier
 */

#include "sensor_list.h"
#include <stdlib.h>

Sensor_I2C_Probe_Intf *discoverable_devices[]= {
	&LSM9DS1_M_intf,
	&LSM9DS1_AG_intf,
	&HTS221_intf,
	&LPS25H_intf,
	NULL
};


void sensor_discoverDevicesOnI2CBus(LListElement **head, I2C_HandleTypeDef *hi2c) {
	Sensor probe_sensor;
	uint8_t i = 0;
	while(discoverable_devices[i] != NULL) {
		if (discoverable_devices[i]->init(&probe_sensor, hi2c) == 0) {
			if (probe_sensor.func_tbl->probe(&probe_sensor) == 0) {
				discoverable_devices[i]->remove(&probe_sensor);
				sensor_addDiscoverableSensor(head, hi2c, i);
			} else {
				discoverable_devices[i]->remove(&probe_sensor);
			}
		}
		i++;
	}
}
