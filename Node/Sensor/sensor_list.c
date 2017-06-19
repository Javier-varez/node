/*
 * sensor_list.c
 *
 *  Created on: Apr 15, 2017
 *      Author: javier
 */

#include "sensor_list.h"
#include <stdlib.h>


Sensor_Probe_Intf *discoverable_devices[]= {
	&LSM9DS1_M_intf,
	&LSM9DS1_AG_intf,
	&HTS221_intf,
	&LPS25H_intf,
	NULL
};


struct_sensor_list available_sensors[] = {
		{
				HTS221_ID,
				SI_I2C,
				&HTS221_intf
		},
		{
				LPS25H_ID,
				SI_I2C,
				&LPS25H_intf
		},
		{
				LSM9DS1_AG_ID,
				SI_I2C,
				&LSM9DS1_AG_intf
		},
		{
				LSM9DS1_M_ID,
				SI_I2C,
				&LSM9DS1_M_intf
		},
		{
				VBAT_ID,
				SI_Analog,
				&VBAT_intf
		},
		{
				LDR_ID,
				SI_Analog,
				&LDR_intf
		},
		{
				0x00,
				SI_Custom,
				NULL
		}
};


void sensor_discoverDevicesOnI2CBus(LListElement **head, I2C_HandleTypeDef *hi2c) {
	Sensor probe_sensor;
	uint8_t i = 0;
	while(discoverable_devices[i] != NULL) {
		if (discoverable_devices[i]->init(&probe_sensor, hi2c, 0) == 0) {
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

uint8_t sensorList_getIndex(uint8_t sensor_ID){
	int index = 0;
	while(available_sensors[index].sensor_ID != 0x00){
		if(available_sensors[index].sensor_ID == sensor_ID)
			return index;
		index++;
	}
	return -1;
}
