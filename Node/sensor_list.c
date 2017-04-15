/*
 * sensor_list.c
 *
 *  Created on: Apr 15, 2017
 *      Author: javier
 */

#include "sensor_list.h"

Sensor_I2C_Probe_Intf *discoverable_devices[]= {
	&LSM9DS1_M_intf,
	&LSM9DS1_AG_intf,
	&HTS221_intf,
	&LPS25H_intf,
	NULL
};
