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

extern Sensor_I2C_Probe_Intf *discoverable_devices[];

#endif /* SENSOR_LIST_H_ */
