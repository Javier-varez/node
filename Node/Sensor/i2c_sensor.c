/*
 * i2c_sensor.c
 *
 *  Created on: Apr 5, 2017
 *      Author: javier
 */

#include "sensor.h"
#include <string.h>

int i2c_sensor_probe(Sensor *sensor) {
	HAL_StatusTypeDef rc = HAL_OK;

	uint8_t data = 0;

	rc = HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
			  	  	 	  sensor->data.i2c.id_addr, 1,
						  &data, 1, 1000);

	if ((rc == HAL_OK)&&(data == sensor->data.i2c.id))
		return 0;

	return 1;
}

int i2c_sensor_init_regs(Sensor *sensor) {
	uint32_t num = sensor->data.i2c.init->size;
	uint32_t data_size = sensor->data.i2c.init->data_size;
	uint32_t reg_size = sensor->data.i2c.init->reg_size;
	Sensor_I2C_Init_Pair *pair = sensor->data.i2c.init->array;

	HAL_StatusTypeDef rc = HAL_OK;

	for (int i = 0; i < num; i++) {
		rc = HAL_I2C_Mem_Write(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						  	   pair[i].reg, reg_size, (uint8_t*)&pair[i].value,
							   data_size, 1000);
		if (rc != HAL_OK)
			break;
	}

	return rc;
}

int i2c_sensor_read_regs(Sensor *sensor, void *data) {
	HAL_StatusTypeDef rc = HAL_OK;

	rc = HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
			  	  	 	  sensor->data.i2c.read_addr, sensor->data.i2c.read_addr_size,
						  (uint8_t*)data, sensor->data.i2c.read_size, 1000);

	return rc;
}

int i2c_sensor_powerDown(Sensor *sensor) {
	uint32_t num = sensor->data.i2c.powerdown->size;
	uint32_t data_size = sensor->data.i2c.powerdown->data_size;
	uint32_t reg_size = sensor->data.i2c.powerdown->reg_size;
	Sensor_I2C_Init_Pair *pair = sensor->data.i2c.powerdown->array;

	HAL_StatusTypeDef rc = HAL_OK;

	for (int i = 0; i < num; i++) {
		rc = HAL_I2C_Mem_Write(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						  	   pair[i].reg, reg_size, (uint8_t*)&pair[i].value,
							   data_size, 1000);
		if (rc != HAL_OK)
			break;
	}

	return rc;
}

int i2c_sensor_powerUp(Sensor *sensor) {
	uint32_t num = sensor->data.i2c.powerup->size;
	uint32_t data_size = sensor->data.i2c.powerup->data_size;
	uint32_t reg_size = sensor->data.i2c.powerup->reg_size;
	Sensor_I2C_Init_Pair *pair = sensor->data.i2c.powerup->array;

	HAL_StatusTypeDef rc = HAL_OK;

	for (int i = 0; i < num; i++) {
		rc = HAL_I2C_Mem_Write(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						  	   pair[i].reg, reg_size, (uint8_t*)&pair[i].value,
							   data_size, 1000);
		if (rc != HAL_OK)
			break;
	}

	return rc;
}

static Sensor_Func_Table i2c_func_tbl = {
	.probe = i2c_sensor_probe,
	.init = i2c_sensor_init_regs,
	.read = i2c_sensor_read_regs,
	.powerDown = i2c_sensor_powerDown,
	.powerUp = i2c_sensor_powerUp
};

int i2c_sensor_init(Sensor *sensor, char *name, Sensor_Func_Table *func_tbl,
					I2C_HandleTypeDef *hi2c, uint8_t dev_addr, Sensor_I2C_Init* init,
					uint32_t read_addr, uint32_t read_addr_size, uint32_t read_size,
					uint8_t id_addr, uint8_t id) {
	if (sensor != NULL) {
		if (strlen(name) < NAME_MAXLEN-1) strcpy(sensor->name, name);

		sensor->interface = SI_I2C;

		if (func_tbl != NULL) sensor->func_tbl = func_tbl;
		else sensor->func_tbl = &i2c_func_tbl;

		sensor->data.i2c.hi2c = hi2c;
		sensor->data.i2c.dev_addr = dev_addr;
		sensor->data.i2c.init = init;
		sensor->data.i2c.read_addr = read_addr;
		sensor->data.i2c.read_addr_size = read_addr_size;
		sensor->data.i2c.read_size = read_size;
		sensor->data.i2c.id_addr = id_addr;
		sensor->data.i2c.id = id;

		return 0;
	}

	return 1;
}
