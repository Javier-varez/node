/*
 * i2c_sensor.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef I2C_SENSOR_H_
#define I2C_SENSOR_H_

#include "sensor.h"
#include "stm32f4xx.h"

typedef struct {
	uint32_t reg;
	uint32_t value;
} Sensor_I2C_Init_Pair;

typedef struct {
	Sensor_I2C_Init_Pair *array;
	uint32_t data_size;
	uint32_t reg_size;
	uint32_t size;
} Sensor_I2C_Init;

typedef struct {
	Sensor_I2C_Init_Pair *array;
	uint32_t data_size;
	uint32_t reg_size;
	uint32_t size;
} Sensor_I2C_Power;

typedef struct {
	I2C_HandleTypeDef *hi2c;
	uint8_t dev_addr;
	uint32_t read_addr;
	uint8_t read_addr_size;
	uint8_t read_size;
	Sensor_I2C_Init *init;
	Sensor_I2C_Power *powerup;
	Sensor_I2C_Power *powerdown;
	uint8_t id_addr;
	uint8_t id;
} Sensor_I2C;

typedef struct sensor_struct Sensor;
typedef struct sensor_fuct_tbl Sensor_Func_Table;

int i2c_sensor_init(Sensor *sensor, char *name, uint8_t sensorID, Sensor_Func_Table *func_tbl,
					I2C_HandleTypeDef *hi2c, uint8_t dev_addr, Sensor_I2C_Init* init,
					uint32_t read_addr, uint32_t read_addr_size, uint32_t read_size,
					uint8_t id_addr, uint8_t id, uint32_t sampling_period_s);

int i2c_sensor_probe(Sensor *sensor);
int i2c_sensor_init_regs(Sensor *sensor);
int i2c_sensor_read_regs(Sensor *sensor, void *data);
int i2c_sensor_powerDown(Sensor *sensor);
int i2c_sensor_powerUp(Sensor *sensor);
#endif /* I2C_SENSOR_H_ */
