/*
 * spi_sensor.h
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#ifndef SPI_SENSOR_H_
#define SPI_SENSOR_H_

#include "stm32f4xx.h"

typedef struct {
	uint32_t reg;
	uint32_t value;
	uint32_t data_size;
} Sensor_SPI_Init_Pair;

typedef struct {
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef *cs_port;
	uint16_t cs_pin;
	uint8_t read_addr;
	uint8_t read_size;
	Sensor_SPI_Init_Pair *init_array;
} Sensor_SPI;

#endif /* SPI_SENSOR_H_ */
