/*
 * init_periph.h
 *
 *  Created on: Apr 11, 2017
 *      Author: javier
 */

#ifndef INIT_PERIPH_H_
#define INIT_PERIPH_H_

#include "stm32f4xx.h"

int i2c_init(I2C_HandleTypeDef *hi2c, DMA_HandleTypeDef *hdma);

#endif /* INIT_PERIPH_H_ */
