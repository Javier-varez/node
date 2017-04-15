/*
 * init_periph.c
 *
 *  Created on: Apr 11, 2017
 *      Author: javier
 */


#include "init_periph.h"

volatile int done = 0;

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	done = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	done = 1;
}

int i2c_init(I2C_HandleTypeDef *hi2c, DMA_HandleTypeDef *hdma) {
	__HAL_RCC_I2C1_CLK_ENABLE();
	hi2c->Instance = I2C1;
	hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
	hi2c->Init.ClockSpeed = 100000;
	hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
	hi2c->Init.DutyCycle = I2C_DUTYCYCLE_2;
	hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
	hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

	uint32_t rc = HAL_I2C_Init(hi2c);

	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x00, 0x00);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0x00, 0x00);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

	__HAL_RCC_GPIOB_CLK_ENABLE();

	GPIO_InitTypeDef gpio;
	gpio.Alternate = GPIO_AF4_I2C1;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_PULLUP;
	gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOB, &gpio);

	__HAL_RCC_DMA1_CLK_ENABLE();

	hdma->Instance = DMA1_Stream1;
	hdma->Init.Channel = DMA_CHANNEL_0;
	hdma->Init.Direction = DMA_MEMORY_TO_PERIPH;
	hdma->Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	hdma->Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma->Init.MemInc = DMA_MINC_DISABLE;
	hdma->Init.Mode = DMA_NORMAL;
	hdma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma->Init.Priority = DMA_PRIORITY_HIGH;
	hdma->Init.MemBurst = DMA_MBURST_SINGLE;
	hdma->Init.PeriphBurst = DMA_PBURST_SINGLE;

	HAL_DMA_Init(hdma);

	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0x00, 0x00);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	hi2c->hdmatx = hdma;
	hdma->Parent = hi2c;

	uint8_t data = 0x00;

	// Turn off the display on the sense hat
	if ((rc |= HAL_I2C_Mem_Write_DMA(hi2c, 0x46<<1, 0x00, 1, &data, 192)) == HAL_OK) {
		while(done == 0);
	}

	return rc;
}


