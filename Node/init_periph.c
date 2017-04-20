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

#warning TODO: Change i2c interrupt priorities to support FreeRTOS

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

	// Enable I2C interrupts
	HAL_NVIC_SetPriority(I2C1_EV_IRQn, 0x00, 0x00);
	HAL_NVIC_EnableIRQ(I2C1_EV_IRQn);
	HAL_NVIC_SetPriority(I2C1_ER_IRQn, 0x00, 0x00);
	HAL_NVIC_EnableIRQ(I2C1_ER_IRQn);

	// GPIO port B
	// SDA => PB7
	// SCL => PB6
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef gpio;
	gpio.Alternate = GPIO_AF4_I2C1;
	gpio.Mode = GPIO_MODE_AF_OD;
	gpio.Pull = GPIO_PULLUP;
	gpio.Pin = GPIO_PIN_6 | GPIO_PIN_7;
	gpio.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOB, &gpio);

	// Use DMA1, stream 1 to send data from memory to i2c1 peripheral
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

	rc |= HAL_DMA_Init(hdma);

	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, 0x00, 0x00);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	// Link dma to i2c peripheral
	hi2c->hdmatx = hdma;
	hdma->Parent = hi2c;

	// Turn off the display on the sense hat
	uint8_t data = 0x00;
	if ((rc |= HAL_I2C_Mem_Write_DMA(hi2c, 0x46<<1, 0x00, 1, &data, 192)) == HAL_OK) {
		while(done == 0);
	}

	return rc;
}

int spi_init(SPI_HandleTypeDef *hspi) {
	uint8_t rc = 0;

	__HAL_RCC_SPI1_CLK_ENABLE();
	hspi->Instance = SPI1;
	hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64; // 1.5 MHz @ 96 MHz
	hspi->Init.CLKPhase = SPI_PHASE_1EDGE;
	hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
	hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
	hspi->Init.DataSize = SPI_DATASIZE_8BIT;
	hspi->Init.Direction = SPI_DIRECTION_2LINES;
	hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
	hspi->Init.Mode = SPI_MODE_MASTER;
	hspi->Init.NSS = SPI_NSS_HARD_OUTPUT;
	hspi->Init.TIMode = SPI_TIMODE_DISABLE;

	rc |= HAL_SPI_Init(hspi);

	// GPIO port B
	// SCK  => PB3
	// MISO => PB4
	// MOSI => PB5
	// NSS  => PA4
	__HAL_RCC_GPIOB_CLK_ENABLE();
	GPIO_InitTypeDef gpio;
	gpio.Alternate = GPIO_AF5_SPI1;
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Pin = GPIO_PIN_3 | GPIO_PIN_4 |GPIO_PIN_5;
	gpio.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOB, &gpio);

	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Pin = GPIO_PIN_4;
	gpio.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOA, &gpio);

	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);

	return rc;
}

int uart_init(UART_HandleTypeDef *huart) {
	uint8_t rc = 0;
	__HAL_RCC_USART1_CLK_ENABLE();

	huart->Init.BaudRate = 115200;
	huart->Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart->Init.Mode = UART_MODE_TX_RX;
	huart->Init.OverSampling = UART_OVERSAMPLING_16;
	huart->Init.Parity = UART_PARITY_NONE;
	huart->Init.StopBits = UART_STOPBITS_1;
	huart->Init.WordLength = UART_WORDLENGTH_8B;
	huart->Instance = USART1;
	rc = HAL_UART_Init(huart);

	GPIO_InitTypeDef gpio;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Mode = GPIO_MODE_AF_PP;
	gpio.Alternate = GPIO_AF7_USART1;
	gpio.Pin = GPIO_PIN_10 | GPIO_PIN_15;
	gpio.Pull = GPIO_NOPULL;
	gpio.Speed = GPIO_SPEED_HIGH;

	HAL_GPIO_Init(GPIOA, &gpio);

	return rc;
}

int gpio_init() {

	GPIO_InitTypeDef gpio;

	// CE PIN (nRF24L01)
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Mode = GPIO_MODE_OUTPUT_PP;
	gpio.Pull = GPIO_NOPULL;
	gpio.Pin = GPIO_PIN_5;
	gpio.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOA, &gpio);

	// INT PIN (nRF24L01)
	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Mode = GPIO_MODE_IT_FALLING;
	gpio.Pull = GPIO_NOPULL;
	gpio.Pin = GPIO_PIN_3;
	gpio.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &gpio);

	return 0;
}

