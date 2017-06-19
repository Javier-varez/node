/*
 * init_periph.h
 *
 *  Created on: Apr 11, 2017
 *      Author: javier
 */

#ifndef INIT_PERIPH_H_
#define INIT_PERIPH_H_

#include "stm32f4xx.h"
void InitClk();

int i2c_init(I2C_HandleTypeDef *hi2c, DMA_HandleTypeDef *hdma);
int spi_init(SPI_HandleTypeDef *hspi);
int uart_init(UART_HandleTypeDef *huart);
int rtc_init(RTC_HandleTypeDef *rtc);
int rtc_setup_wakeup_interrupt(RTC_HandleTypeDef *rtc, uint32_t period_s);
int gpio_init();
int adc_init(ADC_HandleTypeDef *hadc);

#endif /* INIT_PERIPH_H_ */
