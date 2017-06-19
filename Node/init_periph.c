/*
 * init_periph.c
 *
 *  Created on: Apr 11, 2017
 *      Author: javier
 */


#include "init_periph.h"
#include "FreeRTOS.h"

volatile int done = 0;

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
	done = 1;
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
	done = 1;
}


void InitClk() {
	RCC_ClkInitTypeDef RCC_ClkInitStruct;
	RCC_OscInitTypeDef RCC_OscInitStruct;

	__HAL_RCC_PWR_CLK_ENABLE();
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);

	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.PLL.PLLN = 192;
	RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
	RCC_OscInitStruct.PLL.PLLM = 4;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLQ = 8;
	HAL_RCC_OscConfig(&RCC_OscInitStruct);

	RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_HCLK |
								   RCC_CLOCKTYPE_PCLK1 |
								   RCC_CLOCKTYPE_PCLK2 |
								   RCC_CLOCKTYPE_SYSCLK);
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

	HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2);
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

	HAL_StatusTypeDef rc = HAL_I2C_Init(hi2c);

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

	HAL_NVIC_SetPriority(DMA1_Stream1_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0x00);
	HAL_NVIC_EnableIRQ(DMA1_Stream1_IRQn);

	// Link dma to i2c peripheral
	hi2c->hdmatx = hdma;
	hdma->Parent = hi2c;

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

	HAL_NVIC_SetPriority(EXTI3_IRQn, configLIBRARY_LOWEST_INTERRUPT_PRIORITY, 0x00);
	HAL_NVIC_EnableIRQ(EXTI3_IRQn);

	return 0;
}

int rtc_init(RTC_HandleTypeDef *rtc) {
	__HAL_RCC_PWR_CLK_ENABLE();

	HAL_PWR_EnableBkUpAccess();

	rtc->Instance = RTC;
	rtc->Init.HourFormat = RTC_HOURFORMAT_24;
	rtc->Init.AsynchPrediv = 128;
	rtc->Init.SynchPrediv = 256; // 128*256 = 32768 => 32768 Hz / (128*256) = 1Hz
	rtc->Init.HourFormat = RTC_HOURFORMAT_24;
	rtc->Init.OutPut = RTC_OUTPUT_DISABLE;
	rtc->Init.OutPutType = RTC_OUTPUT_TYPE_PUSHPULL;
	rtc->Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;

	// Configure LSI Clk
	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_PeriphCLKInitTypeDef PeriphClkInitStruct;

	PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_RTC;

	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI;
	RCC_OscInitStruct.LSIState = RCC_LSI_ON;

	PeriphClkInitStruct.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;

	HAL_RCC_OscConfig(&RCC_OscInitStruct);
	HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct);

	__HAL_RCC_RTC_ENABLE();

	//Set time and date
	RTC_DateTypeDef RTC_DateStruct;
	RTC_TimeTypeDef RTC_TimeStruct;

	RTC_DateStruct.Year = 0;
	RTC_DateStruct.Month = 1;
	RTC_DateStruct.Date = 1;
	RTC_DateStruct.WeekDay = RTC_WEEKDAY_TUESDAY;
	HAL_RTC_SetDate(rtc, &RTC_DateStruct, RTC_FORMAT_BIN);

	RTC_TimeStruct.Hours = 0x00;
	RTC_TimeStruct.Minutes = 0x00;
	RTC_TimeStruct.Seconds = 0x00;
	RTC_TimeStruct.TimeFormat = RTC_HOURFORMAT_24;
	RTC_TimeStruct.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
	RTC_TimeStruct.StoreOperation = RTC_STOREOPERATION_RESET;
	HAL_RTC_SetTime(rtc, &RTC_TimeStruct, RTC_FORMAT_BCD);

	HAL_RTC_Init(rtc);

	return 0;
}

int rtc_setup_wakeup_interrupt(RTC_HandleTypeDef *rtc, uint32_t period_s) {
	/* Disable wakeup interrupt */
	__HAL_RTC_WAKEUPTIMER_DISABLE(rtc);

	/* Disable RTC interrupt flag */
	__HAL_RTC_WAKEUPTIMER_DISABLE_IT(rtc, RTC_IT_WUT);

	/* Clear pending bit */
	__HAL_RTC_WAKEUPTIMER_EXTI_CLEAR_FLAG();

	/* Clear flag */
	__HAL_RTC_WAKEUPTIMER_CLEAR_FLAG(rtc, RTC_FLAG_WUTF);

	uint32_t val = period_s * (32768 / 16);
	HAL_RTCEx_SetWakeUpTimer_IT(rtc, val, RTC_WAKEUPCLOCK_RTCCLK_DIV16);

	HAL_NVIC_SetPriority(RTC_WKUP_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(RTC_WKUP_IRQn);

	return 0;
}

int adc_init(ADC_HandleTypeDef *hadc){

	GPIO_InitTypeDef gpioInit;
	__GPIOC_CLK_ENABLE();
	__ADC1_CLK_ENABLE();

	gpioInit.Pin = GPIO_PIN_1 | GPIO_PIN_2;
	gpioInit.Mode = GPIO_MODE_ANALOG;
	gpioInit.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(GPIOC, &gpioInit);

	HAL_NVIC_SetPriority(ADC_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(ADC_IRQn);


	hadc->Instance = ADC1;

	hadc->Init.ClockPrescaler = ADC_CLOCKPRESCALER_PCLK_DIV2;
	hadc->Init.Resolution = ADC_RESOLUTION_12B; //resolution: 12 bits (0 to 4095)
	hadc->Init.ScanConvMode = DISABLE;
	hadc->Init.ContinuousConvMode = ENABLE;
	hadc->Init.DiscontinuousConvMode = DISABLE;
	hadc->Init.NbrOfDiscConversion = 0;
	hadc->Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc->Init.ExternalTrigConv = ADC_EXTERNALTRIGCONV_T1_CC1;
	hadc->Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc->Init.NbrOfConversion = 1;
	hadc->Init.DMAContinuousRequests = ENABLE;
	hadc->Init.EOCSelection = DISABLE;

	HAL_ADC_Init(hadc);

	return 0;
}
