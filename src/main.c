/**
  ******************************************************************************
  * @file    main.c
  * @author  Ac6
  * @version V1.0
  * @date    01-December-2013
  * @brief   Default main function.
  ******************************************************************************
*/


#include "stm32f4xx.h"
#include "init_periph.h"
#include "node.h"
#include "nRF24L01.h"

#include "FreeRTOS.h"
#include "task.h"

void InitClk();

// Variables globales

I2C_HandleTypeDef hi2c;
DMA_HandleTypeDef hdma;
SPI_HandleTypeDef hspi;
LListElement *sensor_list = NULL;
nRF24L01 module;

int main(void)
{
	HAL_Init();
	InitClk();

	// Configure peripherals
	i2c_init(&hi2c, &hdma);
	spi_init(&hspi);
	gpio_init();

	module.configuration.addr[0] = 0xE7;
	module.configuration.addr[1] = 0xE7;
	module.configuration.addr[2] = 0xE7;
	module.configuration.addr[3] = 0xE7;
	module.configuration.addr[4] = 0xE7;
	module.configuration.channel = 24; // Test channel
	module.configuration.mode = TRANSMITTER;
	module.configuration.output_power = m0dBm;
	module.hspi = &hspi;
	module.CE.pin = GPIO_PIN_5;
	module.CE.port = GPIOA;
	module.IRQ.pin = GPIO_PIN_3;
	module.IRQ.port = GPIOA;
	module.CSN.pin = GPIO_PIN_4;
	module.CSN.port = GPIOA;

	nRF24L01_init(&module);

	uint8_t data[32];
	nRF24L01_transmit(&module, data);

	sensor_discoverDevicesOnI2CBus(&sensor_list, &hi2c);

	if (xTaskCreate(Node_task, "node_task", 1000, (void*)sensor_list, 5, NULL) != pdPASS) {
		while(1);
	}

	vTaskStartScheduler();

	for(;;);
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
