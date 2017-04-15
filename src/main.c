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
#include <stdlib.h>
#include "LList.h"

#include "node.h"
#include "sensor_list.h"

void InitClk();

I2C_HandleTypeDef hi2c;
DMA_HandleTypeDef hdma;

void addSensor(LListElement **head, uint8_t index) {
	Sensor *sensor = (Sensor*)malloc(sizeof(Sensor));
	if (sensor == NULL) return;

	if (discoverable_devices[index]->init(sensor, &hi2c) == 0) {
		sensor->func_tbl->init(sensor);

		if (*head == NULL) {
			*head = LList_CreateList((void *)sensor);
		} else {
			LList_AppendElement(*head, (void*) sensor);
		}
	}
}

void readSensors(LListElement *head) {
	while(head != NULL) {
		Sensor *sensor = (Sensor*)head->content;
		sensor->func_tbl->read(sensor, sensor->out_data);
		head = head->nextElement;
	}
}

int main(void)
{
	HAL_Init();
	InitClk();

	i2c_init(&hi2c, &hdma);

	LListElement *head = NULL;

	Sensor probe_sensor;
	uint8_t i = 0;
	while(discoverable_devices[i] != NULL) {
		if (discoverable_devices[i]->init(&probe_sensor, &hi2c) == 0) {
			if (probe_sensor.func_tbl->probe(&probe_sensor) == 0) {
				discoverable_devices[i]->remove(&probe_sensor);
				addSensor(&head, i);
			} else {
				discoverable_devices[i]->remove(&probe_sensor);
			}
		}
		i++;
	}

	for(;;) {
		readSensors(head);
		HAL_Delay(2000);
	}
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
