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

// Variables globales
Node node;

int main(void)
{
	HAL_Init();
	InitClk();

	// Node Initialization
	Node_init(&node, 1);

	if (xTaskCreate(Node_task, "node_task", 1000, (void*)&node, 5, NULL) != pdPASS) {
		while(1);
	}

	vTaskStartScheduler();

	for(;;);
}
