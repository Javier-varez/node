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
#include "node.h"

#include "FreeRTOS.h"
#include "task.h"

// Variables globales
Node node;

int main(void)
{
	HAL_Init();

	// Node Initialization
	Node_init(&node, 1);

	vTaskStartScheduler();

	for(;;);
}
