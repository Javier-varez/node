/*
 * node.c
 *
 *  Created on: Apr 4, 2017
 *      Author: javier
 */

#include "stm32f4xx.h"
#include "node.h"

#include "FreeRTOS.h"
#include "task.h"

I2C_HandleTypeDef hi2c;
DMA_HandleTypeDef hdma;


#warning TODO: Remove select mode after testing

uint8_t mode;

void select_mode() {

	GPIO_InitTypeDef gpio;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	gpio.Mode = GPIO_MODE_INPUT;
	gpio.Pull = GPIO_NOPULL;
	gpio.Pin = GPIO_PIN_0;
	gpio.Speed = GPIO_SPEED_FAST;
	HAL_GPIO_Init(GPIOA, &gpio);

	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0) == 1)
		mode = RECEIVER;
	else
		mode = TRANSMITTER;
}

int Node_init(Node *node, uint32_t id) {
	if (node != NULL) {
		uint8_t rc = 0;

		node->id = id;

		rc = i2c_init(&hi2c, &hdma);
		sensor_discoverDevicesOnI2CBus(&node->sensor_list, &hi2c);

		select_mode();
		rc |= comms_module_Init(&node->comms, mode);

		return rc;
	}

	return 1;
}

void receive(Node *node) {

	char buf[32];

	while(1) {
		nRF24L01_pollForRXPacket(&node->comms.device);
		nRF24L01_readPayload(&node->comms.device, (uint8_t*)buf, 32);
	}
}

void Node_task(void *param) {
	Node *node = (Node*) param;
	LListElement *sensor_list = node->sensor_list;


	if (mode == RECEIVER)
		receive(node);

	TickType_t lastWakeTime = xTaskGetTickCount();
	while(1) {
		sensor_readSensors(sensor_list);

		char data[32] = "Transmitiendo!";
		nRF24L01_transmit(&node->comms.device, (uint8_t*)data);
		uint8_t status;
		nRF24L01_readRegister(&node->comms.device, STATUS, &status, 1);

		vTaskDelayUntil(&lastWakeTime, 2000/portTICK_PERIOD_MS);
	}
}
