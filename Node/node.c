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

#include <string.h>

I2C_HandleTypeDef hi2c;
DMA_HandleTypeDef hdma;
UART_HandleTypeDef huart;

#warning TODO: Remove select mode after testing

uint8_t mode;
char str[150];

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

		rc = uart_init(&huart);
		rc |= i2c_init(&hi2c, &hdma);
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

		int16_t *x = (int16_t*) &buf[2];
		int16_t *y = (int16_t*) &buf[4];
		int16_t *z = (int16_t*) &buf[6];

		int16_t *Ax = (int16_t*) &buf[2];
		int16_t *Ay = (int16_t*) &buf[4];
		int16_t *Az = (int16_t*) &buf[6];
		int16_t *Gx = (int16_t*) &buf[8];
		int16_t *Gy = (int16_t*) &buf[10];
		int16_t *Gz = (int16_t*) &buf[12];

		float *humidity = (float*) &buf[2];
		float *temperature = (float*) &buf[6];

		float *pressure = (float*) &buf[2];

		switch(buf[1]) {
			case 0:
				sprintf(str, "NodeID: %d\r\nMag[x]: %d\r\nMag[y]: %d\r\nMag[z]: %d\r\n", buf[0], *x, *y, *z);
				break;
			case 1:
				sprintf(str, "NodeID: %d\r\nA[x]: %d\r\nA[y]: %d\r\nA[z]: %d\r\nG[x]: %d\r\nG[y]: %d\r\nG[z]: %d\r\n", buf[0], *Ax, *Ay, *Az, *Gx, *Gy, *Gz);
				break;
			case 2:
				sprintf(str, "NodeID: %d\r\nH: %f\r\nT: %f\r\n", buf[0], *humidity, *temperature);
				break;
			case 3:
				sprintf(str, "NodeID: %d\r\nP: %f\r\nT: %f\r\n\r\n\r\n", buf[0], *pressure, *temperature);
				break;
		}

		HAL_UART_Transmit(&huart, (uint8_t*)str, strlen(str), 1000);
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
		sensor_sendSensorData(sensor_list, &node->comms.device, node->id);

		vTaskDelayUntil(&lastWakeTime, 2000/portTICK_PERIOD_MS);
	}
}
