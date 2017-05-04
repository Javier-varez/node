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
RTC_HandleTypeDef hrtc;

#define MIN_WAKEUP_PERIOD_S		2
#define PAYLOAD_QUEUE_MAX_LEN	10
#define ACK_QUEUE_MAX_LEN		5

void Node_error_handler() {
	while(1);
}

// Entry point for the whole system
int Node_init(Node *node, uint32_t id) {
	if (node != NULL) {

		InitClk();

		uint8_t rc = 0;
		node->id = id;
		node->queued_packages = 0;
		node->comms.sent_packages = 0;

		// Init peripherals
		rc |= i2c_init(&hi2c, &hdma);
		rc |= rtc_init(&hrtc);

		// Discover Sensors
		sensor_discoverDevicesOnI2CBus(&node->sensor_list, &hi2c);

		// Wake microcontroller each 10 seconds
		rtc_setup_wakeup_interrupt(&hrtc, MIN_WAKEUP_PERIOD_S);

		// Create Queues
		node->comms.payloadQueue = xQueueCreate(PAYLOAD_QUEUE_MAX_LEN, sizeof(Comms_Payload));
		node->comms.ACKQueue = xQueueCreate(ACK_QUEUE_MAX_LEN, sizeof(Comms_Payload));

		if ((node->comms.payloadQueue == NULL) || (node->comms.ACKQueue == NULL))
			Node_error_handler();

		// Create Node task
		if (xTaskCreate(Node_task, "node_task", 1000, (void*)node, 4, NULL) != pdPASS) {
			Node_error_handler();
		}

		// Create Comms task
		if (xTaskCreate(comms_task, "comms_task", 1000, (void*)&node->comms, 5, NULL) != pdPASS) {
			Node_error_handler();
		}

		return rc;
	}

	return 1;
}


void Node_WFI(Node *node) {
	// wait until all packages have been sent
	while(node->queued_packages > node->comms.sent_packages);

	// Disable SO interrupts
	taskENTER_CRITICAL();
	// Enter STOP mode
	HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
	// Reconfigure Clk
	InitClk();
	// Enable SO Interrupts
	taskEXIT_CRITICAL();

	// Reset Queue and Sent
	node->queued_packages = 0;
	node->comms.sent_packages = 0;
}

void Node_sendSensorData(Node *node, LListElement *head) {

	Comms_Payload payload;

	while (head != NULL) {
		Sensor *sensor = (Sensor*)head->content;

		// Insert data into package
		if (sensor->func_tbl->packData(sensor, &payload.data[0], 30)) {
			// Add data to transmission queue, wait until ready to receive
			if (xQueueSendToBack(node->comms.payloadQueue, (void *)&payload, portMAX_DELAY) == pdPASS)
				node->queued_packages++;
		}

		head = head->nextElement;
	}
}

void Node_handleACKPayload(Node *node) {
#warning do things with ACK payload!
	Comms_Payload ACK_Payload;

	// Don't wait forever here
	while (xQueueReceive(node->comms.ACKQueue, (void *) &ACK_Payload, 0) == pdPASS) {
		// Remember to increase node->queued_packages
		if (ACK_Payload.data[0] == 'C' &&
		    ACK_Payload.data[1] == 'F' &&
		    ACK_Payload.data[2] == 'G' &&
			ACK_Payload.data[3] == 'A') {
				uint8_t data = 0x00;
				if (ACK_Payload.data[4] == '1') {
					data = 0x45;
				} else if (ACK_Payload.data[4] == '0') {
					data = 0x05;
				}
				HAL_I2C_Mem_Write(&hi2c, 0x46 << 1, 0xf4, 1, &data, 1, 1000);
		}
	}
}

void Node_task(void *param) {
	Node *node = (Node*) param;
	LListElement *sensor_list = node->sensor_list;

	//Node_Receiver_test();

	// Init Comms module
	comms_module_Init(&node->comms, TRANSMITTER);

	while(1) {
		sensor_readSensors(sensor_list);
		Node_sendSensorData(node, sensor_list);
		Node_handleACKPayload(node);

		Node_WFI(node);
	}
}


char str[320];
uint8_t alarm_on = 0;
void Node_Receiver_test() {
	uint8_t rc = 0;
	Comms_module comms;
	Comms_Payload Payload;

	Comms_Payload ACK_Payload;

	InitClk();

	rc = uart_init(&huart);
	rc |= comms_module_Init(&comms, RECEIVER);

	while(1) {

		// Receive data
		nRF24L01_setMode(&comms.device, RECEIVER);
		if (nRF24L01_pollForRXPacketWithTimeout(&comms.device, 100)) {
			nRF24L01_readPayload(&comms.device, (uint8_t*)&Payload, sizeof(Payload));

			ACK_Payload.PID = Payload.PID;
			ACK_Payload.address = Payload.address;

			ACK_Payload.data[0] = 'A';
			ACK_Payload.data[1] = 'C';
			ACK_Payload.data[2] = 'K';
			ACK_Payload.data[3] = '\0';

			if (Payload.data[0] == 'A') {
				int16_t *z = (int16_t*)(&Payload.data[5]);
				if (*z > 15000) {
					alarm_on = 1;
					ACK_Payload.data[0] = 'C';
					ACK_Payload.data[1] = 'F';
					ACK_Payload.data[2] = 'G';
					ACK_Payload.data[3] = 'A';
					ACK_Payload.data[4] = '1';
					ACK_Payload.data[5] = '\0';
				} else if (alarm_on) {
					alarm_on = 0;
					ACK_Payload.data[0] = 'C';
					ACK_Payload.data[1] = 'F';
					ACK_Payload.data[2] = 'G';
					ACK_Payload.data[3] = 'A';
					ACK_Payload.data[4] = '0';
					ACK_Payload.data[5] = '\0';
				}
			}

			nRF24L01_setMode(&comms.device, TRANSMITTER);
			nRF24L01_transmit(&comms.device, (uint8_t*)&ACK_Payload);
			nRF24L01_pollForTXPacket(&comms.device);

			// Print Package
			uint8_t *data = Payload.data;
			while(data[0] != '\0') {
				if (data[0] == 'M') {
					sprintf(str, "Received Magnetometer\r\n");
					data += 7;
				} else if (data[0] == 'A') {
					sprintf(str, "Received Acc\r\n");
					data += 7;
				} else if (data[0] == 'G') {
					sprintf(str, "Received Gyro\r\n");
					data += 7;
				} else if (data[0] == 'P') {
					sprintf(str, "Pres:\t%f\r\n", *((float*)&data[1]));
					data += 5;
				} else if (data[0] == 'H') {
					sprintf(str, "Humi:\t%f\r\n", *((float*)&data[1]));
					data += 5;
				} else if (data[0] == 'T') {
					sprintf(str, "Temp:\t%f\r\n", *((float*)&data[1]));
					data += 5;
				} else {
					break;
				}

				HAL_UART_Transmit(&huart, (uint8_t*)str, strlen(str), 1000);
			}

		}

	}
}

