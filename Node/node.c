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

#define MIN_WAKEUP_PERIOD_S		1
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
		node->current_sample = 0;

		// Init peripherals
		rc |= i2c_init(&hi2c, &hdma);
		rc |= rtc_init(&hrtc);

		// Discover Sensors
		sensor_discoverDevicesOnI2CBus(&node->sensor_list, &hi2c);

#warning test load for MP45DT02 Microphone
		Sensor *sensor = (Sensor*)malloc(sizeof(Sensor));
		if (sensor != NULL) {
			MP45DT02_init(sensor);
			sensor->func_tbl->init(sensor);
			if (node->sensor_list == NULL) {
				node->sensor_list = LList_CreateList((void *)sensor);
			} else {
				LList_AppendElement(node->sensor_list, (void*) sensor);
			}
		}

		// Wake microcontroller
		rtc_setup_wakeup_interrupt(&hrtc, MIN_WAKEUP_PERIOD_S);

		// Restore configuration
		//if (Node_loadConfiguration(node)) {
			// If it fails, then apply std configuration
		//	node->configuration.node_id = 0x00; // Standard node id informs that it lacks configuration
		//}
		//node->id = node->configuration.node_id;

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

		if (!(node->current_sample % sensor->sampling_period_s)) {
			// Insert data into package
			if (sensor->func_tbl->packData(sensor, &payload.data[0], 30)) {
				// Add data to transmission queue, wait until ready to receive
				if (xQueueSendToBack(node->comms.payloadQueue, (void *)&payload, portMAX_DELAY) == pdPASS)
					node->queued_packages++;
			}
		}

		head = head->nextElement;
	}
}

void Node_handleACKPayload(Node *node) {
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

	// Init Comms module
	comms_module_Init(&node->comms, TRANSMITTER, node->id);

	while(1) {
		sensor_readSensors(sensor_list, node->current_sample);
		Node_sendSensorData(node, sensor_list);
		Node_handleACKPayload(node);

		Node_WFI(node);
		node->current_sample++; // Increment current sample
	}
}

