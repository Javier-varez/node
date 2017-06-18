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
extern struct_sensor_list available_sensors[];

#define MIN_WAKEUP_PERIOD_S		1
#define PAYLOAD_QUEUE_MAX_LEN	10
#define ACK_QUEUE_MAX_LEN		5

void* LList_GetElementById(LListElement *head, uint8_t sensorID){
	if(head == NULL) return NULL;
	while(head){
		if(((Sensor *)head->content)->sensorID == sensorID) return head->content;
		else head = head->nextElement;
	}
	return NULL;
}

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
		node->alarm = 0;

		// Init peripherals
		rc |= i2c_init(&hi2c, &hdma);
		rc |= rtc_init(&hrtc);
		Sensor* sensor;

		// Discover Sensors
		sensor_discoverDevicesOnI2CBus(&node->sensor_list, &hi2c);

		// Wake microcontroller
		rtc_setup_wakeup_interrupt(&hrtc, MIN_WAKEUP_PERIOD_S);

		// Restore configuration
		if (Node_loadConfiguration(node)) {
			// If it fails, then apply std configuration
			node->configuration.node_id = 0x00; // Standard node id informs that it lacks configuration
			node->id = 0x00;
		} else {
			node->id = node->configuration.node_id;
			int index = 0;
			for(int i = 0; i < MAX_CONF_SENSORS; i++){
				if(node->configuration.sensor_config[i].Sensor_ID != SENSOR_NULL){
					sensor = (Sensor*)LList_GetElementById(node->sensor_list, node->configuration.sensor_config[i].Sensor_ID);

					if(sensor == NULL){
						//If sensor is not loaded automatically (not discoverable) load it
						index = sensorList_getIndex(node->configuration.sensor_config[i].Sensor_ID);
						//If sensor is available, load it:
						if(index >= 0){
							sensor_addSensor(&node->sensor_list,&hi2c, available_sensors[index].probe_intf, node->configuration.sensor_config[i].Sensor_period);
						}
					}

					sensor_setSamplingPeriod(sensor, node->configuration.sensor_config[i].Sensor_period);
				}
			}
		}

		// Once we are done loading from flash, we need to store configuration details for
		// any sensors that are not on the configuration list

		LListElement *element = node->sensor_list;
		while(element) {
			sensor = (Sensor*) element->content;
			for (int i = 0; i < MAX_CONF_SENSORS; i++) {
				if (node->configuration.sensor_config[i].Sensor_ID == sensor->sensorID) break;
				else if (node->configuration.sensor_config[i].Sensor_ID == SENSOR_NULL) {
					node->configuration.sensor_config[i].Sensor_ID = sensor->sensorID;
					node->configuration.sensor_config[i].Sensor_period = sensor->sampling_period_s;
					break;
				}
			}

			element = element->nextElement;
		}

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

	// Send null packet if no sensors have been loaded
	if (head == NULL) {
		for (int i = 0; i < 30; i++) {
			payload.data[i] = 0; // Clear payload
		}
		if (xQueueSendToBack(node->comms.payloadQueue, (void *)&payload, portMAX_DELAY) == pdPASS)
			node->queued_packages++;
		return;
	}

	// Else, some sensors are available to send data
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

void Node_handleCFGPayload(Node *node, Comms_Payload ACK_Payload){
	LListElement *sensor_list = node->sensor_list;
	uint8_t node_id = ACK_Payload.data[3] >> 2;
	uint8_t data = (ACK_Payload.data[3] & 0x01) ? 0x45 : 0x05; // 1 second interval.
	uint8_t alarm_change = (node->alarm == 0) != ((ACK_Payload.data[3] & 0x01) == 0);
	uint16_t period;
	Sensor *sensor;
	uint8_t index = 0;
	uint8_t writeToFlash = 0;

	// Update node ID
	if (node_id != 0) {
		node->id = node_id;
		node->configuration.node_id = node_id;
		Node_storeConfiguration(node, C_NODE_ID);
	}
	if (node->id == 0) {
		// Can't apply changes until ID is configured
		return;
	}

	// Update Alarm status
	if (alarm_change) {
		node->alarm = (ACK_Payload.data[3] & 0x01) != 0;
		HAL_I2C_Mem_Write(&hi2c, 0x46 << 1, 0xf4, 1, &data, 1, 1000);
	}

	// Are these changes persistent?
	if(ACK_Payload.data[3] & 0x02)
		writeToFlash = 1; // Bit 2 of data element 3 must make persistent changes

	// Retrieve data from package
	uint8_t *sensor_ptr = &ACK_Payload.data[4];
	while(sensor_ptr[0] != 0){
		period = (sensor_ptr[2] << 8) | sensor_ptr[1];

		sensor = (Sensor*) LList_GetElementById(sensor_list, sensor_ptr[0]);

		if(!sensor){
			index = sensorList_getIndex(sensor_ptr[0]);
			sensor_addSensor(&sensor_list, &hi2c, available_sensors[index].probe_intf, period);
		}

		sensor_setSamplingPeriod(sensor, period);
		for(int j = 0; j < (C_MAX_DATA_LEN - C_NODE_ID - 1)/2; j++){
			if((node->configuration.sensor_config[j].Sensor_ID == sensor_ptr[0]) ||
			   (node->configuration.sensor_config[j].Sensor_ID == SENSOR_NULL)) {

				node->configuration.sensor_config[j].Sensor_ID = sensor_ptr[0];
				node->configuration.sensor_config[j].Sensor_period = period;
				if(writeToFlash){
					// Commit persistent changes
					Node_storeConfiguration(node, C_S0_ID+j*2);
					Node_storeConfiguration(node, C_S0_P+j*2);
				}
				break;
			}
		}

		// prepare sensor_ptr for next iteration
		sensor_ptr += 3;
	}


	// Answer to central node, to stop sending configuration data.
	Comms_Payload payload;
	payload.data[0] = 'C';
	payload.data[1] = 'F';
	payload.data[2] = 'G';
	payload.data[3] = 'D';

	if (xQueueSendToBack(node->comms.payloadQueue, (void *)&payload, portMAX_DELAY) == pdPASS)
		node->queued_packages++;
}


void Node_handleACKPayload(Node *node) {
	Comms_Payload ACK_Payload;
	// Don't wait forever here
	while (xQueueReceive(node->comms.ACKQueue, (void *) &ACK_Payload, 0) == pdPASS) {
		// Remember to increase node->queued_packages
		Node_handleCFGPayload(node, ACK_Payload);
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


