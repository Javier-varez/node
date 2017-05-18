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
extern struct_sensor_list available_sensors[];


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
		LListElement *sensor_list = node->sensor_list;
		// Init peripherals
		rc |= i2c_init(&hi2c, &hdma);
		rc |= rtc_init(&hrtc);

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
			int index = 0;
			for(int i = 0; i < (C_MAX_DATA_LEN - C_NODE_ID - 1)/2; i++){
				if(node->configuration.sensor_config[i].Sensor_ID != SENSOR_NULL){
					index = sensorList_getIndex(node->configuration.sensor_config[i].Sensor_ID);
					if(index == -1){
						//Sensor doesn't exist
					} else{
						//If sensor exists, initialize it (Sensor indice i en flash esta en el indice index en ram)
						sensor_addSensor(&node->sensor_list, &hi2c, available_sensors[index].probe_intf,
								node->configuration.sensor_config[i].Sensor_period);
					}
				}
			}
			//Apply saved id to node
			node->id = node->configuration.node_id;
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
	LListElement *sensor_list = node->sensor_list;
	// Don't wait forever here
	while (xQueueReceive(node->comms.ACKQueue, (void *) &ACK_Payload, 0) == pdPASS) {
		// Remember to increase node->queued_packages
		if (ACK_Payload.data[0] == 'C' &&
				ACK_Payload.data[1] == 'F' &&
				ACK_Payload.data[2] == 'G') {
			uint8_t data = 0x00;
			uint16_t period[NUM_SENSORS];
			//Initializing period array to 0
			for(int i = 0; i< NUM_SENSORS; i++){
				period[i] = 0x00;
			}
			uint8_t writeToMemory = 0;
			if ((ACK_Payload.data[3] & 0x01) == 0x01) {
				data = 0x45;
			} else if (ACK_Payload.data[4] == '0') {
				data = 0x05;
			}
			HAL_I2C_Mem_Write(&hi2c, 0x46 << 1, 0xf4, 1, &data, 1, 1000);
			if((ACK_Payload.data[3] & 0x02) == 0x01){
				writeToMemory = 1;
			}
			for(int i = 4; i<32; i++){
				switch (ACK_Payload.data[i]){
				case HTS221_ID:
					if(ACK_Payload.data[i+2]>127){
						period[HTS221_ID-1] = 0xFF;
					} else {
						period[HTS221_ID-1] = (ACK_Payload.data[i+1] << 7) | (ACK_Payload.data[i+2] & 0x7F);
					}
					i+=2;
					break;
				case LPS25H_ID:
					if(ACK_Payload.data[i+2]>127){
						period[LPS25H_ID-1] = 0xFF;
					} else {
						period[LPS25H_ID-1] = (ACK_Payload.data[i+1] << 7) | (ACK_Payload.data[i+2] & 0x7F);
					}
					i+=2;
					break;
				case LSM9DS1_AG_ID:
					if(ACK_Payload.data[i+2]>127){
						period[LSM9DS1_AG_ID-1] = 0xFF;
					} else {
						period[LSM9DS1_AG_ID-1] = (ACK_Payload.data[i+1] << 7) | (ACK_Payload.data[i+2] & 0x7F);
					}
					i+=2;
					break;
				case LSM9DS1_M_ID:
					if(ACK_Payload.data[i+2]>127){
						period[LSM9DS1_M_ID-1] = 0xFF;
					} else {
						period[LSM9DS1_M_ID-1] = (ACK_Payload.data[i+1] << 7) | (ACK_Payload.data[i+2] & 0x7F);
					}
					i+=2;
					break;
				default:
					break;
				}
				if (ACK_Payload.data[i] == 0x00) break;
			}
			//Iterate through sensor list and change sampling period
			while(sensor_list != NULL){
				if(((Sensor *)sensor_list->content)->sensorID == HTS221_ID && period[HTS221_ID-1] != 0){
					sensor_setSamplingPeriod(((Sensor *)sensor_list->content), period[HTS221_ID-1]);
					//hace falta añadir check para asegurar que el sensor este en node_configuration?
					Node_Configuration_setPeriod(node, HTS221_ID, period[HTS221_ID]);
					//Write period to flash
					if(writeToMemory){
						Node_storeConfiguration(node, C_NODE_ID+HTS221_ID*2);
					}
				} else if(((Sensor *)sensor_list->content)->sensorID == LPS25H_ID && period[LPS25H_ID-1] != 0){
					sensor_setSamplingPeriod(((Sensor *)sensor_list->content), period[LPS25H_ID-1]);
					Node_Configuration_setPeriod(node, HTS221_ID, period[LPS25H_ID]);
					if(writeToMemory){
						Node_storeConfiguration(node, C_NODE_ID+LPS25H_ID*2);
					}
				} else if(((Sensor *)sensor_list->content)->sensorID == LSM9DS1_AG_ID && period[LSM9DS1_AG_ID-1] != 0){
					sensor_setSamplingPeriod(((Sensor *)sensor_list->content), period[LSM9DS1_AG_ID-1]);
					Node_Configuration_setPeriod(node, HTS221_ID, period[LSM9DS1_AG_ID]);
					if(writeToMemory){
						Node_storeConfiguration(node, C_NODE_ID+LSM9DS1_AG_ID*2);
					}
				} else if(((Sensor *)sensor_list->content)->sensorID == LSM9DS1_M_ID && period[LSM9DS1_M_ID-1] != 0){
					sensor_setSamplingPeriod(((Sensor *)sensor_list->content), period[LSM9DS1_M_ID-1]);
					Node_Configuration_setPeriod(node, HTS221_ID, period[LSM9DS1_M_ID]);
					if(writeToMemory){
						Node_storeConfiguration(node, C_NODE_ID+LSM9DS1_M_ID*2);
					}
				}
				else{
					sensor_list = sensor_list->nextElement;
				}
			}

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

