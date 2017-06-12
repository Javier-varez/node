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
		else continue;
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
		LListElement *sensor_list = node->sensor_list;
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
			for(int i = 0; i< (C_MAX_DATA_LEN - C_NODE_ID - 1)/2; i++){
				sensor = (Sensor*)LList_GetElement(sensor_list,i);
				if(sensor){
					node->configuration.sensor_config[i].Sensor_ID = sensor->sensorID;
					node->configuration.sensor_config[i].Sensor_period = sensor->sampling_period_s;
				} else break;
			}
		} else {
			node->id = node->configuration.node_id;
			int index = 0;
			for(int i = 0; i < (C_MAX_DATA_LEN - C_NODE_ID - 1)/2; i++){
				if(node->configuration.sensor_config[i].Sensor_ID != SENSOR_NULL){
					sensor = (Sensor*)LList_GetElementById(sensor_list, node->configuration.sensor_config[i].Sensor_ID);
					//If sensor is loaded, apply the saved period
					if(sensor){
						//If period is max(15 bits are '1') then assign max uint32 as period
						if(node->configuration.sensor_config[i].Sensor_period == (0xFFFF >> 1)){
							sensor->sampling_period_s = UINT32_MAX;
						} else{
							sensor->sampling_period_s = node->configuration.sensor_config[i].Sensor_period;
						}
					} else{
						//If sensor is not loaded automatically (not discoverable) load it
						index = sensorList_getIndex(node->configuration.sensor_config[i].Sensor_ID);
						//If sensor is available, load it:
						if(index >= 0){
							sensor_addSensor(&sensor_list,&hi2c, available_sensors[index].probe_intf, node->configuration.sensor_config[i].Sensor_period);
						}
					}
				}
			}
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

void Node_handleCFGPayload(Node *node, Comms_Payload ACK_Payload){
	LListElement *sensor_list = node->sensor_list;
	uint8_t data = 0x00;
	uint16_t period[NUM_SENSORS];
	Sensor *sensor;
	uint8_t index = 0;
	uint8_t writeToFlash = 0;
	//Initializing period array to 0
	for(int i = 0; i< NUM_SENSORS; i++){
		period[i] = 0x00;
	}
	if ((ACK_Payload.data[3] & 0x01) == 0x01) {
		data = 0x45;
	} else if (ACK_Payload.data[4] == '0') {
		data = 0x05;
	}
	HAL_I2C_Mem_Write(&hi2c, 0x46 << 1, 0xf4, 1, &data, 1, 1000);
	if((ACK_Payload.data[3] & 0x02) == 0x01){
		writeToFlash = 1;
	}
	for(int i = 4; i<32; i++){
		//Check flag of payload
		switch (ACK_Payload.data[i]){
		case HTS221_ID:
			if(ACK_Payload.data[i+2]>127){
				period[HTS221_ID-1] = 0xFF;
			} else {
				period[HTS221_ID-1] = ((ACK_Payload.data[i+2] & 0x7F) << 8) | ACK_Payload.data[i+1];
			}
			i+=2;
			break;
		case LPS25H_ID:
			if(ACK_Payload.data[i+2]>127){
				period[LPS25H_ID-1] = 0xFF;
			} else {
				period[LPS25H_ID-1] = ((ACK_Payload.data[i+2] & 0x7F) << 8) | ACK_Payload.data[i+1];
			}
			i+=2;
			break;
		case LSM9DS1_AG_ID:
			if(ACK_Payload.data[i+2]>127){
				period[LSM9DS1_AG_ID-1] = 0xFF;
			} else {
				period[LSM9DS1_AG_ID-1] = ((ACK_Payload.data[i+2] & 0x7F) << 8) | ACK_Payload.data[i+1];
			}
			i+=2;
			break;
		case LSM9DS1_M_ID:
			if(ACK_Payload.data[i+2]>127){
				period[LSM9DS1_M_ID-1] = 0xFF;
			} else {
				period[LSM9DS1_M_ID-1] = ((ACK_Payload.data[i+2] & 0x7F) << 8) | ACK_Payload.data[i+1];
			}
			i+=2;
			break;
		default:
			break;
		}
		if (ACK_Payload.data[i] == 0x00) break;
	}
	//Iterate through sensor list and change sampling period
	for(int i = 0; i<NUM_SENSORS;i++){
		//If period was not changed in this package do nothing
		if(period[i] == 0) continue;
		//Get sensor with ID i+1
		sensor = (Sensor*) LList_GetElementById(sensor_list, i+1);
		//If sensor is loaded change period
		if(sensor){
			sensor->sampling_period_s = period[i];
			for(int j = 0; j < (C_MAX_DATA_LEN - C_NODE_ID - 1)/2; j++){
				if(node->configuration.sensor_config[j].Sensor_ID == i+1){
					node->configuration.sensor_config[j].Sensor_period = period[i];
					if(writeToFlash){
						//Write sensor state in flash
						Node_storeConfiguration(node, C_NODE_ID+j*2+1);
						Node_storeConfiguration(node, C_NODE_ID+j*2+2);
					}
				} else if(node->configuration.sensor_config[j].Sensor_ID == SENSOR_NULL){
					node->configuration.sensor_config[j].Sensor_ID = i+1;
					node->configuration.sensor_config[j].Sensor_period = period[i];
					if(writeToFlash){
						Node_storeConfiguration(node, C_NODE_ID+j*2+1);
						Node_storeConfiguration(node, C_NODE_ID+j*2+2);
					}
				}
			}
		} else{
			//Si el sensor no esta cargado se carga
			index = sensorList_getIndex(i+1);
			if(index >= 0){
				sensor_addSensor(&sensor_list,&hi2c, available_sensors[index].probe_intf, period[i]);
				for(int j = 0; j < (C_MAX_DATA_LEN - C_NODE_ID - 1)/2; j++){
					if(node->configuration.sensor_config[j].Sensor_ID == i+1){
						node->configuration.sensor_config[j].Sensor_period = period[i];
						if(writeToFlash){
							Node_storeConfiguration(node, C_NODE_ID+j*2+1);
							Node_storeConfiguration(node, C_NODE_ID+j*2+2);
						}
					} else if(node->configuration.sensor_config[j].Sensor_ID == SENSOR_NULL){
						node->configuration.sensor_config[j].Sensor_ID = i+1;
						node->configuration.sensor_config[j].Sensor_period = period[i];
						if(writeToFlash){
							Node_storeConfiguration(node, C_NODE_ID+j*2+1);
							Node_storeConfiguration(node, C_NODE_ID+j*2+2);
						}
					}
				}
			}
		}
	}

}


void Node_handleACKPayload(Node *node) {
	Comms_Payload ACK_Payload;
	// Don't wait forever here
	while (xQueueReceive(node->comms.ACKQueue, (void *) &ACK_Payload, 0) == pdPASS) {
		// Remember to increase node->queued_packages
		if (ACK_Payload.data[0] == 'C' &&
				ACK_Payload.data[1] == 'F' &&
				ACK_Payload.data[2] == 'G') Node_handleCFGPayload(node, ACK_Payload);

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


