/*
 * Comms_module.c
 *
 *  Created on: Apr 19, 2017
 *      Author: javier
 */

#include "Comms_module.h"
#include "init_periph.h"

#define 	COMMS_CHANNEL	24
#define		RX_TIMEOUT_MS	5
#define		TX_TIMEOUT_MS	2
#define 	TX_ADDR_0		0xE7
#define 	TX_ADDR_1		0xE7
#define 	TX_ADDR_2		0xE7
#define 	TX_ADDR_3		0xE7
#define 	TX_ADDR_4		0xE7

SPI_HandleTypeDef hspi;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
	if (GPIO_Pin == GPIO_PIN_3) {
		// Signal nRF24L01 IRQ through a semaphore
		nRF24L01_IRQ_Received();
	}
}

int comms_module_Init(Comms_module *module, Comms_module_Mode mode, uint8_t id) {
	spi_init(&hspi);
	gpio_init();

	// Module Address (Not related to nRF24L01)
	module->address = id;
	module->PID = 0x00;

	module->device.configuration.addr[0] = TX_ADDR_0;
	module->device.configuration.addr[1] = TX_ADDR_1;
	module->device.configuration.addr[2] = TX_ADDR_2;
	module->device.configuration.addr[3] = TX_ADDR_3;
	module->device.configuration.addr[4] = TX_ADDR_4;
	module->device.configuration.channel = COMMS_CHANNEL; // Test channel
	module->device.configuration.output_power = m0dBm;
	module->device.hspi = &hspi;
	module->device.CE.pin = GPIO_PIN_5;
	module->device.CE.port = GPIOA;
	module->device.IRQ.pin = GPIO_PIN_3;
	module->device.IRQ.port = GPIOA;
	module->device.CSN.pin = GPIO_PIN_4;
	module->device.CSN.port = GPIOA;

	module->device.configuration.mode = mode;

	nRF24L01_init(&module->device);

	return 0;
}

int comms_module_sendData(Comms_module *module, Comms_Payload *payload) {

	nRF24L01 *device = &module->device;
	Comms_Payload ACK_Payload;
	uint8_t received_ack = 0;

	// Prepare payload
	payload->address = module->address;
	payload->PID = module->PID;

	// We will continue to send the package as long as we haven't received an ACK
	do {
		// Transmit data
		nRF24L01_setMode(device, TRANSMITTER);
		nRF24L01_transmit(device, (uint8_t*)payload);
		nRF24L01_pollForTXPacketWithTimeout(device, TX_TIMEOUT_MS);

		// Wait ACK
		nRF24L01_setMode(device, RECEIVER);
		if (nRF24L01_pollForRXPacketWithTimeout(device, RX_TIMEOUT_MS)) {
			while(nRF24L01_fifoNotEmpty(device)) {
				nRF24L01_readPayload(device, (uint8_t*)&ACK_Payload, sizeof(ACK_Payload));
				if ((ACK_Payload.address == module->address) &&
					(ACK_Payload.PID == payload->PID)){
					received_ack = 1;
					break; //exit loop, we already found what we wanted
				}
			}
		}

	} while(!received_ack);

	// Handle Payload
	if (ACK_Payload.data[0] == 'A' &&
		ACK_Payload.data[1] == 'C' &&
		ACK_Payload.data[2] == 'K') {
		// Received empty ACK package
	} else if (ACK_Payload.data[0] == 'C' &&
			   ACK_Payload.data[1] == 'F' &&
			   ACK_Payload.data[2] == 'G') {
		// Received CFG package
		xQueueSendToBack(module->ACKQueue, (void *) &ACK_Payload, portMAX_DELAY);
	}

	// Increment PID after complete transaction;
	module->PID++;
	// Increment sent_packages
	module->sent_packages++;

	return 0;
}

void comms_task(void *param) {
	Comms_module *module = (Comms_module*)param;

	Comms_Payload payload;

	while(1) {
		// Handle Data Transmission between modules and center node
		xQueueReceive(module->payloadQueue, (void *) &payload, portMAX_DELAY); // Receive data to be sent

		// Transmit payload
		comms_module_sendData(module, &payload);
	}
}
