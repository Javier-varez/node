/*
 * Comms_module.c
 *
 *  Created on: Apr 19, 2017
 *      Author: javier
 */

#include "Comms_module.h"
#include "init_periph.h"

SPI_HandleTypeDef hspi;

int comms_module_Init(Comms_module *module, Comms_module_Mode mode) {
	spi_init(&hspi);
	gpio_init();

	module->device.configuration.addr[0] = 0xE7;
	module->device.configuration.addr[1] = 0xE7;
	module->device.configuration.addr[2] = 0xE7;
	module->device.configuration.addr[3] = 0xE7;
	module->device.configuration.addr[4] = 0xE7;
	module->device.configuration.channel = 24; // Test channel
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
