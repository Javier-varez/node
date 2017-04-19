/*
 * nRF24L01.c
 *
 *  Created on: Apr 17, 2017
 *      Author: javier
 */

#include "nRF24L01.h"

#define nRF24L01_SPI_TIMEOUT 1000

int nRF24L01_init(nRF24L01 *module) {
	uint8_t rc = 0;

	// Set chip enable low
	HAL_GPIO_WritePin(module->CE.port, module->CE.pin, GPIO_PIN_RESET);

	// Wait POR just in case
	HAL_Delay(10);

	// Power up, set mode, enable CRC (1 bit)
	uint8_t reg = CONFIG_PWR_UP | CONFIG_EN_CRC | module->configuration.mode;
	nRF24L01_writeRegister(module, CONFIG, &reg, 1);

	// Delay 2ms to provide powerup
	HAL_Delay(2);

	// Setup automatic retransmission to 500us delay to allow 32 bytes payloads
	reg = 0x13;
	nRF24L01_writeRegister(module, SETUP_RETR, &reg, 1);

	// Configure channel
	nRF24L01_writeRegister(module, RF_CH, &module->configuration.channel, 1);

	// Configure PA
	reg = RF_SETUP_RF_DR | (module->configuration.output_power << 1);
	nRF24L01_writeRegister(module, RF_SETUP, &reg, 1);

	// Configure TX address
	nRF24L01_writeRegister(module, TX_ADDR, module->configuration.addr, ADDR_LENGTH);

	// Configure RX address (PIPE 0)
	nRF24L01_writeRegister(module, RX_ADDR_P0, module->configuration.addr, ADDR_LENGTH);

	return rc;
}

int nRF24L01_setMode(nRF24L01 *module, nRF24L01_Mode mode) {
	module->configuration.mode = mode;

	uint8_t config_reg;
	if (nRF24L01_readRegister(module, CONFIG, &config_reg, 1) == 0) {
		if (mode == TRANSMITTER)
			config_reg &= ~CONFIG_PRIM_RX;
		else if (mode == RECEIVER)
			config_reg |= CONFIG_PRIM_RX;

		return nRF24L01_writeRegister(module, CONFIG, &config_reg, 1);
	}

	return 1;
}

int nRF24L01_transmit(nRF24L01 *module, uint8_t *payload) {
	uint8_t rc = 0;
	rc = nRF24L01_writePayload(module, payload, 32); // 32 bytes payload
	HAL_GPIO_WritePin(module->CE.port, module->CE.pin, GPIO_PIN_SET);
	HAL_Delay(1);
	HAL_GPIO_WritePin(module->CE.port, module->CE.pin, GPIO_PIN_RESET);

	return rc;
}

#warning TODO: Implement interrupt support for nRF24L01
int nRF24L01_pollForRXPacket(nRF24L01 *module) {
	uint8_t status = 0;
	uint8_t rc = 0;

	HAL_GPIO_WritePin(module->CE.port, module->CE.pin, GPIO_PIN_SET);

	while (!(status & STATUS_RX_DR)) {
		if ((rc = nRF24L01_readRegister(module, STATUS, &status, 1)) != 0)
			break;
	}

	if (rc == 0) {
		// Clear IT flag by writing 1
		status = STATUS_RX_DR;
		rc = nRF24L01_writeRegister(module, STATUS, &status, 1);
	}

	HAL_GPIO_WritePin(module->CE.port, module->CE.pin, GPIO_PIN_RESET);

	return rc;
}

int nRF24L01_pollForTXPacket(nRF24L01 *module) {
	uint8_t status = 0;
	uint8_t rc = 0;
	while (!(status & STATUS_TX_DS)) {
		if ((rc = nRF24L01_readRegister(module, STATUS, &status, 1)) != 0)
			break;
	}
	if (rc == 0) {
		// Clear IT flag by writing 1
		status = STATUS_TX_DS;
		rc = nRF24L01_writeRegister(module, STATUS, &status, 1);
	}
	return rc;
}

int nRF24L01_writePayload(nRF24L01 *module, uint8_t *buf, uint8_t len) {
	uint8_t rc = 0;
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_RESET);

	if (nRF24L01_sendCommand(module, W_TX_PAYLOAD) == 0) {
		rc = nRF24L01_writeData(module, buf, len);
	}
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_SET);

	return rc;
}

int nRF24L01_readPayload(nRF24L01 *module, uint8_t *buf, uint8_t len) {
	uint8_t rc = 0;
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_RESET);

	if (nRF24L01_sendCommand(module, R_RX_PAYLOAD) == 0) {
		rc = nRF24L01_readData(module, buf, len);
	}
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_SET);

	return rc;
}

int nRF24L01_readRegister(nRF24L01 *module, uint8_t reg, uint8_t *buf, uint8_t len) {
	uint8_t rc = 0;
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_RESET);

	if (nRF24L01_sendCommand(module, R_REGISTER | reg) == 0) {
		rc = nRF24L01_readData(module, buf, len);
	}
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_SET);

	return rc;
}

int nRF24L01_writeRegister(nRF24L01 *module, uint8_t reg, uint8_t *buf, uint8_t len) {
	uint8_t rc = 0;
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_RESET);

	if (nRF24L01_sendCommand(module, W_REGISTER | reg) == 0) {
		rc = nRF24L01_writeData(module, buf, len);
	}
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_SET);

	return rc;
}

///////////// Abstraction layer //////////////////

inline int nRF24L01_sendCommand(nRF24L01 *module, uint8_t command) {
	return HAL_SPI_Transmit(module->hspi, &command, 1, nRF24L01_SPI_TIMEOUT);
}

inline int nRF24L01_readData(nRF24L01 *module, uint8_t *buf, uint8_t len) {
	return HAL_SPI_Receive(module->hspi, buf, len, nRF24L01_SPI_TIMEOUT);
}

inline int nRF24L01_writeData(nRF24L01 *module, uint8_t *buf, uint8_t len) {
	return HAL_SPI_Transmit(module->hspi, buf, len, nRF24L01_SPI_TIMEOUT);
}
