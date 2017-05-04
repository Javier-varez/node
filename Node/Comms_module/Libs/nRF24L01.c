/*
 * nRF24L01.c
 *
 *  Created on: Apr 17, 2017
 *      Author: javier
 */

#include "nRF24L01.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define nRF24L01_SPI_TIMEOUT 1000

xSemaphoreHandle IRQ_Smphr;

int nRF24L01_init(nRF24L01 *module) {
	uint8_t rc = 0;

	// Init semaphore
	IRQ_Smphr = xSemaphoreCreateBinary(); // Initially empty
	module->IRQ_Semaphore = IRQ_Smphr;

	// Set chip enable low
	HAL_GPIO_WritePin(module->CE.port, module->CE.pin, GPIO_PIN_RESET);

	// Wait POR just in case
	vTaskDelay(20/portTICK_RATE_MS);

	// Power up, set mode, enable CRC (1 bit)
	uint8_t reg = CONFIG_EN_CRC | module->configuration.mode;
	nRF24L01_writeRegister(module, CONFIG, &reg, 1);

	// Setup automatic retransmission to 1500us delay to allow 32 bytes payloads
	reg = 0x4f;
	nRF24L01_writeRegister(module, SETUP_RETR, &reg, 1);

	// Configure channel
	nRF24L01_writeRegister(module, RF_CH, &module->configuration.channel, 1);

	// Configure PA
	reg = (module->configuration.output_power << 1) | RF_SETUP_LNA_HCURR;
	nRF24L01_writeRegister(module, RF_SETUP, &reg, 1);

	// Configure TX address
	nRF24L01_writeRegister(module, TX_ADDR, module->configuration.addr, ADDR_LENGTH);

	// Configure RX address (PIPE 0)
	nRF24L01_writeRegister(module, RX_ADDR_P0, module->configuration.addr, ADDR_LENGTH);

	// Configure
	reg = 32; // Set RX payload size
	nRF24L01_writeRegister(module, RX_PW_P0, &reg, 1);

	// Disable AutoACK
	reg = 0x00;
	nRF24L01_writeRegister(module, EN_AA, &reg, 1);

	reg = 0x01; // Enable pipe 0
	nRF24L01_writeRegister(module, EN_RXADDR, &reg, 1);

	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_RESET);
	nRF24L01_sendCommand(module, FLUSH_TX);
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_SET);

	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_RESET);
	nRF24L01_sendCommand(module, FLUSH_RX);
	HAL_GPIO_WritePin(module->CSN.port, module->CSN.pin, GPIO_PIN_SET);

	reg = STATUS_RX_DR | STATUS_TX_DS | STATUS_MAX_RT;
	nRF24L01_writeRegister(module, STATUS, &reg, 1);

	nRF24L01_readRegister(module, STATUS, &reg, 1);

	nRF24L01_powerUp(module);

	return rc;
}

int nRF24L01_powerUp(nRF24L01* module) {
	uint8_t reg;
	// Power up
	nRF24L01_readRegister(module, CONFIG, &reg, 1);
	reg |= CONFIG_PWR_UP;
	nRF24L01_writeRegister(module, CONFIG, &reg, 1);

	// Delay more than 2ms to provide powerup
	vTaskDelay(4/portTICK_RATE_MS);

	return 0;
}

int nRF24L01_setMode(nRF24L01 *module, nRF24L01_Mode mode) {
	module->configuration.mode = mode;

	uint8_t config_reg;
	if (nRF24L01_readRegister(module, CONFIG, &config_reg, 1) == 0) {
		if (mode == TRANSMITTER) {
			config_reg &= ~CONFIG_PRIM_RX;
			nRF24L01_applyIRQMask(module, MASK_RX_DR | MASK_MAX_RT);
		}
		else if (mode == RECEIVER) {
			config_reg |= CONFIG_PRIM_RX;
			nRF24L01_applyIRQMask(module, MASK_TX_DS | MASK_MAX_RT);
		}

		return nRF24L01_writeRegister(module, CONFIG, &config_reg, 1);
	}

	return 1;
}

int nRF24L01_transmit(nRF24L01 *module, uint8_t *payload) {
	uint8_t rc = 0;
	rc = nRF24L01_writePayload(module, payload, 32); // 32 bytes payload
#warning TODO: Implement CE signal with timer
	HAL_GPIO_WritePin(module->CE.port, module->CE.pin, GPIO_PIN_SET);
	vTaskDelay(1/portTICK_PERIOD_MS);
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

	// Clear flag if fifo is already empty
	uint8_t reg;
	nRF24L01_readRegister(module, FIFO_STATUS, &reg, 1);
	if (reg & RX_EMPTY)
		nRF24L01_clearIRQ(module, STATUS_RX_DR);

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

void nRF24L01_IRQ_Received() {
	xSemaphoreGiveFromISR(IRQ_Smphr, NULL);
}

void nRF24L01_clearIRQ(nRF24L01 *module, uint8_t irq) {
	xSemaphoreTake(module->IRQ_Semaphore, 0); // Just in case there is a pending semaphore

	// Clear status reg
	nRF24L01_writeRegister(module, STATUS, &irq, 1);
}

void nRF24L01_applyIRQMask(nRF24L01 *module, uint8_t mask) {
	// Config IRQ Mask
	uint8_t data = 0x00;
	nRF24L01_readRegister(module, CONFIG, &data, 1);

	// Apply mask
	data &= ~(MASK_RX_DR | MASK_TX_DS | MASK_MAX_RT);
	data |= mask;

	nRF24L01_writeRegister(module, CONFIG, &data, 1);
}

int nRF24L01_pollForRXPacketWithTimeout(nRF24L01 *module, uint32_t timeout_ms) {
	uint8_t rc = 0;
	uint8_t reg = 0x00;

	// CHECK IF Flag is already active
	nRF24L01_readRegister(module, STATUS, &reg, 1);
	if (reg & STATUS_RX_DR)
		return 1;

	// Else, wait for IRQ
	nRF24L01_clearIRQ(module, STATUS_RX_DR);
	// Start receiving
	HAL_GPIO_WritePin(module->CE.port, module->CE.pin, GPIO_PIN_SET);

	xSemaphoreTake(module->IRQ_Semaphore, timeout_ms/portTICK_RATE_MS);

	// Received IRQ, check for STATUS_RX_DR FLAG
	nRF24L01_readRegister(module, STATUS, &reg, 1);
	if (reg & STATUS_RX_DR)
		rc = 1;

	// Enter Standby
	HAL_GPIO_WritePin(module->CE.port, module->CE.pin, GPIO_PIN_RESET);

	return rc;
}

int nRF24L01_pollForTXPacketWithTimeout(nRF24L01 *module, uint32_t timeout_ms) {
	uint8_t rc = 0;
	uint8_t status = 0x00;

	// CHECK IF Flag is already active
	nRF24L01_readRegister(module, STATUS, &status, 1);
	if (status & STATUS_TX_DS) {
		nRF24L01_clearIRQ(module, STATUS_TX_DS);
		return 1;
	}

	nRF24L01_clearIRQ(module, STATUS_TX_DS);
	xSemaphoreTake(module->IRQ_Semaphore, timeout_ms/portTICK_RATE_MS);

	nRF24L01_readRegister(module, STATUS, &status, 1);

	if (status & STATUS_TX_DS) {
		// Clear IT flag by writing 1
		status = STATUS_TX_DS;
		nRF24L01_writeRegister(module, STATUS, &status, 1);
		rc = 1;
	}

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
