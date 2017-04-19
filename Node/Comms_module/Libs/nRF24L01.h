/*
 * nRF24L01.h
 *
 *  Created on: Apr 17, 2017
 *      Author: javier
 */

#ifndef COMMS_MODULE_LIBS_NRF24L01_H_
#define COMMS_MODULE_LIBS_NRF24L01_H_

#include "stm32f4xx.h"


// Commands

#define		R_REGISTER			0x00
#define 	W_REGISTER			0x20
#define 	R_RX_PAYLOAD		0x61
#define 	W_TX_PAYLOAD		0xA0
#define 	FLUSH_TX			0xE1
#define 	FLUSH_RX			0xE2
#define		REUSE_TX_PL			0xE3
#define		ACTIVATE			0x50
#define		R_RX_PL_WID			0x60
#define		W_ACK_PAYLOAD		0xA8
#define 	W_TX_PAYLOAD_NO ACK	0xB0
#define		NOP					0xFF

// Registers

#define 	CONFIG				0x00
#define 	EN_AA				0x01
#define 	EN_RXADDR			0x02
#define 	SETUP_AW			0x03
#define 	SETUP_RETR			0x04
#define		RF_CH				0x05
#define		RF_SETUP			0x06
#define 	STATUS				0x07
#define		OBSERVE_TX			0x08
#define 	CD					0x09
#define 	RX_ADDR_P0			0x0A
#define 	RX_ADDR_P1			0x0B
#define 	RX_ADDR_P2			0x0C
#define 	RX_ADDR_P3			0x0D
#define 	RX_ADDR_P4			0x0E
#define 	RX_ADDR_P5			0x0F
#define 	TX_ADDR				0x10
#define 	RX_PW_P0			0x11
#define		RX_PW_P1			0x12
#define		RX_PW_P2			0x13
#define		RX_PW_P3			0x14
#define		RX_PW_P4			0x15
#define		RX_PW_P5			0x16
#define		FIFO_STATUS			0x17
#define		DYNPD				0x1C
#define		FEATURE				0x1D

// Bit definitions
#define 	CONFIG_PRIM_RX		0x01
#define 	CONFIG_PWR_UP		0x02
#define 	CONFIG_EN_CRC		0x03

#define 	RF_SETUP_RF_DR		0x08

#define 	STATUS_RX_DR		0x40
#define 	STATUS_TX_DS		0x20

#define 	ADDR_LENGTH			5

typedef enum {
	TRANSMITTER = 0x00,
	RECEIVER 	= CONFIG_PRIM_RX
} nRF24L01_Mode;

typedef enum {
	m18dBm = 0x00,
	m12dBm = 0x01,
	m6dBm  = 0x02,
	m0dBm  = 0x03
} nRF24L01_PA;

typedef struct {
	GPIO_TypeDef* 	port;
	uint16_t	  	pin;
} GPIO_PIN_TypeDef;

typedef struct {
	nRF24L01_Mode mode;
	nRF24L01_PA output_power;
	uint8_t channel; // Must be between 0 an 125 (for 1 MHz Bandwidth)
	uint8_t addr[ADDR_LENGTH];
} nRF24L01_Data;

typedef struct {
	nRF24L01_Data configuration;
	GPIO_PIN_TypeDef CE;
	GPIO_PIN_TypeDef IRQ;
	GPIO_PIN_TypeDef CSN;
	SPI_HandleTypeDef *hspi;
} nRF24L01;

int nRF24L01_init(nRF24L01 *module);
int nRF24L01_setMode(nRF24L01 *module, nRF24L01_Mode mode);

int nRF24L01_pollForRXPacket(nRF24L01 *module);
int nRF24L01_pollForTXPacket(nRF24L01 *module);

int nRF24L01_writePayload(nRF24L01 *module, uint8_t *buf, uint8_t len);
int nRF24L01_readRegister(nRF24L01 *module, uint8_t reg, uint8_t *buf, uint8_t len);
int nRF24L01_writeRegister(nRF24L01 *module, uint8_t reg, uint8_t *buf, uint8_t len);

///////////// Abstraction layer //////////////////
int nRF24L01_sendCommand(nRF24L01 *module, uint8_t command);
int nRF24L01_writeData(nRF24L01 *module, uint8_t *buf, uint8_t len);
int nRF24L01_readData(nRF24L01 *module, uint8_t *buf, uint8_t len);

#endif /* COMMS_MODULE_LIBS_NRF24L01_H_ */
