/*
 * LSM9DS1_M.c
 *
 *  Created on: Apr 15, 2017
 *      Author: javier
 */

#include "LSM9DS1_M.h"
#include <stdlib.h>
#include <string.h>

Sensor_I2C_Probe_Intf LSM9DS1_M_intf = {
	.init = LSM9DS1_M_init,
	.remove = LSM9DS1_M_remove
};

#define SIZE_OF_INIT_ARRAY(X)  (sizeof(X)/sizeof(X[0]))

#define		LSM9DS1_M_DEV_ADDR			0x1C
#define		LSM9DS1_M_ID_VALUE			0x3D

/* Register Addresses */

#define 	OFFSET_X_REG_L			0x05
#define 	OFFSET_X_REG_H			0x06
#define 	OFFSET_Y_REG_L			0x07
#define 	OFFSET_Y_REG_H			0x08
#define 	OFFSET_Z_REG_L			0x09
#define 	OFFSET_Z_REG_H			0x0A
#define 	WHO_AM_I				0x0F
#define		CTRL_REG1				0x20
#define 	CTRL_REG2				0x21
#define 	CTRL_REG3				0x22
#define 	CTRL_REG4				0x23
#define		CTRL_REG5				0x24
#define 	STATUS_REG				0x27
#define		OUT_X_L					0x28
#define		OUT_X_H					0x29
#define		OUT_Y_L					0x2A
#define		OUT_Y_H					0x2B
#define		OUT_Z_L					0x2C
#define		OUT_Z_H					0x2D
#define		INT_CFG					0x30
#define 	INT_SRC					0x31
#define		INT_THS_L				0x32
#define		INT_THS_H				0x33

/* Register bit definitions */
#define 	CTRL_REG1_TEMP_COMP		0x80
#define 	CTRL_REG1_OM_POS		0x05
#define		CTRL_REG1_DO_POS		0x02
#define 	CTRL_REG1_FAST_ODR		0x02
#define		CTRL_REG1_ST			0x01

#define 	CTRL_REG2_FS_POS		0x05
#define		CTRL_REG2_REBOOT		0x08
#define 	CTRL_REG2_SOFT_RST		0x04

#define		CTRL_REG3_I2C_DIS		0x80
#define 	CTRL_REG3_LP			0x20
#define 	CTRL_REG3_SIM			0x04
#define		CTRL_REG3_MD_POS		0x00

#define 	CTRL_REG4_OM_POS		0x02
#define 	CTRL_REG4_BLE			0x02

#define 	CTRL_REG5_FAST_READ		0x80
#define		CTRL_REG5_BDU			0x40

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Initialization //////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

static Sensor_I2C_Init_Pair LSM9DS1_M_reg_pairs[] = {
	{
		.reg = CTRL_REG1,
		// ODR = 0.625 Hz, Low power mode (X and Y)
		.value = (0x00 << CTRL_REG1_OM_POS) | (0x00 << CTRL_REG1_DO_POS),
	},
	{
		.reg = CTRL_REG2,
		// Full Scale = +- 4 gauss
		.value = (0x00 << CTRL_REG2_FS_POS),
	},
	{
		.reg = CTRL_REG3,
		// Continuous conversion mode, Low power mode
		.value = CTRL_REG3_LP | (0x00 << CTRL_REG3_MD_POS),
	},
	{
		.reg = CTRL_REG4,
		// Low power mode (Z), Little endian interface configuration
		.value = 0x00 << CTRL_REG4_OM_POS,
	},
	{
		.reg = CTRL_REG5,
		// Block Data Update
		.value = CTRL_REG5_BDU,
	},
};

static Sensor_I2C_Init LSM9DS1_M_init_array = {
	.array = LSM9DS1_M_reg_pairs,
	.size = SIZE_OF_INIT_ARRAY(LSM9DS1_M_reg_pairs),
	.reg_size = sizeof(uint8_t), // Byte addr
	.data_size = sizeof(uint8_t), // Byte data
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Power Down //////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

static Sensor_I2C_Init_Pair LSM9DS1_M_powerDown_reg_pairs[] = {
	{
		.reg = CTRL_REG3,
		// Continuous conversion mode, PowerDown mode
		.value = CTRL_REG3_LP | (0x03 << CTRL_REG3_MD_POS),
	},
};

static Sensor_I2C_Power LSM9DS1_M_powerDown_array = {
	.array = LSM9DS1_M_powerDown_reg_pairs,
	.size = SIZE_OF_INIT_ARRAY(LSM9DS1_M_powerDown_reg_pairs),
	.reg_size = sizeof(uint8_t), // Byte addr
	.data_size = sizeof(uint8_t), // Byte data
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Power Up ////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

static Sensor_I2C_Init_Pair LSM9DS1_M_powerUp_reg_pairs[] = {
	{
		.reg = CTRL_REG3,
		// Continuous conversion mode, Low power mode
		.value = CTRL_REG3_LP | (0x00 << CTRL_REG3_MD_POS),
	},
};

static Sensor_I2C_Power LSM9DS1_M_powerUp_array = {
	.array = LSM9DS1_M_powerUp_reg_pairs,
	.size = SIZE_OF_INIT_ARRAY(LSM9DS1_M_powerUp_reg_pairs),
	.reg_size = sizeof(uint8_t), // Byte addr
	.data_size = sizeof(uint8_t), // Byte data
};

/////////////////////////////////////////////////////////////////////////////////
/////////////////////////// Functions ///////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////

int LSM9DS1_M_read_regs(Sensor *sensor, void* data) {
	HAL_StatusTypeDef rc = HAL_OK;

	sensor_delay(15);

	LSM9DS1_M_Out_Data * out_data = (LSM9DS1_M_Out_Data*) data;

	rc = HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						  OUT_X_L, sensor->data.i2c.read_addr_size,
						  (uint8_t*)out_data->magnetometer, sizeof(out_data->magnetometer), 1000);

	return rc;
}

void LSM9DS1_M_packData(Sensor *sensor, uint8_t *data) {
	memcpy(data, sensor->out_data, sizeof(LSM9DS1_M_Out_Data));
}

static Sensor_Func_Table LSM9DS1_M_func_table = {
	.probe = i2c_sensor_probe,
	.read = LSM9DS1_M_read_regs,
	.init = i2c_sensor_init_regs,
	.packData = LSM9DS1_M_packData,
	.powerDown = i2c_sensor_powerDown,
	.powerUp = i2c_sensor_powerUp
};


int LSM9DS1_M_init(Sensor *sensor, I2C_HandleTypeDef *hi2c) {
	uint8_t rc = i2c_sensor_init(sensor, "LSM9DS1_M", &LSM9DS1_M_func_table, hi2c, LSM9DS1_M_DEV_ADDR << 1, &LSM9DS1_M_init_array,
								 OUT_X_L, sizeof(uint8_t), 3*sizeof(int16_t), WHO_AM_I, LSM9DS1_M_ID_VALUE);

	if (rc == HAL_OK) {
		sensor->out_data = malloc(sizeof(LSM9DS1_M_Out_Data));
		if (sensor->out_data == NULL) rc = 0;
	}

	sensor->data.i2c.powerup = &LSM9DS1_M_powerUp_array;
	sensor->data.i2c.powerdown = &LSM9DS1_M_powerDown_array;

	return rc;
}

int LSM9DS1_M_remove(Sensor *sensor) {
	free(sensor->out_data);
	return 0;
}
