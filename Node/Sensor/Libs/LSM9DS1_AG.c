/*
 * LSM9DS1_AG.c
 *
 *  Created on: Apr 13, 2017
 *      Author: javier
 */

#include "LSM9DS1_AG.h"
#include <stdlib.h>

Sensor_I2C_Probe_Intf LSM9DS1_AG_intf = {
	.init = LSM9DS1_AG_init,
	.remove = LSM9DS1_AG_remove
};
#define SIZE_OF_INIT_ARRAY(X)  (sizeof(X)/sizeof(X[0]))

#define		LSM9DS1_AG_DEV_ADDR			0x6A
#define		LSM9DS1_AG_ID_VALUE			0x68

/* Register Addresses */

#define 	ACT_THS					0x04
#define		ACT_DUR					0x05
#define 	INT_GEN_CFG_XL			0x06
#define 	INT_GEN_THS_X_XL		0x07
#define		INT_GEN_THS_Y_XL		0x08
#define		INT_GEN_THS_Z_XL		0x09
#define 	INT_GEN_DUR_XL			0x0A
#define 	REFERENCE_G				0x0B
#define 	INT1_CTRL				0x0C
#define 	INT2_CTRL				0x0D
#define 	WHO_AM_I				0x0F
#define		CTRL_REG1_G				0x10
#define 	CTRL_REG2_G				0x11
#define 	CTRL_REG3_G				0x12
#define 	ORIENT_CFG_G			0x13
#define 	INT_GEN_SRC_G			0x14
#define 	OUT_TEMP_L				0x15
#define 	OUT_TEMP_H				0x16
#define 	STATUS_REG				0x17
#define 	OUT_X_L_G				0x18
#define 	OUT_X_H_G				0x19
#define		OUT_Y_L_G				0x1A
#define 	OUT_Y_H_G				0x1B
#define 	OUT_Z_L_G				0x1C
#define 	OUT_Z_H_G				0x1D
#define 	CTRL_REG4				0x1E
#define 	CTRL_REG5_XL			0x1F
#define 	CTRL_REG6_XL			0x20
#define 	CTRL_REG7_XL			0x21
#define 	CTRL_REG8				0x22
#define 	CTRL_REG9				0x23
#define 	CTRL_REG10				0x24
#define 	INT_GEN_SRC_XL			0x26
#define 	OUT_X_L_XL				0x28
#define 	OUT_X_H_XL				0x29
#define 	OUT_Y_L_XL				0x2A
#define 	OUT_Y_H_XL				0x2B
#define 	OUT_Z_L_XL				0x2C
#define 	OUT_Z_H_XL				0x2D
#define 	FIFO_CTRL				0x2E
#define 	FIFO_SRC				0x2F
#define 	INT_GEN_CFG_G			0x30
#define 	INT_GEN_THS_XH_G		0x31
#define 	INT_GEN_THS_XL_G		0x32
#define 	INT_GEN_THS_YH_G		0x33
#define 	INT_GEN_THS_YL_G		0x34
#define 	INT_GEN_THS_ZH_G		0x35
#define 	INT_GEN_THS_ZL_G		0x36
#define 	INT_GEN_DUR_G			0x37

/* Register bit definitions */
#define 	CTRL_REG1_G_ODR_POS		0x05
#define 	CTRL_REG1_G_ODR			0xE0
#define 	CTRL_REG1_G_FS_POS		0x03
#define 	CTRL_REG1_G_FS			0x18

#define 	CTRL_REG3_G_LP_POS		0x07
#define 	CTRL_REG3_G_LP			0x80

#define 	CTRL_REG6_XL_ODR_POS	0x05
#define 	CTRL_REG6_XL_ODR		0xE0
#define 	CTRL_REG6_XL_FS_POS		0x03
#define 	CTRL_REG6_XL_FS			0x18

#define 	CTRL_REG8_BDU			0x40
#define 	CTRL_REG8_IF_ADD_INC	0x04

static Sensor_I2C_Init_Pair LSM9DS1_AG_reg_pairs[] = {
	{
		.reg = CTRL_REG1_G,
		// 245 dps at full scale, ODR-G = 14.9 Hz
		.value = (0x01 << CTRL_REG1_G_ODR_POS) | (0x00 << CTRL_REG1_G_FS_POS),
	},
	{
		.reg = CTRL_REG3_G,
		// Enable LP mode. HPF is OFF
		.value = CTRL_REG3_G_LP,
	},
	{
		.reg = CTRL_REG6_XL,
		// +-2g at full scale, ODR-XL = 10 Hz
		.value = 0x01 << CTRL_REG6_XL_ODR_POS,
	},
	{
		.reg = CTRL_REG8,
		// Block data update ON, IF_ADD_INC is ON, little endian data selection.
		.value = CTRL_REG8_BDU | CTRL_REG8_IF_ADD_INC,
	},
};

static Sensor_I2C_Init LSM9DS1_AG_init_array = {
	.array = LSM9DS1_AG_reg_pairs,
	.size = SIZE_OF_INIT_ARRAY(LSM9DS1_AG_reg_pairs),
	.reg_size = sizeof(uint8_t), // Byte addr
	.data_size = sizeof(uint8_t), // Byte data
};

int LSM9DS1_AG_read_regs(Sensor *sensor, void* data) {
	HAL_StatusTypeDef rc = HAL_OK;

	LSM9DS1_AG_Out_Data * out_data = (LSM9DS1_AG_Out_Data*) data;

	rc = HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						  OUT_X_L_G, sensor->data.i2c.read_addr_size,
						  (uint8_t*)out_data->gyroscope, sizeof(out_data->gyroscope), 1000);
	rc |= HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						   OUT_X_L_XL, sensor->data.i2c.read_addr_size,
						   (uint8_t*)out_data->accelerometer, sizeof(out_data->accelerometer), 1000);

	return rc;
}

static Sensor_Func_Table LSM9DS1_AG_func_table = {
	.probe = i2c_sensor_probe,
	.read = LSM9DS1_AG_read_regs,
	.init = i2c_sensor_init_regs,
};


int LSM9DS1_AG_init(Sensor *sensor, I2C_HandleTypeDef *hi2c) {
	uint8_t rc = i2c_sensor_init(sensor, "LSM9DS1_AG", &LSM9DS1_AG_func_table, hi2c, LSM9DS1_AG_DEV_ADDR << 1, &LSM9DS1_AG_init_array,
								 OUT_X_L_G, sizeof(uint8_t), sizeof(int16_t), WHO_AM_I, LSM9DS1_AG_ID_VALUE);

	if (rc == 0) {
		sensor->out_data = malloc(sizeof(LSM9DS1_AG_Out_Data));
		if (sensor->out_data == NULL) rc = 1;
	}

	return rc;
}

int LSM9DS1_AG_remove(Sensor *sensor) {
	free(sensor->out_data);

	return 0;
}
