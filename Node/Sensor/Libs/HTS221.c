/*
 * HTS221.c
 *
 *  Created on: Apr 11, 2017
 *      Author: javier
 */

#include "HTS221.h"
#include <stdlib.h>
#include <string.h>

Sensor_I2C_Probe_Intf HTS221_intf = {
	.init = HTS221_init,
	.remove = HTS221_remove
};

#define SIZE_OF_INIT_ARRAY(X)  (sizeof(X)/sizeof(X[0]))

#define		HTS221_DEV_ADDR			0x5F
#define 	HTS221_DEV_ID			0xBC
/* Register Addresses */

#define 	WHO_AM_I				0x0F

#define 	CTRL_REG1_ADDR 			0x20
#define 	CTRL_REG2_ADDR 			0x21
#define 	CTRL_REG3_ADDR 			0x22
#define 	HUMIDITY_OUT_L_ADDR		0x28
#define 	HUMIDITY_OUT_H_ADDR		0x29
#define 	TEMP_OUT_L_ADDR			0x2A
#define 	TEMP_OUT_H_ADDR			0x2B

#define 	H0_RH_2_ADDR			0x30
#define 	H1_RH_2_ADDR			0x31
#define 	T0_C_8_ADDR				0x32
#define 	T1_C_8_ADDR				0x33
#define 	T1_T0_MSB_ADDR			0x35
#define 	H0_T0_OUT_ADDR			0x36
#define 	H1_T0_OUT_ADDR			0x3A
#define 	T0_OUT_ADDR				0x3C
#define 	T1_OUT_ADDR				0x3E

#define 	STATUS_REG				0x27

#define		AUTO_INCREMENT_ADDR		0x80

/* Bit definitions */

#define 	CTRL_REG1_PD			0x80
#define 	CTRL_REG1_PD_POS		0x07
#define 	CTRL_REG1_BDU			0x04
#define 	CTRL_REG1_BDU_POS		0x02
#define 	CTRL_REG1_ODR			0x03
#define 	CTRL_REG1_ODR_POS		0x00

#define 	CTRL_REG2_BOOT			0x80
#define 	CTRL_REG2_BOOT_POS		0x07
#define 	CTRL_REG2_HEATER		0x02
#define 	CTRL_REG2_HEATER_POS	0x01
#define 	CTRL_REG2_ONESHOT		0x01
#define 	CTRL_REG2_ONESHOT_POS	0x00

#define 	CTRL_REG3_DRDY_H_L		0x80
#define 	CTRL_REG3_DRDY_H_L_POS	0x07
#define 	CTRL_REG3_PP_OD			0x40
#define 	CTRL_REG3_PP_OD_POS		0x06
#define 	CTRL_REG3_DRDY_EN		0x04
#define 	CTRL_REG3_DRDY_EN_POS	0x02

#define 	STATUS_REG_H_DA			0x02
#define 	STATUS_REG_T_DA			0x01

static Sensor_I2C_Init_Pair HTS221_reg_pairs[] = {
	{
		.reg = CTRL_REG1_ADDR,
		.value = CTRL_REG1_PD | CTRL_REG1_BDU | (0x11 << CTRL_REG1_ODR_POS),
	},
	{
		.reg = CTRL_REG2_ADDR,
		.value = 0x00,
	},
	{
		.reg = CTRL_REG3_ADDR,
		.value = 0x00,
	},
};

static Sensor_I2C_Init HTS221_init_array = {
	.array = HTS221_reg_pairs,
	.size = SIZE_OF_INIT_ARRAY(HTS221_reg_pairs),
	.reg_size = sizeof(uint8_t), // Byte addr
	.data_size = sizeof(uint8_t), // Byte data
};

typedef struct {
	float T_A;
	float T_B;
	float H_A;
	float H_B;
} HTS221_Calibration_Data;

int HTS221_init_regs(Sensor *sensor) {
	uint32_t num = sensor->data.i2c.init->size;
	uint32_t data_size = sensor->data.i2c.init->data_size;
	uint32_t reg_size = sensor->data.i2c.init->reg_size;
	Sensor_I2C_Init_Pair *pair = sensor->data.i2c.init->array;

	HAL_StatusTypeDef rc = HAL_OK;

	for (int i = 0; i < num; i++) {
		rc = HAL_I2C_Mem_Write(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						  	   pair[i].reg, reg_size, (uint8_t*)&pair[i].value,
							   data_size, 1000);
		if (rc != HAL_OK)
			break;
	}

	// Perform calibration measurements
	uint8_t cal[16];
	rc += HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						  H0_RH_2_ADDR | AUTO_INCREMENT_ADDR, 1, cal, 16, 1000);

	float t0 = (float)(cal[T0_C_8_ADDR - H0_RH_2_ADDR] | (cal[T1_T0_MSB_ADDR - H0_RH_2_ADDR] & 0x03) << 8) / 8.0;
	float t1 = (float)(cal[T1_C_8_ADDR - H0_RH_2_ADDR] | (cal[T1_T0_MSB_ADDR - H0_RH_2_ADDR] & 0x0C) << 6) / 8.0;

	int16_t t0_out = *((int16_t*)(cal + T0_OUT_ADDR - H0_RH_2_ADDR));
	int16_t t1_out = *((int16_t*)(cal + T1_OUT_ADDR - H0_RH_2_ADDR));

	float h0 = (float)(cal[H0_RH_2_ADDR - H0_RH_2_ADDR]) / 2.0;
	float h1 = (float)(cal[H1_RH_2_ADDR - H0_RH_2_ADDR]) / 2.0;

	int16_t h0_out = *((int16_t*)(cal + H0_T0_OUT_ADDR - H0_RH_2_ADDR));
	int16_t h1_out = *((int16_t*)(cal + H1_T0_OUT_ADDR - H0_RH_2_ADDR));

	HTS221_Calibration_Data *calibration_data = (HTS221_Calibration_Data*)sensor->custom;

	calibration_data->T_A = (t1-t0)/((float) t1_out - (float) t0_out);
	calibration_data->T_B = t0 - calibration_data->T_A * (float)t0_out;

	calibration_data->H_A = (h1-h0)/((float) h1_out - (float) h0_out);
	calibration_data->H_B = h0 - calibration_data->H_A * (float)h0_out;

	return rc;
}

int HTS221_read_regs(Sensor *sensor, void* data) {
	HAL_StatusTypeDef rc = HAL_OK;

	HTS221_Calibration_Data *cal = (HTS221_Calibration_Data*)sensor->custom;
	int16_t reg_data[2];

	rc = HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						  HUMIDITY_OUT_L_ADDR | AUTO_INCREMENT_ADDR, sensor->data.i2c.read_addr_size,
						  (uint8_t*)&reg_data[0], sizeof(int16_t), 1000);
	rc |= HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						   TEMP_OUT_L_ADDR | AUTO_INCREMENT_ADDR, sensor->data.i2c.read_addr_size,
						   (uint8_t*)&reg_data[1], sizeof(int16_t), 1000);

	HTS221_Out_Data *out_data = (HTS221_Out_Data*) data;
	out_data->temperature = cal->T_A * (float)reg_data[1] + cal->T_B;
	out_data->humidity = cal->H_A * (float)reg_data[0] + cal->H_B;

	return rc;
}

int HTS221_packData(Sensor *sensor, uint8_t *data, uint8_t data_len) {
	HTS221_Out_Data *out_data = (HTS221_Out_Data*)sensor->out_data;

	if (sizeof(HTS221_Out_Data) + 3 <= data_len ) {
		data[0] = 'H';
		data++;

		memcpy(data, &out_data->humidity, sizeof(out_data->humidity));
		data += sizeof(out_data->humidity);

		data[0] = 'T';
		data++;

		memcpy(data, &out_data->temperature, sizeof(out_data->temperature));
		data += sizeof(out_data->temperature);

		data[0] = '\0';

		return 1;
	}
	return 0;
}

static Sensor_Func_Table HTS221_func_table = {
	.probe = i2c_sensor_probe,
	.read = HTS221_read_regs,
	.init = HTS221_init_regs,
	.packData = HTS221_packData,
};

int HTS221_init(Sensor *sensor, I2C_HandleTypeDef *hi2c) {
	uint8_t rc = i2c_sensor_init(sensor, "HTS221", &HTS221_func_table, hi2c, HTS221_DEV_ADDR << 1, &HTS221_init_array,
					HUMIDITY_OUT_L_ADDR | AUTO_INCREMENT_ADDR, sizeof(uint8_t), 2 * sizeof(int16_t),
					WHO_AM_I, HTS221_DEV_ID);

	sensor->custom = malloc(sizeof(HTS221_Calibration_Data));
	if (sensor->custom == NULL) rc = 1;
	else {
		sensor->out_data = malloc(sizeof(HTS221_Out_Data));
		if (sensor->custom == NULL) {
			rc = 1;
			free(sensor->custom);
		}
	}

	return rc;
}

int HTS221_remove(Sensor *sensor) {
	free(sensor->custom);
	free(sensor->out_data);

	return 0;
}
