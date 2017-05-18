/*
 * LPS25H.c
 *
 *  Created on: Apr 12, 2017
 *      Author: javier
 */

#include "LPS25H.h"
#include <stdlib.h>
#include <string.h>

Sensor_I2C_Probe_Intf LPS25H_intf = {
	.init = LPS25H_init,
	.remove = LPS25H_remove
};

#define SIZE_OF_INIT_ARRAY(X)  (sizeof(X)/sizeof(X[0]))

#define		LPS25H_DEV_ADDR			0x5C
#define		LPS25H_ID_VALUE			0xBD
#define		AUTO_INCREMENT_ADDR		0x80

#define		DEFAULT_PERIOD_S		10
/* Register Addresses */

#define 	REF_P_XL				0x08
#define		REF_P_L					0x09
#define 	REF_P_H					0x0A
#define 	WHO_AM_I				0x0F
#define		RES_CONF				0x10
#define		CTRL_REG1				0x20
#define 	CTRL_REG2				0x21
#define 	CTRL_REG3				0x22
#define 	CTRL_REG4				0x23
#define 	INT_CFG					0x24
#define 	INT_SOURCE				0x25
#define		STATUS_REG				0x27
#define 	PRESS_OUT_XL			0x28
#define 	PRESS_OUT_L				0x29
#define 	PRESS_OUT_H				0x2A
#define 	TEMP_OUT_L				0x2B
#define 	TEMP_OUT_H				0x2C
#define 	FIFO_CTRL				0x2E
#define 	FIFO_STATUS				0x2F
#define 	THS_P_L					0x30
#define 	THS_P_H					0x31
#define		RPDS_L					0x39
#define 	RPDS_H					0x3A

/* Register bit definitions */

#define 	CTRL_REG1_PD_POS		0x07
#define 	CTRL_REG1_PD			0x80
#define 	CTRL_REG1_ODR_POS		0x04
#define		CTRL_REG1_ODR			0x70
#define		CTRL_REG1_DIFF_EN_POS	0x03
#define 	CTRL_REG1_DIFF_EN		0x08
#define		CTRL_REG1_BDU_POS		0x02
#define 	CTRL_REG1_BDU			0x04
#define		CTRL_REG1_RESET_AZ_POS	0x01
#define 	CTRL_REG1_RESET_AZ		0x02
#define		CTRL_REG1_SIM_POS		0x00
#define 	CTRL_REG1_SIM			0x01

#define 	CTRL_REG2_BOOT_POS				0x07
#define 	CTRL_REG2_BOOT					0x80
#define 	CTRL_REG2_FIFO_EN_POS			0x06
#define 	CTRL_REG2_FIFO_EN				0x40
#define 	CTRL_REG2_WTM_EN_POS			0x05
#define 	CTRL_REG2_WTM_EN				0x20
#define 	CTRL_REG2_FIFO_MEAN_DEC_POS		0x04
#define 	CTRL_REG2_FIFO_MEAN_DEC			0x10
#define		CTRL_REG2_SWRESET_POS			0x02
#define		CTRL_REG2_SWRESET				0x04
#define		CTRL_REG2_AUTO_ZERO_POS			0x01
#define		CTRL_REG2_AUTO_ZERO				0x02
#define		CTRL_REG2_ONE_SHOT_POS			0x00
#define		CTRL_REG2_ONE_SHOT				0x01

#define 	CTRL_REG3_INT_H_L_POS			0x07
#define 	CTRL_REG3_INT_H_L				0x80
#define 	CTRL_REG3_PP_OD_POS				0x06
#define 	CTRL_REG3_PP_OD					0x40
#define 	CTRL_REG3_INT1_S_POS			0x00
#define 	CTRL_REG3_INT1_S				0x03

#define 	CTRL_REG4_P1_EMPTY_POS			0x03
#define 	CTRL_REG4_P1_EMPTY				0x08
#define 	CTRL_REG4_P1_WTM_POS			0x02
#define 	CTRL_REG4_P1_WTM				0x04
#define 	CTRL_REG4_P1_OVERRUN_POS		0x01
#define 	CTRL_REG4_P1_OVERRUN			0x02
#define 	CTRL_REG4_P1_DRDY_POS			0x00
#define 	CTRL_REG4_P1_DRDY				0x01

#define 	FIFO_CTRL_FIFO_MODE_POS			0x05
#define		FIFO_CTRL_FIFO_MODE				0xE0
#define		FIFO_CTRL_WTM_POINT_POS			0x00
#define 	FIFO_CTRL_WTM_POINT				0x1F

static Sensor_I2C_Init_Pair LPS25H_reg_pairs[] = {
	{
		.reg = CTRL_REG1,
		.value = CTRL_REG1_PD | (0x04 << CTRL_REG1_ODR_POS) | CTRL_REG1_BDU,
	},
	{
		// Set number of internal averages
		.reg = RES_CONF,
		.value = 0x05,
	},
	{
		// FIFO set in FIFO_MEAN_MODE with 4 samples averaged
		.reg = FIFO_CTRL,
		.value = (0x06 << FIFO_CTRL_FIFO_MODE_POS) | (0x01 << FIFO_CTRL_WTM_POINT_POS),
	},
	{
		.reg = CTRL_REG2,
		.value = CTRL_REG2_FIFO_EN,
	},
};

static Sensor_I2C_Init LPS25H_init_array = {
	.array = LPS25H_reg_pairs,
	.size = SIZE_OF_INIT_ARRAY(LPS25H_reg_pairs),
	.reg_size = sizeof(uint8_t), // Byte addr
	.data_size = sizeof(uint8_t), // Byte data
};

int LPS25H_read_regs(Sensor *sensor, void* data) {
	HAL_StatusTypeDef rc = HAL_OK;

	uint8_t reg_press[3];
	int16_t reg_temp;

	rc = HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						  PRESS_OUT_XL | AUTO_INCREMENT_ADDR, sensor->data.i2c.read_addr_size,
						  reg_press, sizeof(reg_press), 1000);
	rc |= HAL_I2C_Mem_Read(sensor->data.i2c.hi2c, sensor->data.i2c.dev_addr,
						   TEMP_OUT_L | AUTO_INCREMENT_ADDR, sensor->data.i2c.read_addr_size,
						   (uint8_t*)&reg_temp, sizeof(reg_temp), 1000);

	LPS25H_Out_Data *out_data = (LPS25H_Out_Data*) data;
	out_data->pressure = (float)((reg_press[2] << 16) | (reg_press[1] << 8) | reg_press[0]) / 4096.0;
	out_data->temperature = (float)(reg_temp) / 480.0 + 42.5;

	return rc;
}

int LPS25H_packData(Sensor *sensor, uint8_t *data, uint8_t data_len) {
	LPS25H_Out_Data *out_data = (LPS25H_Out_Data*)sensor->out_data;

	if (sizeof(HTS221_Out_Data) + 3 <= data_len ) {
		data[0] = 'P';
		data++;

		memcpy(data, &out_data->pressure, sizeof(out_data->pressure));
		data += sizeof(out_data->pressure);

		data[0] = 'T';
		data++;

		memcpy(data, &out_data->temperature, sizeof(out_data->temperature));
		data += sizeof(out_data->temperature);

		data[0] = '\0';

		return 1;
	}
	return 0;
}

static Sensor_Func_Table LPS25H_func_table = {
	.probe = i2c_sensor_probe,
	.read = LPS25H_read_regs,
	.init = i2c_sensor_init_regs,
	.packData = LPS25H_packData
};

int LPS25H_init(Sensor *sensor, I2C_HandleTypeDef *hi2c, uint16_t sampling_period_s) {
	uint8_t rc = i2c_sensor_init(sensor, "LPS25H", LPS25H_ID, &LPS25H_func_table, hi2c, LPS25H_DEV_ADDR << 1, &LPS25H_init_array,
								 PRESS_OUT_XL | AUTO_INCREMENT_ADDR, sizeof(uint8_t), sizeof(LPS25H_Out_Data),
								 WHO_AM_I, LPS25H_ID_VALUE, DEFAULT_PERIOD_S | sampling_period_s);
	if (rc == 0) {
		sensor->out_data = malloc(sizeof(LPS25H_Out_Data));
		if (sensor->out_data == 0) rc = 1;
	}

	return rc;
}

int LPS25H_remove(Sensor *sensor) {
	free(sensor->out_data);

	return 0;
}
