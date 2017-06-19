/*
 * node_configuration.c
 *
 *  Created on: May 14, 2017
 *      Author: javier
 */

#include "node.h"
#include "node_configuration.h"

uint16_t VirtAddVarTab[] = {
		C_NODE_ID,
		C_S0_ID,
		C_S0_P,
		C_S1_ID,
		C_S1_P,
		C_S2_ID,
		C_S2_P,
		C_S3_ID,
		C_S3_P,
		C_S4_ID,
		C_S4_P,
		C_S5_ID,
		C_S5_P,
		C_S6_ID,
		C_S6_P,
		C_S7_ID,
		C_S7_P,
		C_S8_ID,
		C_S8_P,
		C_S9_ID,
		C_S9_P,
		C_S10_ID,
		C_S10_P,
		C_S11_ID,
		C_S11_P,
};


int Node_loadConfiguration(Node* node) {
      HAL_FLASH_Unlock();

	  if( EE_Init() != EE_OK)
	  {
		  goto exit_on_fail;
	  }

	  uint16_t data;

	  // Read Node ID
	  if((EE_ReadVariable(VirtAddVarTab[C_NODE_ID - C_NODE_ID], &data)) != HAL_OK)
		  goto exit_on_fail;

	  node->configuration.node_id = data;

	  for (uint8_t i = 0; i < (C_MAX_DATA_LEN-C_NODE_ID-1)/2; i++) {
		  uint16_t id, period, valid = 1;

		  if(EE_ReadVariable(VirtAddVarTab[C_S0_ID + 2*i - C_NODE_ID], &id) != HAL_OK)
			  valid = 0;
		  if(EE_ReadVariable(VirtAddVarTab[C_S0_P + 2*i - C_NODE_ID], &period) != HAL_OK)
			  valid = 0;

		  if (valid) {
			  node->configuration.sensor_config[i].Sensor_ID = id;
			  node->configuration.sensor_config[i].Sensor_period = period;
		  }
	  }

	  HAL_FLASH_Lock();
	  return 0;

exit_on_fail:
	  HAL_FLASH_Lock();
	  return 1;
}

int Node_storeConfiguration(Node* node, uint8_t addr) {
    HAL_FLASH_Unlock();

	if (addr == C_NODE_ID) {
		if (EE_WriteVariable(VirtAddVarTab[addr - C_NODE_ID], node->configuration.node_id) != HAL_OK)
			goto exit_on_fail;
	}
	else if (addr & 0x01) {
		// Sensor ID
		uint16_t data = node->configuration.sensor_config[(addr-C_NODE_ID-1)/2].Sensor_ID;
		if (EE_WriteVariable(VirtAddVarTab[addr - C_NODE_ID], data) != HAL_OK)
			goto exit_on_fail;
	} else {
		// Sensor period
		uint16_t data = node->configuration.sensor_config[(addr-C_NODE_ID-1)/2].Sensor_period;
		if (EE_WriteVariable(VirtAddVarTab[addr - C_NODE_ID], data) != HAL_OK)
			goto exit_on_fail;
	}

	HAL_FLASH_Lock();
	return 0;

exit_on_fail:
	HAL_FLASH_Lock();
	return 1;
}
void Node_Configuration_setPeriod(Node *node, uint8_t sensor_ID, uint16_t sampling_period){
	for(int i = 0; i < (C_MAX_DATA_LEN-C_NODE_ID-1)/2; i++){
		if(node->configuration.sensor_config[i].Sensor_ID == sensor_ID){
			node->configuration.sensor_config[i].Sensor_period = sampling_period;
		}
	}
}

