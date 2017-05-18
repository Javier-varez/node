/*
 * MP45DT02.c
 *
 *  Created on: May 15, 2017
 *      Author: javier
 */

#include "MP45DT02.h"
#include <string.h>
#include <stdlib.h>

#include "stm32f411e_discovery.h"
#include "stm32f411e_discovery_audio.h"

#include "semphr.h"

#include "arm_math.h"

#define	DEFAULT_PERIOD_S		2
#define MP45DT02_MAX_WAIT_TIME 	portMAX_DELAY
#define FFT_TOTAL_POINTS		32


static SemaphoreHandle_t MP45DT02_half_semaphore = NULL;
static SemaphoreHandle_t MP45DT02_cplt_semaphore = NULL;

// (128 samples = 64 samples/channel) at 32 kHz
static uint16_t InternalBuffer[INTERNAL_BUFF_SIZE];
// (32 samples/channel * 2 channels = 64 samples) at 16 kHz
static uint16_t RecBuf[PCM_OUT_SIZE * 4]; // 2 ms total sound
// single channel copy for postprocessing
static float singleChannel[PCM_OUT_SIZE * 2];
// FFT structure with butterfly coefficients already preloaded
arm_rfft_fast_instance_f32 fft_instance;

typedef struct {
	float32_t real;
	float32_t imaginary;
} complex32_t;

int MP45DT02_init_sensor(Sensor *sensor) {
	// If the semaphore hasn't been created yet
	if (MP45DT02_half_semaphore == NULL) {
		MP45DT02_half_semaphore = xSemaphoreCreateBinary();
	}
	if (MP45DT02_cplt_semaphore == NULL) {
		MP45DT02_cplt_semaphore = xSemaphoreCreateBinary();
	}

	// Init FFT structure with butterfly coefficients
	arm_rfft_fast_init_f32(&fft_instance, FFT_TOTAL_POINTS);

	return 0;
}

int MP45DT02_read(Sensor *sensor, void *data) {

	// Update priority
	vTaskPrioritySet(NULL, 6);

	BSP_AUDIO_IN_Init(DEFAULT_AUDIO_IN_FREQ, DEFAULT_AUDIO_IN_BIT_RESOLUTION, DEFAULT_AUDIO_IN_CHANNEL_NBR);
	BSP_AUDIO_IN_Record((uint16_t*)&InternalBuffer[0], INTERNAL_BUFF_SIZE);

	// Wait until processing completes
	xSemaphoreTake(MP45DT02_half_semaphore, MP45DT02_MAX_WAIT_TIME);

	BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[0], (uint16_t*)&RecBuf[0]);

	xSemaphoreTake(MP45DT02_cplt_semaphore, MP45DT02_MAX_WAIT_TIME);

	BSP_AUDIO_IN_Stop(); // Stop recording, we already have the whole buffer

	BSP_AUDIO_IN_PDMToPCM((uint16_t*)&InternalBuffer[INTERNAL_BUFF_SIZE/2], (uint16_t*)&RecBuf[PCM_OUT_SIZE*2]);

	// Restore priority
	vTaskPrioritySet(NULL, 4);

	//////// Complete processing ////////
	// Get rid of duplicate data from the other channel
	for(int i = 0; i < PCM_OUT_SIZE * 2; i++) {
		singleChannel[i-2] = (float)(((int16_t)RecBuf[i<<1]) / 65536.0 * 2.0);
	}

	complex32_t fft[FFT_TOTAL_POINTS/2];

	// Process FFT and get channels
	arm_rfft_fast_f32(&fft_instance, singleChannel, (float32_t*)fft, 0);

	// Compute modulus for each band
	MP45DT02_Out_Data *out_data = (MP45DT02_Out_Data*) sensor->out_data;

	for (int i = 0; i < MAX_BANDS; i++) {
		if (i != 0) {
			out_data->energy[i] = sqrt(pow(fft[i].real,2) + pow(fft[i].imaginary,2));
		} else {
			out_data->energy[i] = fabs(fft[i].real);
		}
	}

	return 0;
}

int MP45DT02_packData(Sensor *sensor, uint8_t *data, uint8_t len) {
	MP45DT02_Out_Data *out_data = (MP45DT02_Out_Data*) sensor->out_data;

	if (sizeof(out_data->energy) + 2 <= len) {
		data[0] = 'S';
		data++;

		memcpy(data, &out_data->energy, sizeof(out_data->energy));
		data += sizeof(out_data->energy);

		data[0] = '\0';

		return 1;
	}
	return 0;
}

static Sensor_Func_Table MP45DT02_func_table = {
	.read = MP45DT02_read,
	.init = MP45DT02_init_sensor,
	.packData = MP45DT02_packData,
};

int MP45DT02_init(Sensor *sensor) {
	sensor->func_tbl = &MP45DT02_func_table;
	sensor->interface = SI_Custom;
	sensor->sampling_period_s = DEFAULT_PERIOD_S;
	strcpy(sensor->name, "MP45DT02");

	sensor->out_data = malloc(sizeof(MP45DT02_Out_Data));
	if (sensor->out_data == NULL)
		return 1;

	return 0;
}

// Callback functions for AUDIO Processing
void BSP_AUDIO_IN_TransferComplete_CallBack(void)
{
	xSemaphoreGiveFromISR(MP45DT02_cplt_semaphore, NULL);
}

void BSP_AUDIO_IN_HalfTransfer_CallBack(void)
{
	xSemaphoreGiveFromISR(MP45DT02_half_semaphore, NULL);
}
