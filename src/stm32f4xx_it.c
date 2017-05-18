/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @author  Ac6
  * @version V1.0
  * @date    02-Feb-2015
  * @brief   Default Interrupt Service Routines.
  ******************************************************************************
*/

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "stm32f4xx.h"
#ifdef USE_RTOS_SYSTICK
#include "cmsis_os.h"
#endif
#include "stm32f4xx_it.h"

extern DMA_HandleTypeDef hdma;
extern I2C_HandleTypeDef hi2c;
extern RTC_HandleTypeDef hrtc;
extern I2S_HandleTypeDef hAudioOutI2s;
extern I2S_HandleTypeDef hAudioInI2s;

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            	  	    Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles SysTick Handler, but only if no RTOS defines it.
  * @param  None
  * @retval None
  */
void SysTick_Handler(void)
{
	HAL_IncTick();
	HAL_SYSTICK_IRQHandler();
#ifdef USE_RTOS_SYSTICK
	osSystickHandler();
#endif
}

void DMA1_Stream1_IRQHandler() {
	HAL_DMA_IRQHandler(&hdma);
}

void I2C1_EV_IRQHandler() {
	HAL_I2C_EV_IRQHandler(&hi2c);
}

void I2C1_ER_IRQHandler() {
	HAL_I2C_ER_IRQHandler(&hi2c);
}

/* Private RTC IRQ handlers */
void RTC_WKUP_IRQHandler() {
	HAL_RTCEx_WakeUpTimerIRQHandler(&hrtc);
}

void EXTI3_IRQHandler() {
	HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_3);
}

void DMA1_Stream7_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioOutI2s.hdmatx);
}

void DMA1_Stream3_IRQHandler(void)
{
  HAL_DMA_IRQHandler(hAudioInI2s.hdmarx);
}
