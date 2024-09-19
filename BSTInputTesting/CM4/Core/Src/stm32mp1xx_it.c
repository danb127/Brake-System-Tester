/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32mp1xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32mp1xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile float pulse_width1, pulse_width2;
extern volatile float period1, period2;
extern volatile float duty_cycle1, duty_cycle2;
extern volatile float frequency1, frequency2;
volatile uint32_t last_rising_edge1 = 0, last_rising_edge2 = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern IPCC_HandleTypeDef hipcc;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim5;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */

  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */

  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */

  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles System service call via SWI instruction.
  */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/**
  * @brief This function handles Pendable request for system service.
  */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
  * @brief This function handles System tick timer.
  */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32MP1xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32mp1xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles TIM3 global interrupt.
  */

void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
	  if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC2) != RESET)
	  {
	    if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC2) != RESET)
	    {
	      __HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC2);

	      uint32_t current_capture = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_2);

	      if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET)
	      {
	        // Rising edge
	        if (current_capture > last_rising_edge1)
	          period1 = current_capture - last_rising_edge1;
	        else
	          period1 = (65536 + current_capture) - last_rising_edge1;

	        last_rising_edge1 = current_capture;
	      }
	      else
	      {
	        // Falling edge
	        if (current_capture > last_rising_edge1)
	          pulse_width1 = current_capture - last_rising_edge1;
	        else
	          pulse_width1 = (65536 + current_capture) - last_rising_edge1;
	      }
	    }
	  }

  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}


/**
  * @brief This function handles TIM5 global interrupt.
  */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
	  if (__HAL_TIM_GET_FLAG(&htim5, TIM_FLAG_CC2) != RESET)
	  {
		  if (__HAL_TIM_GET_IT_SOURCE(&htim5, TIM_IT_CC2) != RESET)
		  {
			  __HAL_TIM_CLEAR_IT(&htim5, TIM_IT_CC2);

			  uint32_t current_capture = HAL_TIM_ReadCapturedValue(&htim5, TIM_CHANNEL_2);

			  if (HAL_GPIO_ReadPin(GPIOH, GPIO_PIN_11) == GPIO_PIN_SET) // PA11 is the input pin for TIM5
			  {
				  // Rising edge
				  period2 = current_capture - last_rising_edge2;
				  last_rising_edge2 = current_capture;
			  }
			  else
			  {
				  // Falling edge
				  pulse_width2 = current_capture - last_rising_edge2;
			  }
		  }
	  }
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}
/**
  * @brief This function handles IPCC RX1 occupied interrupt.
  */
void IPCC_RX1_IRQHandler(void)
{
  /* USER CODE BEGIN IPCC_RX1_IRQn 0 */

  /* USER CODE END IPCC_RX1_IRQn 0 */
  HAL_IPCC_RX_IRQHandler(&hipcc);
  /* USER CODE BEGIN IPCC_RX1_IRQn 1 */

  /* USER CODE END IPCC_RX1_IRQn 1 */
}

/**
  * @brief This function handles IPCC TX1 free interrupt.
  */
void IPCC_TX1_IRQHandler(void)
{
  /* USER CODE BEGIN IPCC_TX1_IRQn 0 */

  /* USER CODE END IPCC_TX1_IRQn 0 */
  HAL_IPCC_TX_IRQHandler(&hipcc);
  /* USER CODE BEGIN IPCC_TX1_IRQn 1 */

  /* USER CODE END IPCC_TX1_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
