/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "openamp.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* USER CODE BEGIN Includes */
#include "openamp.h"
#include "openamp_log.h"
#include <float.h>
#include <math.h>
#include <string.h>
#include <stdint.h>
#include "virt_uart.h" // Or whatever the correct header file is for virtual UART

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
    float stroke;
    float duty_cycle1_min;
    float duty_cycle1_max;
    float duty_cycle2_min;
    float duty_cycle2_max;
} BSTTestPoint;



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE


#define LOGLEVEL LOGINFO
#define __LOG_UART_IO_
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

IPCC_HandleTypeDef hipcc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */

// Virtual UART handle for RPMsg communication with the remote core
VIRT_UART_HandleTypeDef huart0;

// start should be set once communication with server is established
static uint8_t start = 0;
// result: default at 0, -1 if fail, 1 if pass
static int8_t result = 0;


volatile float pulse_width1 = 0;
volatile float pulse_width2 = 0;
volatile float period1 = 0;
volatile float period2 = 0;
volatile float duty_cycle1 = 0;
volatile float duty_cycle2 = 0;
volatile float frequency1 = 0;
volatile float frequency2 = 0;
volatile float stroke = 0;
volatile float estimated_stroke = 0;
int case1 = 0;
int case2 = 0;
int test_passed = 0;

uint8_t use_stringpot = 0;

// 0 mm stroke, 12.5% +- 3.5% and 87.5% +-3.5 %
// 2 mm stroke, 24.4% +- 3.5% and 75.7% +- 3.5%
// 4 mm stroke, 36.3% +- 3.5% and 63.7% +- 3.5%
// 6 mm stroke, 48.3% +- 3.5% and 51.7% +- 3.5%
// 8 mm stroke, 63.2% +- 3.5% and 36.8% +- 3.5%

const BSTTestPoint bst_test_points[] = {
    {0.0f, 9.0f, 16.0f, 84.0f, 91.0f},
    {2.0f, 20.9f, 27.9f, 72.2f, 79.2f},
    {4.0f, 32.8f, 39.8f, 60.2f, 67.2f},
    {6.0f, 44.8f, 51.8f, 48.2f, 55.2f},
    {8.0f, 59.7f, 66.7f, 33.3f, 40.3f}
};

// Flag to indicate that a message has been received from the A7 core
__IO FlagStatus VirtUart0RxMsg = RESET;
// Buffer used for reception from the A7 core
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
// Size of the received message
uint16_t VirtUart0ChannelRxSize = 0;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
static void MX_IPCC_Init(void);
static void MX_ADC1_Init(void);
int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);
/* USER CODE BEGIN PFP */
// Callback function for the virtual UART reception from the A7 core
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);

// Function to perform the BST test with string potentiometer
int check_bst_values(float stroke, float duty_cycle1, float duty_cycle2);

// Function to calculate estimated stroke if no string potentiometer is selected
float estimated_stroke_from_duty_cycles(float duty_cycle1, float duty_cycle2);

// Function to read stroke from ADC
float read_stroke_from_adc(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the system clock */
    SystemClock_Config();
  }

  if(IS_ENGINEERING_BOOT_MODE())
  {
    /* Configure the peripherals common clocks */
    PeriphCommonClock_Config();
  }
  else
  {
    /* IPCC initialisation */
    MX_IPCC_Init();
    /* OpenAmp initialisation ---------------------------------*/
    MX_OPENAMP_Init(RPMSG_REMOTE, NULL);
  }

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  MX_ADC1_Init();
  /* USER CODE BEGIN 2 */
  // Start Timer 3 Channel 2 for PWM Output 1
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // For PWM1 (S1)

  // Start Timer 5 Channel 2 for PWM Output 2
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2); // For PWM2 (S2)

  /*
    * Create Virtual UART device
    * defined by a rpmsg channel attached to the remote device
    */
   /*Need to register callback for message reception by channels*/
   if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
   {
    Error_Handler();
   }


  //csv header
  printf("time(s),duty_cycle1(%%),duty_cycle2(%%),stroke(mm)\r\n");

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  OPENAMP_check_for_message();

	  if (VirtUart0RxMsg) {
		  VirtUart0RxMsg = RESET;
		  VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffRx, VirtUart0ChannelRxSize);
	  }

	    // Calculating the frequency and duty cycle for both channels
	    frequency1 = (period1 > 0) ? (((float) SystemCoreClock) / ((htim3.Init.Prescaler + 1) * period1)) : 0;
	    frequency2 = (period2 > 0) ? (((float) SystemCoreClock) / ((htim5.Init.Prescaler + 1) * period2)) : 0;

	    duty_cycle1 = (period1 > 0) ? (pulse_width1 / period1) * 100.0f : 0;
	    duty_cycle2 = (period2 > 0) ? (pulse_width2 / period2) * 100.0f : 0;

	    if (use_stringpot)
	    {
	    	// Using String Potentiometer

	    	// Read the stroke value from ADC
	    	stroke = read_stroke_from_adc();

	    	// Check if the duty cycles are acceptable for the measured stroke
	    	test_passed = check_bst_values(stroke, duty_cycle1, duty_cycle2);

	    	if (test_passed)
	    	{
	    		// Test Passed
	    		log_info("BST Test Passed for Stroke: %2f mm\n Duty Cycles were: %.2f and %.2f\n", stroke, duty_cycle1, duty_cycle2);
	    	}
	    	else
	    	{
	    		// Test Failed
	    		log_info("BST Test Failed for Stroke: %.2f mm\n Duty Cycles were: %.2f and %.2f\n", stroke, duty_cycle1, duty_cycle2);
	    	}
	    }
	    // If string potentiometer is not being used
	    else
	    {
	    	// Not Using String Potentiometer


	    	estimated_stroke = estimated_stroke_from_duty_cycles(duty_cycle1, duty_cycle2);


	    	test_passed = check_bst_values(estimated_stroke, duty_cycle1, duty_cycle2);
	    	log_info("Estimated Stroke is: %.2f mm\n Duty Cycles were: %.2f and %.2f", estimated_stroke, duty_cycle1, duty_cycle2);
	    }

        OPENAMP_check_for_message();


    if(start) 
    {
      // Calculating the frequency and duty cycle for both channels
      frequency1 = (period1 > 0) ? (((float) SystemCoreClock) / ((htim3.Init.Prescaler + 1) * period1)) : 0;
      frequency2 = (period2 > 0) ? (((float) SystemCoreClock) / ((htim5.Init.Prescaler + 1) * period2)) : 0;

      duty_cycle1 = (period1 > 0) ? (pulse_width1 / period1) * 100.0f : 0;
      duty_cycle2 = (period2 > 0) ? (pulse_width2 / period2) * 100.0f : 0;


      if (use_stringpot)
      {
        // Using String Potentiometer

        // Read the stroke value from ADC
        stroke = read_stroke_from_adc();
        // duty_cycle1,duty_cycle2,stroke
        log_info("%f,%f,%f\r\n",duty_cycle1,duty_cycle2,stroke);

        // Check if the duty cycles are acceptable for the measured stroke
        result = check_bst_values(stroke, duty_cycle1, duty_cycle2);

      }
      // TODO: Implement test condition for No String potentiometer case
      // If string potentiometer is not being used
      else
      {
        // Not Using String Potentiometer
        float estimated_stroke = estimated_stroke_from_duty_cycles(duty_cycle1, duty_cycle2);
        // duty_cycle1,duty_cycle2,estimated_stroke
        log_info("%f,%f,%f\r\n",duty_cycle1,duty_cycle2,estimated_stroke);
      }
    }

    if (VirtUart0RxMsg) {
      VirtUart0RxMsg = RESET;
      VIRT_UART_Transmit(&huart0, VirtUart0ChannelBuffRx, VirtUart0ChannelRxSize);
    }


    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSE;
  RCC_OscInitStruct.PLL2.PLLM = 3;
  RCC_OscInitStruct.PLL2.PLLN = 66;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 5120;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSI;
  RCC_OscInitStruct.PLL3.PLLM = 4;
  RCC_OscInitStruct.PLL3.PLLN = 28;
  RCC_OscInitStruct.PLL3.PLLP = 5;
  RCC_OscInitStruct.PLL3.PLLQ = 2;
  RCC_OscInitStruct.PLL3.PLLR = 37;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 1024;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** RCC Clock Config
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_ACLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3|RCC_CLOCKTYPE_PCLK4
                              |RCC_CLOCKTYPE_PCLK5;
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_PLL2;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV2;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV4;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(24);
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the common periph clock
  */
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_CKPER;
  PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_HSE;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc1.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc1.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief IPCC Initialization Function
  * @param None
  * @retval None
  */
static void MX_IPCC_Init(void)
{

  /* USER CODE BEGIN IPCC_Init 0 */

  /* USER CODE END IPCC_Init 0 */

  /* USER CODE BEGIN IPCC_Init 1 */

  /* USER CODE END IPCC_Init 1 */
  hipcc.Instance = IPCC;
  if (HAL_IPCC_Init(&hipcc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IPCC_Init 2 */

  /* USER CODE END IPCC_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 13;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 65535;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim3, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_IC_InitTypeDef sConfigIC = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
  sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
  sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
  sConfigIC.ICFilter = 0;
  if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

// Callback function for the virtual UART reception from the A7 core
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{

	const char* server_message = (char*) huart->pRxBuffPtr;
  const size_t server_message_len = huart->RxXferSize;
	char response_buffer[64] = "ERROR\n";

	if (strncmp(server_message,"hello\n",server_message_len) == 0)
	{
		char response[] = "BST Test Acknowledged\n";
    strcpy(response_buffer,response);
	}
  // start test once string potentiometer is used or not
	else if(strncmp(server_message,"no\n",server_message_len) == 0)
	{
		char response[] = "Testing with no String Potentiometer\n";
    strcpy(response_buffer,response);
    start = 1;
	}
  else if(strncmp(server_message,"yes\n",server_message_len) == 0)
  {
    char response[] = "Testing with String Potentiometer\n";
    strcpy(response_buffer,response_buffer);
    // set use_stringpot
    use_stringpot = 1;
    start = 1;
  }
  else if(strncmp(server_message,"ping\n",server_message_len) == 0)
  {
    (result == 0)? strcpy(response_buffer,"Testing...\n"):
      (result == -1)? strcpy(response_buffer,"Fail\n"):
      strcpy(response_buffer,"Pass\n");
  }
  else
  {
    strcpy(response_buffer,"Error\n");
  }


    /* Copy received message into buffer */
    // Limit the size of the received message to the maximum buffer size or the size of the message
    VirtUart0ChannelRxSize = strlen(response_buffer) < MAX_BUFFER_SIZE ? strlen(response_buffer) : MAX_BUFFER_SIZE - 1;
    // Copy the received message into the buffer
    memcpy(VirtUart0ChannelBuffRx, response_buffer, VirtUart0ChannelRxSize);
    // Set the flag to indicate that a message has been received
    VirtUart0RxMsg = SET;
}



//TODO: this only can return 
// Checking the BST values

int check_bst_values(float estimated_stroke, float duty_cycle1, float duty_cycle2)

//uint8_t check_bst_values(float stroke, float duty_cycle1, float duty_cycle2)

{
	// number of test points = 5 (25 / 5)
    int num_points = sizeof(bst_test_points) / sizeof(BSTTestPoint);

    // Check if stroke is less then the first stroke or more than the last stroke (out of range)
    if (stroke < bst_test_points[0].stroke || stroke > bst_test_points[num_points - 1].stroke)
    {
    	return 0; // Stroke out of range
    }

    // Initialize pointers to find the nearest two test points
    const BSTTestPoint *lower_point = NULL;
    const BSTTestPoint *upper_point = NULL;

    // For loop for the test points where the stroke might lie in between
	for (int i = 0; i < num_points; i++)
    {
		if (stroke >= bst_test_points[i].stroke && stroke <= bst_test_points[i + 1].stroke)
		{
			lower_point = &bst_test_points[i];
			upper_point = &bst_test_points[i + 1];
			break;
		}
    }

	// If stroke matches test point exactly, lower and upper point are set to that test point
	if (lower_point == NULL || upper_point == NULL)
	{
		// If the absolute difference of the stroke and the first test point is less than 0.1
		if (fabsf(stroke - bst_test_points[0].stroke) < 0.1f)
		{
			// Set both the lower and upper point to 0mm test point
			lower_point = &bst_test_points[0];
			upper_point = &bst_test_points[0];
		}
		// If the absolute difference of the stroke and the test point number is less than 0.1
		else if (fabsf(stroke - bst_test_points[num_points - 1].stroke) < 0.1f)
		{
			// Set both upper and lower point to the same mm test point
			lower_point = &bst_test_points[num_points - 1];
			upper_point = &bst_test_points[num_points - 1];
		}
		else
		{
			return 0; // Stroke not within test point range
		}
	}

	// Calculating interpolation ratio to calculate expected duty cycles
	float ratio = (stroke - lower_point->stroke) / (upper_point->stroke - lower_point->stroke);

	// Getting both pwm channel expected duty cycles with a 3.5% tolerance.
	// Keeping in mind lower_point and upper_point are pointers to the nearest test points surround measured stroke
	// Variable ratio will give us fractional position between the two test points
	// This formula will calculate a value that lies between the lower and upper proportional to the ratio
	// Starting with minimum or maximum duty cucle at lower test point
	// Then adding the proportial difference between lower and upper duty cycle min or max, scaled by ratio
	float expected_duty_cycle1_min = lower_point->duty_cycle1_min + ratio * (upper_point->duty_cycle1_min - lower_point->duty_cycle1_min);
	float expected_duty_cycle1_max = lower_point->duty_cycle1_max + ratio * (upper_point->duty_cycle1_max - lower_point->duty_cycle1_max);
	float expected_duty_cycle2_min = lower_point->duty_cycle2_min + ratio * (upper_point->duty_cycle2_min - lower_point->duty_cycle2_min);
	float expected_duty_cycle2_max = lower_point->duty_cycle2_max + ratio * (upper_point->duty_cycle2_max - lower_point->duty_cycle2_max);


	// Allow tolerance of 3.5% according to BST product specs
	float tolerance = 3.5f;

	// Adjust interpolated ranges by tolerance
	expected_duty_cycle1_min -= tolerance;
	expected_duty_cycle1_max += tolerance;
	expected_duty_cycle2_min -= tolerance;
	expected_duty_cycle2_max += tolerance;

	// Check if measured duty cycles fit within ranges in either order
    case1 = (duty_cycle1 >= expected_duty_cycle1_min &&
                 duty_cycle1 <= expected_duty_cycle1_max &&
                 duty_cycle2 >= expected_duty_cycle2_min &&
                 duty_cycle2 <= expected_duty_cycle2_max);

    case2 = (duty_cycle1 >= expected_duty_cycle2_min &&
                 duty_cycle1 <= expected_duty_cycle2_max &&
                 duty_cycle2 >= expected_duty_cycle1_min &&
                 duty_cycle2 <= expected_duty_cycle1_max);

    // Check if either case passed to send back a test passed or failed to A7 Cortex
    if (case1 || case2)
    {
    	return 1; // passed
    }
    else
    {
    	return 0; // failed
    }
}

// Estimate stroke from the given duty cycles if no string potentiometer
float estimated_stroke_from_duty_cycles(float duty_cycle1, float duty_cycle2)
{
	// Getting the number of test points (5) = 25 / 5
	int num_points = sizeof(bst_test_points) / sizeof(BSTTestPoint);

	// Initializing the minimum error to the maximum floating point
	float min_error = FLT_MAX;

	// Initializing the stroke to 0
	estimated_stroke = 0.0f;

	// Iterate through the 5 test points
	for(int i = 0; i < num_points; i++)
	{
		// Average of the duty cycle 1 max and min for test point i
		float avg_duty_cycle1 = (bst_test_points[i].duty_cycle1_min + bst_test_points[i].duty_cycle1_max) / 2.0f;
		// Average of the duty cycle 2 max and min for test point i
		float avg_duty_cycle2 = (bst_test_points[i].duty_cycle2_min + bst_test_points[i].duty_cycle2_max) / 2.0f;


		// Calculate sum of absolute differences between measured duty cycles and their average expected duty cycles
		float error_case1 = fabsf(duty_cycle1 - avg_duty_cycle1) + fabsf(duty_cycle2 - avg_duty_cycle2);
		// Calculate sum of absolute differences in case the measured duty cycle pins are swapped from the expected average duty cycles
		float error_case2 = fabsf(duty_cycle1 - avg_duty_cycle2) + fabsf(duty_cycle2 - avg_duty_cycle1);

		// Choosing the smaller error of the 2 cases to ensure best stroke correlation (smallest error), regardless or duty cycle order
		float total_error = (error_case1 < error_case2) ? error_case1 : error_case2;

		// Updating the estimcheck_bst_valuesated stroke if lower total error is found within test points
		if (total_error < min_error)
		{
			min_error = total_error;
			estimated_stroke = bst_test_points[i].stroke;
		}
	}

	return estimated_stroke;
}


float read_stroke_from_adc(void)
{
	// Start ADC conversion
	HAL_ADC_Start(&hadc1);
	// Wait for conversion to complete
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	// Get ADC value
	uint32_t adc_value = HAL_ADC_GetValue(&hadc1);
	// Stop ADC
	HAL_ADC_Stop(&hadc1);

	// ADC parameters

	// For 12-bit ADC res
	uint32_t ADC_max_value = 4095;

	// Reference voltage
	float V_ref = 3.3f;

	// Maximum Sensor Voltage
	float V_sensor_max = 10.0f;

	float V_sensor_min = 0.0f; // Adjust this value as needed

	// Maximum Stroke in mm
	float Stroke_max = 1270.0f;

	// Voltage divider scaling factor
	float scaling_factor = (8.5f + 3.3f) / 3.3f; //(R1 + R2) / R2

	// Calculating ADC input
	float V_adc = ((float)adc_value / (float)ADC_max_value) * V_ref;

	// Calculating acutal sensor voltage before voltage divider
	float V_sensor = V_adc * scaling_factor;


    // Calculate stroke in mm
    float stroke = ((V_sensor - V_sensor_min) / (V_sensor_max - V_sensor_min)) * Stroke_max;

    return stroke;

}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
