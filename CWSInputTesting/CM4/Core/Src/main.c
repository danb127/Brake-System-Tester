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
#include "virt_uart.h"
#include <string.h>
#include <stdio.h>
#include "openamp_log.h"
#include <float.h>
#include <math.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef struct {
  uint8_t passed;
  uint8_t tested;
}test_point;

typedef struct {
  size_t done;
  test_point* tests;
}CWS_Test;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE
#define LOGLEVEL LOGINFO
#define __LOG_UART_IO_
`
#define VOLTAGE_DIVIDER_RATIO 2.0f  // 3k/3k divider

// CWS specifications
#define VOLTAGE_MIN 0.7f      // Minimum voltage at 18.5mm (new pad)
#define VOLTAGE_MAX 3.5f      // Maximum voltage for linear voltage scaling range
#define VOLTAGE_RANGE 2.8f    // VOLTAGE_MAX - VOLTAGE_MIN
#define WEAR_MIN 18.5f        // Minimum wear (new pad)
#define WEAR_MAX 53.5f        // Maximum wear (wear limit)
#define WEAR_RANGE 35.0f      // WEAR_MAX - WEAR_MIN
#define MAX_POINTS 7          // 5mm per test point (35mm / 7)
/* USER CODE END PD */
/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IPCC_HandleTypeDef hipcc;

TIM_HandleTypeDef htim4;

/* USER CODE BEGIN PV */
// Virtual UART handle for RPMsg communication with the remote core
VIRT_UART_HandleTypeDef huart0;

__IO FlagStatus VirtUart0RxMsg = RESET;
uint8_t VirtUart0ChannelBuffRx[MAX_BUFFER_SIZE];
uint16_t VirtUart0ChannelRxSize = 0;

int cws_test_active = 0;
float wear = 0.0f;
volatile float V_sensor = 0.0f;
volatile int test_status = 0; // 0= fail, 1 = pass
int test_passed = 0;

// ADC DMA Buffer
volatile uint32_t adc_buffer[ADC_BUFFER_SIZE];

// ADC Parameters
const float V_ref = 3.146f;
const float divider_factor = 2.0f;

// Test Variables
test_point tests[WEAR_MIN];

CWS_Test cws_test;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_IPCC_Init(void);
static void MX_TIM4_Init(void);
int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);
/* USER CODE BEGIN PFP */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);
float get_expected_wear(float voltage);
int test_cws_functionality(float voltage, float wear);
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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
  {
      Error_Handler();
  }

  // Small delay after calibration (as per documentation)
  HAL_Delay(10);
  // Start conversion with DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);

  // Start timer to trigger ADC
  HAL_TIM_Base_Start(&htim3);

  if (VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
	 Error_Handler();
  }

  /* Register callback for message reception */
  if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
  {
	 Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  // Not Using String Potentiometer
	  float estimated_stroke = estimated_stroke_from_duty_cycles(duty_cycle1, duty_cycle2);
	  // duty_cycle1,duty_cycle2,estimated_stroke
	  int w = (int)(100* wear);
	  int v = (int)(100* V_sensor);
	  //        Wear x Voltage Output
	  log_info("%02d.%02d\r\n",w,v);

	  test_cws_functionality(V_sensor, wear);

	  if(cws_test.done == MAX_DISTANCE) {
		start = 0;
		evaluate_test_result();

	  }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSI;
  RCC_OscInitStruct.PLL3.PLLM = 4;
  RCC_OscInitStruct.PLL3.PLLN = 28;
  RCC_OscInitStruct.PLL3.PLLP = 5;
  RCC_OscInitStruct.PLL3.PLLQ = 2;
  RCC_OscInitStruct.PLL3.PLLR = 2;
  RCC_OscInitStruct.PLL3.PLLRGE = RCC_PLL3IFRANGE_1;
  RCC_OscInitStruct.PLL3.PLLFRACV = 1024;
  RCC_OscInitStruct.PLL3.PLLMODE = RCC_PLL_FRACTIONAL;
  RCC_OscInitStruct.PLL4.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL4.PLLSource = RCC_PLL4SOURCE_HSI;
  RCC_OscInitStruct.PLL4.PLLM = 4;
  RCC_OscInitStruct.PLL4.PLLN = 25;
  RCC_OscInitStruct.PLL4.PLLP = 2;
  RCC_OscInitStruct.PLL4.PLLQ = 2;
  RCC_OscInitStruct.PLL4.PLLR = 4;
  RCC_OscInitStruct.PLL4.PLLRGE = RCC_PLL4IFRANGE_1;
  RCC_OscInitStruct.PLL4.PLLFRACV = 0;
  RCC_OscInitStruct.PLL4.PLLMODE = RCC_PLL_INTEGER;
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
  RCC_ClkInitStruct.AXISSInit.AXI_Clock = RCC_AXISSOURCE_HSI;
  RCC_ClkInitStruct.AXISSInit.AXI_Div = RCC_AXI_DIV1;
  RCC_ClkInitStruct.MCUInit.MCU_Clock = RCC_MCUSSOURCE_PLL3;
  RCC_ClkInitStruct.MCUInit.MCU_Div = RCC_MCU_DIV1;
  RCC_ClkInitStruct.APB4_Div = RCC_APB4_DIV1;
  RCC_ClkInitStruct.APB5_Div = RCC_APB5_DIV1;
  RCC_ClkInitStruct.APB1_Div = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2_Div = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB3_Div = RCC_APB3_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Set the HSE division factor for RTC clock
  */
  __HAL_RCC_RTC_HSEDIV(1);
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
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_EXTERNALTRIG_T4_TRGO;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_RISING;
  hadc1.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DMA_CIRCULAR;
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
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_810CYCLES_5;
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
  est_point tests[MAX_DISTANCE];


  BST_Test bst_test;
}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 89;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 999;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_ENABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 1, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

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
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart)
{
  char* received_msg = (char*)huart->pRxBuffPtr;

  if (strcmp(received_msg, "START_CWS_TEST") == 0) {
    cws_test_active = 1;
    char* response = "CWS_TEST_STARTED";
    VIRT_UART_Transmit(&huart0, (uint8_t*)response, strlen(response));
  }
  else if (strcmp(received_msg, "STOP_CWS_TEST") == 0) {
    cws_test_active = 0;
    char* response = "CWS_TEST_STOPPED";
    VIRT_UART_Transmit(&huart0, (uint8_t*)response, strlen(response));
  }
  else {
    char* response = "UNKNOWN_COMMAND";
    VIRT_UART_Transmit(&huart0, (uint8_t*)response, strlen(response));
  }

  VirtUart0RxMsg = SET;
}

// Function to calculate expected voltage for any wear value
float get_expected_wear(float voltage)
{
    // Ensure voltage is within bounds
    //if(voltage < VOLTAGE_MIN) voltage = VOLTAGE_MIN;
    //if(voltage > VOLTAGE_MAX) voltage = VOLTAGE_MAX;

    //wear = wear_min + (v - v_min)/(v_max - v_min) * wear_range
    return WEAR_MIN + ((voltage - VOLTAGE_MIN) / VOLTAGE_RANGE) * WEAR_RANGE;
}

int test_cws_functionality(float voltage, float wear) {
    int test_passed = 1; // Assuming test passes initially

	// Checking for internal sensor failure or loss of GND
    if(voltage < 0.5f || voltage > 4.5f){
    	test_passed = -1;
    }
    else if (voltage >= 0.7f && voltage <= 4.2f){
    	float expected_wear = get_expected_wear(voltage);
    	if (fabsf(wear - expected_wear) > 0.5f){
    		test_passed = -1; // signaling fail
    	}
    }
    else{
    	test_passed = -1; // fail
    }

    // Update test_point
	float wear_percentage = (wear - WEAR_MIN) / WEAR_RANGE;
	int wear_index = (int)(wear_percentage * (MAX_POINTS - 1) + 0.5f);

	// Ensure index is in range
	if (wear_index < 0) wear_index = 0;
	if (wear_index >= MAX_POINTS) wear_index = MAX_POINTS - 1;

	if (cws_test.tests[wear_index].tested == 0)
	{
		cws_test.tests[wear_index].tested = 1;
		cws_test.tests[wear_index].passed = test_passed;
		cws_test.done++;
	}

	return test_passed;


}


void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
    static int sum = 0;
    static int count = 0;

    for(int i = ADC_BUFFER_SIZE/2; i < ADC_BUFFER_SIZE; i++) {
        sum += adc_buffer[i];
        count++;

        if(count >= ADC_BUFFER_SIZE) {
            uint32_t average = sum / ADC_BUFFER_SIZE;
            V_sensor = ((float)average * V_ref / 4095.0f) * VOLTAGE_DIVIDER_RATIO;

            // Calculate wear from voltage
            if(V_sensor < VOLTAGE_MIN)
            {
            	wear = WEAR_MIN;
            }
            else if(V_sensor > VOLTAGE_MAX)
            {
            	wear = WEAR_MAX;
            }
            else
            {
            	wear = get_expected_wear(V_sensor);
            }

            // Test the sensor
            test_status = test_cws_functionality(V_sensor, wear);

            sum = 0;
            count = 0;
        }
    }
}

// Half transfer callback
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{
    static int sum = 0;
    static int count = 0;

    for(int i = 0; i < ADC_BUFFER_SIZE/2; i++) {
        sum += adc_buffer[i];
        count++;

        if(count >= ADC_BUFFER_SIZE) {
            uint32_t average = sum / ADC_BUFFER_SIZE;
            V_sensor = ((float)average * V_ref / 4095.0f) * VOLTAGE_DIVIDER_RATIO;

            if(V_sensor < VOLTAGE_MIN)
            {
            	wear = WEAR_MIN;
            }
            else if(V_sensor > VOLTAGE_MAX)
            {
            	wear = get_expected_wear(V_sensor);
            }

            test_status = test_cws_functionality(V_sensor, wear);

            sum = 0;
            count = 0;
        }
    }
}

void evaluate_test_result(void) {
  // default result is passing
  result = 1;
  for(int x = 0; x < MAX_POINTS; ++x) {
    if(cws_test.tests[x].passed == -1) {
      result = -1;
      break;
    }
  }
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
