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
#include "openamp.h"
#include "openamp_log.h"
#include <string.h>
#include <stdint.h>
#include "virt_uart.h"
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
}BST_Test;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_BUFFER_SIZE RPMSG_BUFFER_SIZE

//#define __LOG_TRACE_IO_


#define LOGLEVEL LOGINFO
#define __LOG_UART_IO_

#define STROKE_MIN 1
#define STROKE_MAX 9 // From graph
#define DUTY_CYCLE_TOLERANCE 5.0f  // ±5% DC per specs
#define SENSITIVITY 5.96f  // 5.96% DC/mm per specs
                           //#define S1_OFFSET 12.5f   // PWM1 offset: 12.5% DC
                           //#define S2_OFFSET 87.5f   // PWM2 offset: 87.5% DC
                           // 6 test_points for each distance
#define MAX_DISTANCE 6 + 1 // 1 for array offset

#define ADC_BUFFER_SIZE     20

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables -----#define ADC_BUFFER_SIZE     20----------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

IPCC_HandleTypeDef hipcc;

TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
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
volatile float expected_duty_cycle1 = 0;
volatile float expected_duty_cycle2 = 0;

// ADC DMA Buffer
volatile uint32_t adc_buffer[ADC_BUFFER_SIZE];

// ADC Parameters
volatile float V_sensor = 0.0f;
const float V_ref = 3.146f;
const float scaling_factor = (8.5f + 3.3f) / 3.3f;
const float V_sensor_max = 10.0f;
const float V_sensor_min = 0.0f;
const float stroke_max = 9.1f;

volatile int new_adc_data_ready = 0;
float adc_average = 0;

float s1_offset;
float s2_offset;

float estimated_stroke = 0;
int case1 = 0;
int case2 = 0;

uint8_t use_stringpot = 0;

test_point tests[MAX_DISTANCE];


BST_Test bst_test;

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
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM4_Init(void);
static void MX_IPCC_Init(void);
int MX_OPENAMP_Init(int RPMsgRole, rpmsg_ns_bind_cb ns_bind_cb);
/* USER CODE BEGIN PFP */
// Callback function for the virtual UART reception from the A7 core
void VIRT_UART0_RxCpltCallback(VIRT_UART_HandleTypeDef *huart);

// Function to perform the BST test with string potentiometer
int check_bst_values(float stroke, float duty_cycle1, float duty_cycle2);

// Function to calculate estimated stroke if no string potentiometer is selected
float estimated_stroke_from_duty_cycles(float duty_cycle1, float duty_cycle2);

// Functions to read stroke from ADC
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc);
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc);

// evaluate if test cases passed or not, a single fail causes it to fail
void evaluate_test_result(void);



/* for testing;
 *  from estimated stroke distance, check to ensure each one is within range
 *  distance that should be tested, maximum of 6 (for now)
 */


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
  MX_DMA_Init();
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM4_Init();
  /* USER CODE BEGIN 2 */
  // Start Timer 3 Channel 2 for PWM Output 1
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2); // For PWM1 (S1)

  // Start Timer 5 Channel 2 for PWM Output 2
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2); // For PWM2 (S2)


  if (HAL_ADCEx_Calibration_Start(&hadc1, ADC_CALIB_OFFSET_LINEARITY, ADC_SINGLE_ENDED) != HAL_OK)
   {
       Error_Handler();
   }

  // Start conversion with DMA
  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_SIZE);

  // Start timer to trigger ADC
  HAL_TIM_Base_Start(&htim4);

  /*
   * Create Virtual UART device
   * defined by a rpmsg channel attached to the remote device
   */
  /*Need to register callback for message reception by channels*/
  if(VIRT_UART_RegisterCallback(&huart0, VIRT_UART_RXCPLT_CB_ID, VIRT_UART0_RxCpltCallback) != VIRT_UART_OK)
  {
    Error_Handler();
  }

  // Create Virtual RPMSG file
  if(VIRT_UART_Init(&huart0) != VIRT_UART_OK) {
    log_err("Virt UART is FUCKED!\r\n");
    Error_Handler();
  }
  //csv header
  printf("time(s),duty_cycle1(%%),duty_cycle2(%%),stroke(mm),freq1(%%),freq2(%%)\r\n");
  uint32_t counter = 0;

  // keep track of previous duty cycle to see if there is a large change
  float dc_prev1 = -1;
  float dc_prev2 = -1;


  for(int x = 0;x < MAX_DISTANCE;x++) {
    tests[x].passed = 0;
    tests[x].tested = 0;
  }

  bst_test.tests = tests;
  bst_test.done = 0;

  size_t wrong_counter = 0;



  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    OPENAMP_check_for_message();

    // if dc changes more than 10%, skip current iteration

    HAL_Delay(10);

    if(start)
    {
      // Calculating the frequency and duty cycle for both channels
      frequency1 = (period1 > 0) ? (((float) SystemCoreClock) / ((htim3.Init.Prescaler + 1) * period1)) : 0;
      frequency2 = (period2 > 0) ? (((float) SystemCoreClock) / ((htim5.Init.Prescaler + 1) * period2)) : 0;


      if((frequency1 > 210) || (frequency1 < 190) || (frequency2 > 210) || (frequency2) < 190)
        continue;

      duty_cycle1 = (period1 > 0) ? (pulse_width1 / period1) * 100.0f : 0;
      duty_cycle2 = (period2 > 0) ? (pulse_width2 / period2) * 100.0f : 0;

      if(duty_cycle1 > 99.9 || duty_cycle2 > 99.9)
        continue;

      float lower_duty = (duty_cycle1 > duty_cycle2)? duty_cycle2: duty_cycle1;
      float higher_duty = (duty_cycle1>duty_cycle2)? duty_cycle1: duty_cycle2;

      if((((lower_duty>16) || (lower_duty< 9)) && ((higher_duty> 91) || (higher_duty< 84))) &&
          (dc_prev1 == -1 && dc_prev2 == -1)) {
        continue;
      }


      if(dc_prev1 == -1.0f && dc_prev2 == -1.0f) {
        dc_prev1 = duty_cycle1;
        dc_prev2 = duty_cycle2;

        s1_offset = duty_cycle1;
        s2_offset = duty_cycle2;
        continue;
      }

      float change1 =  dc_prev1 - duty_cycle1;
      int change_dc1 = (change1 < 0)? change1 *-1: change1;

      float change2 =  dc_prev2 - duty_cycle2;
      float change_dc2 = (change2 < 0)? change2 *-1: change2;
      // if change is greater than 10%, not possible change physically, skip reading that data
      if(change_dc1 > 50 || change_dc2 > 50)
        continue;


      dc_prev1 = duty_cycle1;
      dc_prev2 = duty_cycle2;

      if(use_stringpot)
      {
        if(new_adc_data_ready == 1)
        {
          new_adc_data_ready = 0;

          V_sensor = ((adc_average * V_ref) / 4095.0f) * scaling_factor;

          int dc1 = (int)(100* duty_cycle1);
          int dc2 = (int)(100* duty_cycle2);
          int f1 = (int)frequency1;
          int f2 = (int)frequency2;
          int strk = (int)(100* stroke);
          //        DC1.x,DC2.x,MM.x ,F1,j2
          log_info("%02d.%02d,%02d.%02d,%d.%02d,%02d,%02d\r\n"
              ,dc1/100,dc1%100
              ,dc2/100,dc2%100,
              strk/100,strk%100,
              f1,f2);
          check_bst_values(stroke, duty_cycle1, duty_cycle2);

          if(bst_test.done == MAX_DISTANCE) {
            start = 0;
            evaluate_test_result();
          }
        }
      }
      else
      {
        // Not Using String Potentiometer
        float estimated_stroke = estimated_stroke_from_duty_cycles(duty_cycle1, duty_cycle2);
        // duty_cycle1,duty_cycle2,estimated_stroke
        int dc1 = (int)(100* duty_cycle1);
        int dc2 = (int)(100* duty_cycle2);
        int f1 = (int)frequency1;
        int f2 = (int)frequency2;
        int strk = (int)(10* estimated_stroke);
        //        DC1.x,DC2.x,MM.x ,F1,j2  
        log_info("%02d.%02d,%02d.%02d,%02d.%d,%02d,%02d\r\n"
            ,dc1/100,dc1%100
            ,dc2/100,dc2%100,
            strk/10,strk%10,
            f1,f2);

        check_bst_values(estimated_stroke, duty_cycle1, duty_cycle2);

        if(bst_test.done == MAX_DISTANCE) {
          start = 0;
          evaluate_test_result();

        }
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
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSIDivValue = RCC_HSI_DIV1;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  RCC_OscInitStruct.PLL2.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL2.PLLSource = RCC_PLL12SOURCE_HSI;
  RCC_OscInitStruct.PLL2.PLLM = 4;
  RCC_OscInitStruct.PLL2.PLLN = 25;
  RCC_OscInitStruct.PLL2.PLLP = 2;
  RCC_OscInitStruct.PLL2.PLLQ = 1;
  RCC_OscInitStruct.PLL2.PLLR = 1;
  RCC_OscInitStruct.PLL2.PLLFRACV = 0;
  RCC_OscInitStruct.PLL2.PLLMODE = RCC_PLL_INTEGER;
  RCC_OscInitStruct.PLL3.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL3.PLLSource = RCC_PLL3SOURCE_HSI;
  RCC_OscInitStruct.PLL3.PLLM = 4;
  RCC_OscInitStruct.PLL3.PLLN = 28;
  RCC_OscInitStruct.PLL3.PLLP = 5;
  RCC_OscInitStruct.PLL3.PLLQ = 3;
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
  PeriphClkInit.CkperClockSelection = RCC_CKPERCLKSOURCE_HSI;
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
  htim3.Init.Prescaler = 127;
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
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
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

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

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
  float test_stroke = estimated_stroke;

  // Check if stroke in valid range
  /*
     if (test_stroke < STROKE_MIN || test_stroke > STROKE_MAX)
     {
     return -1;
     }
     */



  //Calculate expected duty cycles for stroke
  expected_duty_cycle1 = s1_offset+ (test_stroke * SENSITIVITY);
  expected_duty_cycle2 = s2_offset- (test_stroke * SENSITIVITY);

  float upperdc = (duty_cycle1 > duty_cycle2)? duty_cycle1:duty_cycle2;
  float lowerdc= (duty_cycle1 > duty_cycle2)? duty_cycle2:duty_cycle1;

  float expected_upperdc = (expected_duty_cycle1 > expected_duty_cycle2)? expected_duty_cycle1:expected_duty_cycle2;
  float expected_lowerdc = (expected_duty_cycle1 > expected_duty_cycle2)? expected_duty_cycle2:expected_duty_cycle1;

  // is measured DC within 5 of the expected
  // eg. measured = 12% expected = 20%, should fail 
  // distance away from 20 is greater than 5

  if(bst_test.tests[(int)test_stroke].tested == 0) {
    bst_test.tests[(int)test_stroke].tested = 1;

    // multiply result of number by -1 if less than 0 
    float upper_diff =  (upperdc - expected_upperdc < 0)? (upperdc - expected_upperdc) * -1: upperdc - expected_upperdc;
    float lower_diff =  (lowerdc - expected_lowerdc < 0)? (lowerdc - expected_lowerdc) * -1: lowerdc - expected_lowerdc;

    // one of them failed
    if(upper_diff > 5 || lower_diff > 5) {
      bst_test.tests[(int)test_stroke].passed = -1;
    }

    bst_test.done++;
  }



  // Check both possible cases (PWM1/PWM2 could be swapped)
  /*
     case1 = (~((long int)(duty_cycle1 *10) - (long int)(expected_duty_cycle1 * 10))-1 <= DUTY_CYCLE_TOLERANCE &&
     ~((long int)(duty_cycle2 * 10) - (long int)(expected_duty_cycle2 * 10))-1 <= DUTY_CYCLE_TOLERANCE);

     case2 = (~((long int)(duty_cycle1 *10) - (long int)(expected_duty_cycle2 * 10))-1 <= DUTY_CYCLE_TOLERANCE &&
     ~((long int)(duty_cycle2 * 10) - (long int)(expected_duty_cycle1 * 10))-1 <= DUTY_CYCLE_TOLERANCE);

*/
  return 0;
}

// Estimate stroke from the given duty cycles if no string potentiometer
float estimated_stroke_from_duty_cycles(float duty_cycle1, float duty_cycle2)
{
  // Sort duty cycles to identify which is PWM1
  float lower_duty = (duty_cycle1 > duty_cycle2)? duty_cycle2: duty_cycle1;
  float higher_duty = (duty_cycle1>duty_cycle2)? duty_cycle1: duty_cycle2;

  //Calculate stroke using 5.96% DC/mm sensitivity
  float stroke_from_lower = ((lower_duty - s1_offset) / SENSITIVITY);
  float stroke_from_higher = ((s2_offset- higher_duty) / SENSITIVITY);

  // error margin is 3.5%

  // Average the two estimates
  estimated_stroke = (stroke_from_lower + stroke_from_higher) / 2;

  return estimated_stroke;
}

void evaluate_test_result(void) {
  // default result is passing
  result = 1;
  for(int x = 0; x < MAX_DISTANCE; ++x) {
    if(bst_test.tests[x].passed == -1) {
      result = -1;
    } 
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
  static int sum = 0;
  static int count = 0;

  for(int i = ADC_BUFFER_SIZE/2; i < ADC_BUFFER_SIZE; i++) {
    sum += adc_buffer[i];
    count++;

    if(count >= ADC_BUFFER_SIZE) {
      adc_average = sum / ADC_BUFFER_SIZE;

      sum = 0;
      count = 0;
    }
    new_adc_data_ready = 1;
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
      adc_average = sum / ADC_BUFFER_SIZE;

      sum = 0;
      count = 0;
    }

    new_adc_data_ready = 1;
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
