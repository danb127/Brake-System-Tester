# Brake Signal Transmitter Testing with STM32f429i-DISC1

## Overview
In this document, I will be providing the finished code for testing the Brake Signal Transmitter and providing insight on why it works. The PWM signals frequency should read 200 Hz +- 10, tested on an oscilloscope first, then tested on the STMCubeIDE using UART and Expressions. The voltage supply should be 12 V, scaled down to 0 - 3.3V using a voltage divider or a buck converter to reach the microcontroller with proper voltage. The duty cycle at 0 mm stroke should be 12.5% +- 5% for PWM signal 1 and 87.5% +- 5% for PWM signal 2. All of this is specified in [[ZF Documentations - Brake Signal Transmitter - 480 004 002 0]].

## Test Circuit Set Up

### ECU Simulation Circuit
To set up the test circuit, you will first want to approach it by setting up the pull-up resistor. To do this, I simply connected a wire from the voltage through the power rail, to a bus on the breadboard. In series with that wire, I connected the voltage supply input of the Brake Signal Transmitter. In series with that is a diode to protect from reverse voltage as described in the test instructions. On the other side of the diode, also in series, I have connected a 3.3 kiloohm resistor. Once all of this has been set up, you can proceed to connect the PWM output signal in series to this resistor. Lastly, if you just want pure measurements of the BST PWM signal output, you should connect a ~2.2 nF capacitor in series with this PWM signal output and ground.

### Complete Circuit for Microcontroller
Because I am connecting this to my microcontroller and it cannot handle the voltage output of this, I also connected a voltage divider in series right after the PWM signal output and before the capacitor, with my R1 = 3.3 kiloohms and my R2 = 8.5 kiloohms to scale it down to around 3V given a 12V output (1/3 of whatever voltage you have).

### Schematic Design
Below I provided my circuit schematic drawing as well as my circuit breadboard design:



## CubeMX Set Up

### Clock Set Up
To make this circuit come to life on our microcontroller, I first set up my clock. I went to System Core/RCC/ and enabled my HSE clock and set it as the Crystal/Ceramic Resonator. I was then able to go into my Clock Configuration to set my HCLK to 180 MHz (fastest possible) and the rest of the timer configurations set itself up. Note the APB1 Timer clock time is 90 MHz as this will be important for later tasks.

### Timer Configurations

#### Timer Troubles
Initially, I tried using TIM2 and TIM5. The reason being is they are both 32-bit timers, so overflow is hard to achieve on these timers, making life a lot easier. Overflow is something to take note of because of period calculations using the microcontrollers timer ticks. The problem I ran into using these pins is that PA0 was not functional on the microcontroller. TIM2 and TIM5 can only use two pins so using both of these timers required both of those pins (PA0 and PA5). Since I ran into the problem of PA0 not reading input signals at all, I decided to switch to a different timer, the only problem being that, the rest are 16-bit timers and could not handle the reading of 200 Hz with a 180 MHz clock speed and a 90 MHz timer clock without overflow.

#### Timers Used
The timers that I ended up using was TIM2 (32-bit timer) and TIM3 (16-bit timer). Because TIM3 is a 16-bit timer trying to read 200 Hz frequency on a 180 MHz clock speed and a 90 MHz timer clock, you must account for overflow. To do this you must first change the prescaler of TIM3. I chose a prescaler of 13 (which results in 14 because it will always be (prescaler + 1)) because with a 90 MHz timer clock, we wanted to try to measure low frequencies without the timer overflowing. 13 was chosen by choosing 100 Hz since it is a lower frequency than 200 Hz, to endure the system could accurately measure even lower frequencies than 200 Hz. To measure 100 Hz, we need to count up to 10 ms (1/100 Hz) without overflow. At 90 MHz, 10 ms = 900,000 counts (90,000,000 * .01). We take the 900,000 counts and divide it by the max the 16-bit timer can count up to which is 65,536. 900,000 / 65,536 = 13.73f so we rounded up to 14 for safety reasons, giving us the prescaler of 13.

You may ask why I didn't just choose a lower clock speed but this messes with the UART and the baud rate which is something I must dive deeper into to learn what baud rate is acceptable at the lower frequency clock speeds.

### UART Configurations
We ended up using USART2 and setting the baud rate to 115200 as this is standard for high frequency clocks to output messages to the SWV ITM Data Console.

## main.c Code Added for PWM Testing
### Private Variables Added
The private variables included were all related to PWM including pulse width calculations for both signals, period calculations for both signals, duty cycle calculations for both signals, and frequency calculations for both signals

```C
/* USER CODE BEGIN PV */
// Pulse Width variable initializations
volatile float pulse_width1 = 0;
volatile float pulse_width2 = 0;

// Period variable initializations
volatile float period1 = 0;
volatile float period2 = 0;

// Duty Cycle variable initializations
volatile float duty_cycle1 = 0;
volatile float duty_cycle2 = 0;

// Frequncy variable initializations
volatile float frequency1 = 0;
volatile float frequency2 = 0;
/* USER CODE END PV */
```

### Timer Initializations
To begin the timers being used (TIM2 & TIM3) we needed to add there initialization functions to the beginning of the main code.

```C
/* USER CODE BEGIN 2 */
// Start Timer 2 Channel 1 for PWM Output 1
HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_1); // For PWM1 (S1)

// Start Timer 3 Channel 1 for PWM Output 2 
HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_1); // For PWM2 (S2)
/* USER CODE END 2 */
```

### While Loop Functionality
To make sure we continuously receive the frequency and the duty cycle calculations correctly from our interrupt handler calculations of the period and pulse width, we decided to put the calculations within our while loop and output UART messages every time it loops.

```C
/* USER CODE BEGIN WHILE */
while (1)
{
// Check for valid period, then calculate frequency
// (Timer Clock / Prescaler) / Measured Period (TIM2 has no prescaler)
// Return 0 if period is invalid to avoid division by 0
float frequency1 = (period1 > 0) ? (((float)90000000.0f / 14.0f)) / (period1) : 0;
float frequency2 = (period2 > 0) ? (float)90000000.0f / period2 : 0;

// Check for valid period, then calculate duty cycle
// (Pulse Width / Period) * 100 for percentage
// Return 0 if period is invalid
float duty_cycle1 = (period1 > 0) ? (((float)pulse_width1) / (period1)) * 100 : 0;
float duty_cycle2 = (period2 > 0) ? (float)pulse_width2 / period2 * 100 : 0;

// Print PWM 1 and PWM 2 frequency and duty cycle to SWV ITM Data Console via UART
char msg[100];
snprintf(msg, sizeof(msg), "PWM1: Freq=%.2f Hz, Duty=%.2f%% | PWM2: Freq=%.2f Hz, Duty=%.2f%%\n",
frequency1, duty_cycle1, frequency2, duty_cycle2);
}
// Function to print the msg sent via UART
Debug_Print(msg);
// Add a small delay to prevent getting stuck in HAL_GetTick
HAL_Delay(1);
/* USER CODE END WHILE */
```

## stm32f4xx_it.c Code Added for PWM Testing
### Private Variables Added
The same private variables from the main.c file are added but we use the term "extern" before adding them with no need for a declaration value because it is included in the main.c file. Basically we are importing the same variables so we can use them in both files without cross referencing. We also added variables for the last rising edge to help us accurately calculate the period and pulse width. The last rising edge variable is not declared with an "extern" term so it is the first use case of the variable, therefore, we need to declare an initial value of 0.

```C
/* USER CODE BEGIN PV */
extern volatile float pulse_width1, pulse_width2;

extern volatile float period1, period2;

extern volatile float duty_cycle1, duty_cycle2;

extern volatile float frequency1, frequency2;

extern volatile float latest_pressure;

// Used for the period calculation
volatile uint32_t last_rising_edge1 = 0, last_rising_edge2 = 0;
/* USER CODE END PV */
```
### Private Function Prototypes Added
We added two functions specifically to be able to print UART messages in the SWV ITM Data Console.

```C
/* USER CODE BEGIN PFP */
// Allows you to pass a char type variable to output a message
void Debug_Print(const char *format,...);
// Parses a string of characters which is used in the Debug Print function
void ITM_SendString(const char *str);
/* USER CODE END PFP */
```

### External Variables Added
Added an external variable for the UART message to be passed through the stm32f4xx_it.c file as well.
```C
/* USER CODE BEGIN EV */
extern UART_HandleTypeDef huart2;
/* USER CODE END EV */
```

### Timer Interrupt Handlers
In the timer interrupt handlers, I realized the code must be before the call of the interrupt handler itself or else this will mess up your values received and you will not get accurate readings. I begin both codes by checking to make sure an interrupt happens in the first place before capturing any values. I then capture the current value of the timer and if the pin is set (set = 1 = rising edge) then we calculate the period by subtracting the capture minus the last rising edge. we then set the capture to the last rising edge to update it. On the falling edge we capture the pulse width by taking the current capture and subtracting it from the last rising edge.

```C
void TIM2_IRQHandler(void)
{
/* USER CODE BEGIN TIM2_IRQn 0 */

	if (__HAL_TIM_GET_FLAG(&htim2, TIM_FLAG_CC1) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim2, TIM_IT_CC1) != RESET)
		{
			__HAL_TIM_CLEAR_IT(&htim2, TIM_IT_CC1);
	
			uint32_t current_capture = HAL_TIM_ReadCapturedValue(&htim2, TIM_CHANNEL_1);
	
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_5) == GPIO_PIN_SET) // Assuming PA5 is the input pin for TIM2
	
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

/* USER CODE END TIM2_IRQn 0 */

HAL_TIM_IRQHandler(&htim2);

/* USER CODE BEGIN TIM2_IRQn 1 */

/* USER CODE END TIM2_IRQn 1 */
}
```

However, because TIM3 is a 16-bit timer, we had to add protocols to ensure overflow doesn't happen. To do this we added checks for even when the timer wraps around from 65,535 to 0.

```C
void TIM3_IRQHandler(void)
{
/* USER CODE BEGIN TIM3_IRQn 0 */
	
	if (__HAL_TIM_GET_FLAG(&htim3, TIM_FLAG_CC1) != RESET)
	{
		if (__HAL_TIM_GET_IT_SOURCE(&htim3, TIM_IT_CC1) != RESET)
		{
	
			__HAL_TIM_CLEAR_IT(&htim3, TIM_IT_CC1);
	
			uint32_t current_capture = HAL_TIM_ReadCapturedValue(&htim3, TIM_CHANNEL_1);
	
			if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_6) == GPIO_PIN_SET)
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
```

### UART Messages
The remaining code is standard coding practice for displaying UART messages via the SWV ITM Data Console
```C
/* USER CODE BEGIN 1 */

void Debug_Print(const char *format, ...)
{
	char buffer[128];

	va_list args;
	
	va_start(args, format);

	vsnprintf(buffer, sizeof(buffer), format, args);

	va_end(args);

	// Send through UART
	HAL_UART_Transmit(&huart2, (uint8_t*)buffer, strlen(buffer), HAL_MAX_DELAY);

	// Send through SWV
	ITM_SendString(buffer);
}


void ITM_SendString(const char *str)
{
	while (*str)
	{
		ITM_SendChar(*str++);
	}
}
```




# Setting Up the STM32MP157F-DK2 Board
## PWM Input Capture Code
This code works on it's own as of now for the dk2 controller PWM Inputs
### main.c
**Variables**
```C
/* USER CODE BEGIN PV */
volatile float pulse_width1 = 0;
volatile float pulse_width2 = 0;
volatile float period1 = 0;
volatile float period2 = 0;
volatile float duty_cycle1 = 0;
volatile float duty_cycle2 = 0;
volatile float frequency1 = 0;
volatile float frequency2 = 0;
/* USER CODE END PV */
```
**Private Funcion Prototypes**
```C
static void MX_TIM5_Init(void);
static void MX_TIM3_Init(void);
```
**Initialize Peripherals**
```C
 /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM5_Init();
  MX_TIM3_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_IC_Start_IT(&htim3, TIM_CHANNEL_2);
  HAL_TIM_IC_Start_IT(&htim5, TIM_CHANNEL_2);
  /* USER CODE END 2 */
```
**While Loop**
```C
 /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	    float frequency1 = (period1 > 0) ? (((float)90000000.0f / 14.0f))  / (period1) : 0;
	    float frequency2 = (period2 > 0) ? (float)90000000.0f / period2 : 0;

	    float duty_cycle1 = (period1 > 0) ? (((float)pulse_width1) / (period1)) * 100  : 0;
	    float duty_cycle2 = (period2 > 0) ? (float)pulse_width2 / period2 * 100 : 0;


	    // Add a small delay to prevent getting stuck in HAL_GetTick
	    HAL_Delay(100);
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

```


### stm32mp1xx_it.c:
```C
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

```

## ADC Enable
Some ADC pins are available with this starter package Device Tree on DK2 board (please also refer to [board documentation](https://www.st.com/en/evaluation-tools/stm32mp157f-dk2.html#documentation):(

- ADC1_INP0 and ADC2_INP0 on pin ANA0 (Arduino connector A2)
- ADC1_INP1 and ADC2_INP0 on pin ANA1 (Arduino connector A3)
- ADC1_INP6 on pin PF12 (Arduino connector A5)
- ADC1_INP13 on pin PC3 (Arduino connector A4)
- ADC2_INP2 on pin PF13 (Arduino connector A1)
- ADC2_INP6 on PF14 (Arduino connector A0)
  
- When I add adc_start_it, tim_start_it stops working and adc only runs once

