/**
 ******************************************************************************
 * File Name          : main.c
 * Description        : Main program body
 ******************************************************************************
 * This notice applies to any and all portions of this file
 * that are not between comment pairs USER CODE BEGIN and
 * USER CODE END. Other portions of this file, whether
 * inserted by the user or by software development tools
 * are owned by their respective copyright owners.
 *
 * Copyright (c) 2017 STMicroelectronics International N.V.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted, provided that the following conditions are met:
 *
 * 1. Redistribution of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 * 3. Neither the name of STMicroelectronics nor the names of other
 *    contributors to this software may be used to endorse or promote products
 *    derived from this software without specific written permission.
 * 4. This software, including modifications and/or derivative works of this
 *    software, must execute solely and exclusively on microcontroller or
 *    microprocessor devices manufactured by or for STMicroelectronics.
 * 5. Redistribution and use of this software other than as permitted under
 *    this license is void and will automatically terminate your rights under
 *    this license.
 *
 * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
 * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
 * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "cmsis_os.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

UART_HandleTypeDef huart4;

osThreadId piscaLedHandle;
osThreadId taskSporadicHandle;
osThreadId readSensorHandle;
osThreadId taskControlHandle;
osThreadId gateKeeperHandle;
osMessageQId myQueueHandle;

/* USER CODE BEGIN PV */

/* Private variables ---------------------------------------------------------*/
int adcValue;
int adcValuePPM;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_UART4_Init(void);
static void MX_ADC1_Init(void);
void StartPiscaLed(void const * argument);
void StartTaskSporadic(void const * argument);
void StartTaskReadSensor(void const * argument);
void StartControlTask(void const * argument);
void StartTaskGatekeeper(void const * argument);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

int main(void)
{

	/* USER CODE BEGIN 1 */

	/* USER CODE END 1 */

	/* MCU Configuration----------------------------------------------------------*/

	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();

	/* USER CODE BEGIN Init */

	/* USER CODE END Init */

	/* Configure the system clock */
	SystemClock_Config();

	/* USER CODE BEGIN SysInit */

	/* USER CODE END SysInit */

	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_UART4_Init();
	MX_ADC1_Init();

	/* USER CODE BEGIN 2 */

	/* USER CODE END 2 */

	/* USER CODE BEGIN RTOS_MUTEX */
	/* add mutexes, ... */
	/* USER CODE END RTOS_MUTEX */

	/* USER CODE BEGIN RTOS_SEMAPHORES */
	/* add semaphores, ... */
	/* USER CODE END RTOS_SEMAPHORES */

	/* USER CODE BEGIN RTOS_TIMERS */
	/* start timers, add new ones, ... */
	/* USER CODE END RTOS_TIMERS */

	/* Create the thread(s) */
	/* definition and creation of piscaLed */
	osThreadDef(piscaLed, StartPiscaLed, osPriorityNormal, 0, 128);
	piscaLedHandle = osThreadCreate(osThread(piscaLed), NULL);

	/* definition and creation of taskSporadic */
	osThreadDef(taskSporadic, StartTaskSporadic, osPriorityRealtime, 0, 128);
	taskSporadicHandle = osThreadCreate(osThread(taskSporadic), NULL);

	/* definition and creation of readSensor */
	osThreadDef(readSensor, StartTaskReadSensor, osPriorityNormal, 0, 128);
	readSensorHandle = osThreadCreate(osThread(readSensor), NULL);

	/* definition and creation of taskControl */
	osThreadDef(taskControl, StartControlTask, osPriorityNormal, 0, 128);
	taskControlHandle = osThreadCreate(osThread(taskControl), NULL);

	/* definition and creation of gateKeeper */
	osThreadDef(gateKeeper, StartTaskGatekeeper, osPriorityNormal, 0, 128);
	gateKeeperHandle = osThreadCreate(osThread(gateKeeper), NULL);

	/* USER CODE BEGIN RTOS_THREADS */
	/* add threads, ... */

	/* USER CODE END RTOS_THREADS */

	/* Create the queue(s) */
	/* definition and creation of myQueue */
	osMessageQDef(myQueue, 128, uint8_t[42]);
	myQueueHandle = osMessageCreate(osMessageQ(myQueue), NULL);

	/* USER CODE BEGIN RTOS_QUEUES */
	/* add queues, ... */
	/* USER CODE END RTOS_QUEUES */

	/*suspend task sporadic	 */

	vTaskSuspend(taskSporadicHandle);

	/* Start scheduler */
	osKernelStart();

	/* We should never get here as control is now taken by the scheduler */

	/* Infinite loop */
	/* USER CODE BEGIN WHILE */
	while (1)
	{
		/* USER CODE END WHILE */

		/* USER CODE BEGIN 3 */

	}
	/* USER CODE END 3 */

}

/** System Clock Configuration
 */
void SystemClock_Config(void)
{

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Configure the main internal regulator output voltage
	 */
	__HAL_RCC_PWR_CLK_ENABLE();

	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
	RCC_OscInitStruct.HSIState = RCC_HSI_ON;
	RCC_OscInitStruct.HSICalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
			|RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
	RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 15, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* UART4 init function */
static void MX_UART4_Init(void)
{

	huart4.Instance = UART4;
	huart4.Init.BaudRate = 115200;
	huart4.Init.WordLength = UART_WORDLENGTH_8B;
	huart4.Init.StopBits = UART_STOPBITS_1;
	huart4.Init.Parity = UART_PARITY_NONE;
	huart4.Init.Mode = UART_MODE_TX_RX;
	huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart4.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_UART_Init(&huart4) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void)
{

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14, GPIO_PIN_RESET);

	/*Configure GPIO pins : PB11 PB13 PB14 */
	GPIO_InitStruct.Pin = GPIO_PIN_11|GPIO_PIN_13|GPIO_PIN_14;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* StartPiscaLed function */
void StartPiscaLed(void const * argument)
{

	/* USER CODE BEGIN 5 */
	uint8_t a[42];
	int aux;
	TickType_t xTime1, xTime2;

	/* Infinite loop */
	for(;;)
	{
		xTime1 = xTaskGetTickCountFromISR();

		HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_11);
		osDelay(300);

		xTime2 = xTaskGetTickCountFromISR();

		a[0] = '[';
		a[1] = 'p';
		a[2] = 'i';
		a[3] = 's';
		a[4] = 'c';
		a[5] = 'a';
		a[6] = 'L';
		a[7] = 'e';
		a[8] = 'd';
		a[9] = '_';
		a[10] = (xTime1 / 1000000000) + '0';
		xTime1 = xTime1 % 1000000000;
		a[11] = (xTime1 / 100000000) + '0';
		xTime1 = xTime1 % 100000000;
		a[12] = (xTime1 / 10000000) + '0';
		xTime1 = xTime1 % 10000000;
		a[13] =(xTime1 / 1000000) + '0';
		xTime1 = xTime1 % 1000000;
		a[14] = (xTime1 / 100000) + '0';
		xTime1 = xTime1 % 100000;
		a[15] = (xTime1 / 10000) + '0';
		xTime1 = xTime1 % 10000;
		a[16] = (xTime1 / 1000) + '0';
		xTime1 = xTime1 % 1000;
		a[17] = (xTime1 / 100) + '0';
		xTime1 = xTime1 % 100;
		a[18] = (xTime1 / 10) + '0';
		xTime1 = xTime1 % 10;
		a[19] = xTime1 + '0';
		a[20] = '_';
		a[21] = (xTime2 / 1000000000) + '0';
		xTime2 = xTime2 % 1000000000;
		a[22] = (xTime2 / 100000000) + '0';
		xTime2 = xTime2 % 100000000;
		a[23] = (xTime2 / 10000000) + '0';
		xTime2 = xTime2 % 10000000;
		a[24] =(xTime2 / 1000000) + '0';
		xTime2 = xTime2 % 1000000;
		a[25] = (xTime2 / 100000) + '0';
		xTime2 = xTime2 % 100000;
		a[26] = (xTime2 / 10000) + '0';
		xTime2 = xTime2 % 10000;
		a[27] = (xTime2 / 1000) + '0';
		xTime2 = xTime2 % 1000;
		a[28] = (xTime2 / 100) + '0';
		xTime2 = xTime2 % 100;
		a[29] = (xTime2 / 10) + '0';
		xTime2 = xTime2 % 10;
		a[30] = xTime2 + '0';
		a[31] = '_';
		aux = adcValue;
		a[32] = (aux / 1000) + '0';
		aux = aux % 1000;
		a[33] = (aux / 100) + '0';
		aux = aux % 100;
		a[34] = (aux / 10) + '0';
		aux = aux % 10;
		a[35] = aux + '0';
		a[36] ='_';
		aux = adcValuePPM;
		a[37] = (aux / 100) + '0';
		aux = aux % 100;
		a[38] = (aux / 10) + '0';
		aux = aux % 10;
		a[39] = aux + '0';
		a[40] = ']';
		a[41] = '\n';

		xQueueSendToBack(myQueueHandle, a, 0);
		osDelay(1);
	}
	/* USER CODE END 5 */
}

/* StartTaskSporadic function */
void StartTaskSporadic(void const * argument)
{
	/* USER CODE BEGIN StartTaskSporadic */
	uint8_t b[42];
	int aux;
	TickType_t xTime1, xTime2;

	/* Infinite loop */
	for(;;)
	{
		xTime1 =  xTaskGetTickCountFromISR();

		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 1);
		HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 1);

		xTime2 =  xTaskGetTickCountFromISR();

		b[0] = '[';
		b[1] = 's';
		b[2] = 'p';
		b[3] = 'o';
		b[4] = 'r';
		b[5] = 'a';
		b[6] = 'd';
		b[7] = 'i';
		b[8] = 'c';
		b[9] = '_';
		b[10] = (xTime1 / 1000000000) + '0';
		xTime1 = xTime1 % 1000000000;
		b[11] = (xTime1 / 100000000) + '0';
		xTime1 = xTime1 % 100000000;
		b[12] = (xTime1 / 10000000) + '0';
		xTime1 = xTime1 % 10000000;
		b[13] =(xTime1 / 1000000) + '0';
		xTime1 = xTime1 % 1000000;
		b[14] = (xTime1 / 100000) + '0';
		xTime1 = xTime1 % 100000;
		b[15] = (xTime1 / 10000) + '0';
		xTime1 = xTime1 % 10000;
		b[16] = (xTime1 / 1000) + '0';
		xTime1 = xTime1 % 1000;
		b[17] = (xTime1 / 100) + '0';
		xTime1 = xTime1 % 100;
		b[18] = (xTime1 / 10) + '0';
		xTime1 = xTime1 % 10;
		b[19] = xTime1 + '0';
		b[20] = '_';
		b[21] = (xTime2 / 1000000000) + '0';
		xTime2 = xTime2 % 1000000000;
		b[22] = (xTime2 / 100000000) + '0';
		xTime2 = xTime2 % 100000000;
		b[23] = (xTime2 / 10000000) + '0';
		xTime2 = xTime2 % 10000000;
		b[24] =(xTime2 / 1000000) + '0';
		xTime2 = xTime2 % 1000000;
		b[25] = (xTime2 / 100000) + '0';
		xTime2 = xTime2 % 100000;
		b[26] = (xTime2 / 10000) + '0';
		xTime2 = xTime2 % 10000;
		b[27] = (xTime2 / 1000) + '0';
		xTime2 = xTime2 % 1000;
		b[28] = (xTime2 / 100) + '0';
		xTime2 = xTime2 % 100;
		b[29] = (xTime2 / 10) + '0';
		xTime2 = xTime2 % 10;
		b[30] = xTime2 + '0';
		b[31] = '_';
		aux = adcValue;
		b[32] = (aux / 1000) + '0';
		aux = aux % 1000;
		b[33] = (aux / 100) + '0';
		aux = aux % 100;
		b[34] = (aux / 10) + '0';
		aux = aux % 10;
		b[35] = aux + '0';
		b[36] ='_';
		aux = adcValuePPM;
		b[37] = (aux / 100) + '0';
		aux = aux % 100;
		b[38] = (aux / 10) + '0';
		aux = aux % 10;
		b[39] = aux + '0';
		b[40] = ']';
		b[41] = '\n';

		xQueueSendToBack(myQueueHandle, b, 0);

		osDelay(1);

		vTaskSuspend(taskSporadicHandle);
	}
	/* USER CODE END StartTaskSporadic */
}

/* StartTaskReadSensor function */
void StartTaskReadSensor(void const * argument)
{
	/* USER CODE BEGIN StartTaskReadSensor */
	uint8_t c[42];
	int aux;
	TickType_t xTime1, xTime2;

	/* Infinite loop */
	for(;;)
	{
		xTime1 =  xTaskGetTickCountFromISR();

		HAL_ADC_Start(&hadc1);
		if (HAL_ADC_PollForConversion(&hadc1, 1000000) == HAL_OK)
		{
			adcValue = HAL_ADC_GetValue(&hadc1);
		}
		HAL_ADC_Stop(&hadc1);

		adcValuePPM = adcValue * 100 / 4096;

		xTime2 =  xTaskGetTickCountFromISR();

		c[0] = '[';
		c[1] = 'l';
		c[2] = 'e';
		c[3] = 'S';
		c[4] = 'e';
		c[5] = 'n';
		c[6] = 's';
		c[7] = 'o';
		c[8] = 'r';
		c[9] = '_';
		c[10] = (xTime1 / 1000000000) + '0';
		xTime1 = xTime1 % 1000000000;
		c[11] = (xTime1 / 100000000) + '0';
		xTime1 = xTime1 % 100000000;
		c[12] = (xTime1 / 10000000) + '0';
		xTime1 = xTime1 % 10000000;
		c[13] =(xTime1 / 1000000) + '0';
		xTime1 = xTime1 % 1000000;
		c[14] = (xTime1 / 100000) + '0';
		xTime1 = xTime1 % 100000;
		c[15] = (xTime1 / 10000) + '0';
		xTime1 = xTime1 % 10000;
		c[16] = (xTime1 / 1000) + '0';
		xTime1 = xTime1 % 1000;
		c[17] = (xTime1 / 100) + '0';
		xTime1 = xTime1 % 100;
		c[18] = (xTime1 / 10) + '0';
		xTime1 = xTime1 % 10;
		c[19] = xTime1 + '0';
		c[20] = '_';
		c[21] = (xTime2 / 1000000000) + '0';
		xTime2 = xTime2 % 1000000000;
		c[22] = (xTime2 / 100000000) + '0';
		xTime2 = xTime2 % 100000000;
		c[23] = (xTime2 / 10000000) + '0';
		xTime2 = xTime2 % 10000000;
		c[24] =(xTime2 / 1000000) + '0';
		xTime2 = xTime2 % 1000000;
		c[25] = (xTime2 / 100000) + '0';
		xTime2 = xTime2 % 100000;
		c[26] = (xTime2 / 10000) + '0';
		xTime2 = xTime2 % 10000;
		c[27] = (xTime2 / 1000) + '0';
		xTime2 = xTime2 % 1000;
		c[28] = (xTime2 / 100) + '0';
		xTime2 = xTime2 % 100;
		c[29] = (xTime2 / 10) + '0';
		xTime2 = xTime2 % 10;
		c[30] = xTime2 + '0';
		c[31] = '_';
		aux = adcValue;
		c[32] = (aux / 1000) + '0';
		aux = aux % 1000;
		c[33] = (aux / 100) + '0';
		aux = aux % 100;
		c[34] = (aux / 10) + '0';
		aux = aux % 10;
		c[35] = aux + '0';
		c[36] ='_';
		aux = adcValuePPM;
		c[37] = (aux / 100) + '0';
		aux = aux % 100;
		c[38] = (aux / 10) + '0';
		aux = aux % 10;
		c[39] = aux + '0';
		c[40] = ']';
		c[41] = '\n';

		xQueueSendToBack(myQueueHandle, c, 0);

		osDelay(50);
	}
	/* USER CODE END StartTaskReadSensor */
}

/* StartControlTask function */
void StartControlTask(void const * argument)
{
	/* USER CODE BEGIN StartControlTask */
	uint8_t d[42];
	int aux;
	TickType_t xTime1, xTime2;

	/* Infinite loop */
	for(;;)
	{
		xTime1 =  xTaskGetTickCountFromISR();

		if(adcValuePPM >= 20){
			vTaskResume(taskSporadicHandle);
		}
		else
		{
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_14, 0);
			HAL_GPIO_WritePin(GPIOB, GPIO_PIN_13, 0);
		}

		xTime2 =  xTaskGetTickCountFromISR();

		d[0] = '[';
		d[1] = 'c';
		d[2] = 'o';
		d[3] = 'n';
		d[4] = 't';
		d[5] = 'r';
		d[6] = 'o';
		d[7] = 'l';
		d[8] = 'e';
		d[9] = '_';
		d[10] = (xTime1 / 1000000000) + '0';
		xTime1 = xTime1 % 1000000000;
		d[11] = (xTime1 / 100000000) + '0';
		xTime1 = xTime1 % 100000000;
		d[12] = (xTime1 / 10000000) + '0';
		xTime1 = xTime1 % 10000000;
		d[13] =(xTime1 / 1000000) + '0';
		xTime1 = xTime1 % 1000000;
		d[14] = (xTime1 / 100000) + '0';
		xTime1 = xTime1 % 100000;
		d[15] = (xTime1 / 10000) + '0';
		xTime1 = xTime1 % 10000;
		d[16] = (xTime1 / 1000) + '0';
		xTime1 = xTime1 % 1000;
		d[17] = (xTime1 / 100) + '0';
		xTime1 = xTime1 % 100;
		d[18] = (xTime1 / 10) + '0';
		xTime1 = xTime1 % 10;
		d[19] = xTime1 + '0';
		d[20] = '_';
		d[21] = (xTime2 / 1000000000) + '0';
		xTime2 = xTime2 % 1000000000;
		d[22] = (xTime2 / 100000000) + '0';
		xTime2 = xTime2 % 100000000;
		d[23] = (xTime2 / 10000000) + '0';
		xTime2 = xTime2 % 10000000;
		d[24] = (xTime2 / 1000000) + '0';
		xTime2 = xTime2 % 1000000;
		d[25] = (xTime2 / 100000) + '0';
		xTime2 = xTime2 % 100000;
		d[26] = (xTime2 / 10000) + '0';
		xTime2 = xTime2 % 10000;
		d[27] = (xTime2 / 1000) + '0';
		xTime2 = xTime2 % 1000;
		d[28] = (xTime2 / 100) + '0';
		xTime2 = xTime2 % 100;
		d[29] = (xTime2 / 10) + '0';
		xTime2 = xTime2 % 10;
		d[30] = xTime2 + '0';
		d[31] = '_';
		aux = adcValue;
		d[32] = (aux / 1000) + '0';
		aux = aux % 1000;
		d[33] = (aux / 100) + '0';
		aux = aux % 100;
		d[34] = (aux / 10) + '0';
		aux = aux % 10;
		d[35] = aux + '0';
		d[36] ='_';
		aux = adcValuePPM;
		d[37] = (aux / 100) + '0';
		aux = aux % 100;
		d[38] = (aux / 10) + '0';
		aux = aux % 10;
		d[39] = aux + '0';
		d[40] = ']';
		d[41] = '\n';

		xQueueSendToBack(myQueueHandle, d, 0);

		osDelay(20);
	}
	/* USER CODE END StartControlTask */
}

/* StartTaskGatekeeper function */
void StartTaskGatekeeper(void const * argument)
{
	/* USER CODE BEGIN StartTaskGatekeeper */
	BaseType_t xStatus;
	uint8_t send[42];

	/* Infinite loop */
	for(;;)
	{
		xStatus = xQueueReceive( myQueueHandle, &send, pdMS_TO_TICKS( 100 )  );

		if( xStatus == pdPASS )
		{
			HAL_UART_Transmit_IT(&huart4, send , 42);
		}
		else
		{

		}
		osDelay(1);
	}
	/* USER CODE END StartTaskGatekeeper */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  None
 * @retval None
 */
void _Error_Handler(char * file, int line)
{
	/* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	while(1)
	{

	}
	/* USER CODE END Error_Handler_Debug */
}

#ifdef USE_FULL_ASSERT

/**
 * @brief Reports the name of the source file and the source line number
 * where the assert_param error has occurred.
 * @param file: pointer to the source file name
 * @param line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line)
{
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */

}

#endif

/**
 * @}
 */

/**
 * @}
 */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
