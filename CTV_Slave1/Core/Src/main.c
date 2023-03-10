/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2022 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define portSLV1	GPIOC
#define pinSLV1H	7
#define pinSLV1L	(pinSLV1H+16)

#define  ARM_CM_DEMCR      (*(uint32_t *)0xE000EDFC)

#define  ARM_CM_DWT_CTRL   (*(uint32_t *)0xE0001000)

#define  ARM_CM_DWT_CYCCNT (*(uint32_t *)0xE0001004)
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN PV */
#define NUMBER_ADC_CHANNEL 12
#define NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL 1

uint16_t DMAhasilADC[NUMBER_ADC_CHANNEL * NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL];

//const int adcChCount = sizeof(DMAhasilADC) / sizeof(DMAhasilADC[0]);
int ConvOK = 0;
//uint16_t abc[6]; int abcd[6];
int Rx[6],positive,negative;
int debug_adc[12];

uint32_t  start;

uint32_t  stop;

uint32_t  delta;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
static void MX_SPI2_Init(void);
/* USER CODE BEGIN PFP */
int ADC_DMA_AVERAGE(int channel)
{
	int adc_sum;
	int i;

	adc_sum = 0;
	if(channel < NUMBER_ADC_CHANNEL )
	{
		for(i=0; i<NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL; i++)
			adc_sum += DMAhasilADC[channel+i*NUMBER_ADC_CHANNEL];
	}
	else
		return 1;

	adc_sum = adc_sum/NUMBER_ADC_CHANNEL_AVERAGE_PER_CHANNEL;
	return adc_sum;
}

void ADC_Select_Ch0(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_10;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch1(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_11;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch2(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch3(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_1;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch4(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_2;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch5(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_3;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch6(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_4;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch7(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_5;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch8(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_6;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch9(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_7;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch10(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_8;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

void ADC_Select_Ch11(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
	/** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
	 */
	sConfig.Channel = ADC_CHANNEL_9;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		Error_Handler();
	}
}

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

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_ADC1_Init();
  MX_SPI2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  if (ARM_CM_DWT_CTRL != 0) {        // See if DWT is available

	  ARM_CM_DEMCR      |= 1 << 24;  // Set bit 24

	  ARM_CM_DWT_CYCCNT  = 0;

	  ARM_CM_DWT_CTRL   |= 1 << 0;   // Set bit 0
  }
//  HAL_ADC_Start_DMA(&hadc1, (uint32_t*)DMAhasilADC, sizeof(DMAhasilADC));
  while (1)
  {
	  //===== WAIT READ ADC CMD =====
	  while(HAL_GPIO_ReadPin(MSTR_GPIO_Port, MSTR_Pin) == 0){}
	  //===== READING ADC =====
	  ADC_Select_Ch0();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[0] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);

	  ADC_Select_Ch1();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[1] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);
//	  debug_adc[0] = positive;
//	  debug_adc[1] = negative;
//	  Rx[0] = positive - negative;

	  ADC_Select_Ch2();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[2] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);

	  ADC_Select_Ch3();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[3] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);
//	  debug_adc[2] = positive;
//	  debug_adc[3] = negative;
//	  Rx[1] = positive - negative;

	  ADC_Select_Ch4();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[4] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);

	  ADC_Select_Ch5();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[5] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);
//	  debug_adc[4] = positive;
//	  debug_adc[5] = negative;
//	  Rx[2] = positive - negative;

	  ADC_Select_Ch6();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[6] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);

	  ADC_Select_Ch7();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[7] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);
//	  debug_adc[6] = positive;
//	  debug_adc[7] = negative;
//	  Rx[3] = positive - negative;

	  ADC_Select_Ch8();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[8] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);

	  ADC_Select_Ch9();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[9] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);
//	  debug_adc[8] = positive;
//	  debug_adc[9] = negative;
//	  Rx[4] = positive - negative;

	  ADC_Select_Ch10();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[10] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);

	  ADC_Select_Ch11();
	  HAL_ADC_Start(&hadc1);
	  HAL_ADC_PollForConversion(&hadc1, 1000);
	  debug_adc[11] = HAL_ADC_GetValue(&hadc1);
//	  HAL_ADC_Stop(&hadc1);
//	  debug_adc[10] = positive;
//	  debug_adc[11] = negative;
//	  Rx[5] = positive - negative;

	  //start = ARM_CM_DWT_CYCCNT;
	  //HAL_Delay(2);

//	  while(ConvOK == 0){}
//	  ConvOK = 0;
//	  Rx[0] = debug_adc[0] - debug_adc[1];
//	  Rx[1] = debug_adc[2] - debug_adc[3];
//	  Rx[2] = debug_adc[4] - debug_adc[5];
//	  Rx[3] = debug_adc[6] - debug_adc[7];
//	  Rx[4] = debug_adc[8] - debug_adc[9];
//	  Rx[5] = debug_adc[10] - debug_adc[11];


	  //stop  = ARM_CM_DWT_CYCCNT;

	  //delta = stop - start;

	  //Send ADC Finish Signal (PIN SLV1)
	  //portSLV1->BSRR |= (1<<pinSLV1H);

	  HAL_SPI_Transmit(&hspi2, (uint8_t *)debug_adc, sizeof(debug_adc), 5000);

	  while(HAL_GPIO_ReadPin(CS1_GPIO_Port, CS1_Pin) == 0){}


	  asm("NOP");

	  //Turn Off the whole Signal
	  //portSLV1->BSRR |= (1<<pinSLV1L);

	  //waiting falling edge
	  while(HAL_GPIO_ReadPin(MSTR_GPIO_Port, MSTR_Pin) == 1){}

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

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE2);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 4;
  RCC_OscInitStruct.PLL.PLLN = 84;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */
  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 12;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_10;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_112CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_11;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SLV1_GPIO_Port, SLV1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC13 PC14 PC15 PC2
                           PC3 PC4 PC5 PC8
                           PC9 PC10 PC11 PC12 */
  GPIO_InitStruct.Pin = GPIO_PIN_13|GPIO_PIN_14|GPIO_PIN_15|GPIO_PIN_2
                          |GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_8
                          |GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11|GPIO_PIN_12;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PB2 PB10 PB4 PB5
                           PB6 PB7 PB8 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_10|GPIO_PIN_4|GPIO_PIN_5
                          |GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : CS1_Pin */
  GPIO_InitStruct.Pin = CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PB14 */
  GPIO_InitStruct.Pin = GPIO_PIN_14;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : MSTR_Pin */
  GPIO_InitStruct.Pin = MSTR_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MSTR_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SLV1_Pin */
  GPIO_InitStruct.Pin = SLV1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(SLV1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 PA11
                           PA12 PA15 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11
                          |GPIO_PIN_12|GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : PD2 */
  GPIO_InitStruct.Pin = GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PB9 */
  GPIO_InitStruct.Pin = GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_MEDIUM;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */
void HAL_ADC_ConvHalfCpltCallback(ADC_HandleTypeDef* hadc)
{


}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin == GPIO_PIN_14)
  {
	  HAL_SPI_DeInit(&hspi2);
	  HAL_NVIC_SystemReset();
  }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	UNUSED(hadc);
	debug_adc[0] = DMAhasilADC[0];
	debug_adc[1] = DMAhasilADC[1];
	debug_adc[2] = DMAhasilADC[2];
	debug_adc[3] = DMAhasilADC[3];
	debug_adc[4] = DMAhasilADC[4];
	debug_adc[5] = DMAhasilADC[5];
	debug_adc[6] = DMAhasilADC[6];
	debug_adc[7] = DMAhasilADC[7];
	debug_adc[8] = DMAhasilADC[8];
	debug_adc[9] = DMAhasilADC[9];
	debug_adc[10] = DMAhasilADC[10];
	debug_adc[11] = DMAhasilADC[11];
//	debug_adc[0] = ADC_DMA_AVERAGE(0);
//	debug_adc[1] = ADC_DMA_AVERAGE(1);
//	debug_adc[2] = ADC_DMA_AVERAGE(2);
//	debug_adc[3] = ADC_DMA_AVERAGE(3);
//	debug_adc[4] = ADC_DMA_AVERAGE(4);
//	debug_adc[5] = ADC_DMA_AVERAGE(5);
//	debug_adc[6] = ADC_DMA_AVERAGE(6);
//	debug_adc[7] = ADC_DMA_AVERAGE(7);
//	debug_adc[8] = ADC_DMA_AVERAGE(8);
//	debug_adc[9] = ADC_DMA_AVERAGE(9);
//	debug_adc[10] = ADC_DMA_AVERAGE(10);
//	debug_adc[11] = ADC_DMA_AVERAGE(11);


//	Rx[0] = debug_adc[0] - debug_adc[1];
//	Rx[1] = debug_adc[2] - debug_adc[3];
//	Rx[2] = debug_adc[4] - debug_adc[5];
//	Rx[3] = debug_adc[6] - debug_adc[7];
//	Rx[4] = debug_adc[8] - debug_adc[9];
//	Rx[5] = debug_adc[10] - debug_adc[11];

	ConvOK = 1;
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

