/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "LCD_DRIVER.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define TRUE 1
#define FALSE 0

#define FFT_SIZE 32

#define PI 3.14159265359
#define SPEED_OF_LIGHT 3e8

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

/* USER CODE BEGIN PV */
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_ADC1_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

//pomocna funkcija za konverziju float u string
void floatToString(char *buffer, int bufferSize, float value) {
    snprintf(buffer, bufferSize, "%.2f", value);
}

void collectADCSamples(float* samples, uint16_t size) {
	lcdWriteString("Gathering data...");

	HAL_ADCEx_Calibration_Start(&hadc1);

    for (int i = 0; i < size; i++) {
    	HAL_ADC_Start(&hadc1);
        if(HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY) == HAL_OK){

    		samples[i] = HAL_ADC_GetValue(&hadc1);
//    		char myString[10] = "";
//    		floatToString(myString, sizeof(myString), samples[i]);
//    		lcdWriteString(myString);
//    		lcdClear();

        }
		HAL_ADC_Stop(&hadc1);
    }

    lcdClear();
}


void calculateDFT(const float inputSignal[], float realPart[], float imagPart[], int length) {
    lcdWriteString("Converting...");

    for (int k = 0; k < length; k++) {
        realPart[k] = 0.0;
        imagPart[k] = 0.0;

        for (int n = 0; n < length; n++) {
            float angle = -2 * PI * k * n / length;
            float cos_val = cosf(angle);
            float sin_val = sinf(angle);
            realPart[k] += inputSignal[n] * cos_val;
            imagPart[k] -= inputSignal[n] * sin_val;
        }

        // Correct the DC offset for the first element
        if (k == 0) {
            realPart[k] /= length;
            imagPart[k] = 0.0;
        }
    }

    lcdClear();
}


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
  /* USER CODE BEGIN 2 */
  lcdInit();


  //Init FFT handle
  //arm_cfft_radix2_instance_q15 fftInstance;

  //q15_t fftBufIn[FFT_SIZE];
  //q15_t fftBufOut[FFT_SIZE/2];

  char distanceString[20];
  float inputSignal[FFT_SIZE];

  float realPart[FFT_SIZE];
  float imagPart[FFT_SIZE];

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	  collectADCSamples(inputSignal, FFT_SIZE);

//	  arm_cfft_radix2_init_q15(&fftInstance, FFT_SIZE, 0, 1);
//	  arm_cfft_radix2_q15(&fftInstance, fftBufIn);
//	  arm_cmplx_mag_q15(fftBufIn, fftBufOut, FFT_SIZE);

	  calculateDFT(inputSignal, realPart, imagPart, FFT_SIZE);


	  float amplitudeSpectrum[FFT_SIZE / 2];
	  for (int i = 0; i < FFT_SIZE / 2; i++) {
	      amplitudeSpectrum[i] = sqrt(realPart[i] * realPart[i] + imagPart[i] * imagPart[i]);
	  }


	  float maxAmplitude = amplitudeSpectrum[0];
	  int peakIndex = 0;
	  for (int i = 1; i < FFT_SIZE / 2; i++) {
	      if (amplitudeSpectrum[i] > maxAmplitude) {
	          maxAmplitude = amplitudeSpectrum[i];
	          peakIndex = i;

	      }
	  }


	  float frequencySlope = 200;

	  // Constants
	  uint32_t ADC_Clock_Frequency = 8000000;
	  uint32_t ADC_Prescaler = 2;
	  uint32_t ADC_Sampling_Time = 239.5;

	  float Sampling_Frequency_Hz;
	  Sampling_Frequency_Hz = (float)(ADC_Clock_Frequency / ADC_Prescaler) / ADC_Sampling_Time;

	  float peakFrequency = peakIndex * (Sampling_Frequency_Hz / FFT_SIZE);
	  float targetRange = (((peakFrequency/100) * SPEED_OF_LIGHT) / (2*frequencySlope*1000000));

	  floatToString(distanceString, sizeof(distanceString), targetRange);

	  lcdClear();
	  lcdWriteString(distanceString);
	  lcdWriteString("m");

//	  floatToString(distanceString, sizeof(distanceString), peakFrequency);
//
//	  lcdClear();
//	  lcdWriteString(distanceString);
//	  lcdWriteString("Hz");

	  HAL_Delay(1500);
	  lcdClear();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_NONE;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_HSI;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV2;
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

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_8;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

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
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_12|GPIO_PIN_13, GPIO_PIN_RESET);

  /*Configure GPIO pins : PA0 PA1 PA2 PA3
                           PA4 PA5 PA6 PA7
                           PA12 PA13 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2|GPIO_PIN_3
                          |GPIO_PIN_4|GPIO_PIN_5|GPIO_PIN_6|GPIO_PIN_7
                          |GPIO_PIN_12|GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

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
