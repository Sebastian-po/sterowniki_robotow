/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "dfsdm.h"
#include "dma.h"
#include "lcd.h"
#include "sai.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stm32l476g_discovery.h"
#include "../Components/cs43l22/cs43l22.h"
#include "app.h"
#include "arm_math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
#define SaturaLH(N, L, H) (((N)<(L))?(L):(((N)>(H))?(H):(N)))
#define FFT_SAMPLES 1024
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
AUDIO_DrvTypeDef *audio_drv;
int32_t RecBuff[2048];
int16_t PlayBuff[4096];
uint64_t Rx_Data[4096];
uint32_t DmaRecHalfBuffCplt	= 0;
uint32_t DmaRecBuffCplt		= 0;
uint32_t PlaybackStarted	= 0;
uint8_t volume = 80;
uint32_t i;
float FFTInBuffer[FFT_SAMPLES];
float FFTOutBuffer[FFT_SAMPLES];

arm_rfft_fast_instance_f32 FFTHandler;

volatile uint8_t SamplesReady;

uint8_t OutFreqArray[10];

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void AudioDriver_Init(void);
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
  MX_DMA_Init();
  MX_DFSDM1_Init();
  MX_SAI1_Init();
  MX_LCD_Init();
  /* USER CODE BEGIN 2 */

  APP_Init();

  AudioDriver_Init();
  arm_rfft_fast_init_f32(&FFTHandler, FFT_SAMPLES);

  /* Start DFSDM conversions */
  if(HAL_OK != HAL_DFSDM_FilterRegularStart_DMA(&hdfsdm1_filter0, RecBuff, 2048))
  {
    Error_Handler();
  }

  if(0 != audio_drv->Play(AUDIO_I2C_ADDRESS, (uint16_t *) &PlayBuff[0], 4096))
  {
	Error_Handler();
  }
  if(0 != audio_drv->Pause(AUDIO_I2C_ADDRESS))
  {
	Error_Handler();
  }
  if(HAL_OK != HAL_SAI_Transmit_DMA(&hsai_BlockA1, (uint8_t *) &PlayBuff[0], 4096))
  {
	Error_Handler();
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	 /* Store values on Play buff */
	 for(i = 0; i < 2048; i++)
	 {
	  	PlayBuff[2*i]     = SaturaLH((RecBuff[i] >> 8), -32768, 32767);
	  	PlayBuff[(2*i)+1] = PlayBuff[2*i];

	    if(SamplesReady == 1)
	    {
	    	  SamplesReady = 0;

	    	  for(i = 0; i < FFT_SAMPLES; i++)
	    	  {
	    		  FFTInBuffer[i] =  (float)PlayBuff[i];
	    	  }

	    	  CalculateFFT();
	    }
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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI|RCC_OSCILLATORTYPE_MSI;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.MSIState = RCC_MSI_ON;
  RCC_OscInitStruct.MSICalibrationValue = 0;
  RCC_OscInitStruct.MSIClockRange = RCC_MSIRANGE_6;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_MSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 40;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC|RCC_PERIPHCLK_SAI1
                              |RCC_PERIPHCLK_DFSDM1;
  PeriphClkInit.Sai1ClockSelection = RCC_SAI1CLKSOURCE_PLLSAI1;
  PeriphClkInit.Dfsdm1ClockSelection = RCC_DFSDM1CLKSOURCE_PCLK;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  PeriphClkInit.PLLSAI1.PLLSAI1Source = RCC_PLLSOURCE_MSI;
  PeriphClkInit.PLLSAI1.PLLSAI1M = 1;
  PeriphClkInit.PLLSAI1.PLLSAI1N = 48;
  PeriphClkInit.PLLSAI1.PLLSAI1P = RCC_PLLP_DIV17;
  PeriphClkInit.PLLSAI1.PLLSAI1Q = RCC_PLLQ_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1R = RCC_PLLR_DIV2;
  PeriphClkInit.PLLSAI1.PLLSAI1ClockOut = RCC_PLLSAI1_SAI1CLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
uint32_t Flash_Write_Data (uint32_t StartPageAddress, int64_t *Data, int numofwords)
{
	static FLASH_EraseInitTypeDef EraseInitStruct;
	uint32_t PAGEError;
	int sofar=0;

	/* Unlock the Flash to enable the flash control register access *************/
   HAL_FLASH_Unlock();

   /* Fill EraseInit structure*/
   EraseInitStruct.TypeErase = FLASH_TYPEERASE_PAGES;
   //EraseInitStruct.Banks = FLASH_BANK_BOTH;
   EraseInitStruct.Page = StartPageAddress;
   EraseInitStruct.NbPages = (numofwords * 8/FLASH_PAGE_SIZE)+1;

   if (HAL_FLASHEx_Erase(&EraseInitStruct, &PAGEError) != HAL_OK)
   {
	 /*Error occurred while page erase.*/
	  return -1;
   }

   /* Program the user Flash area word by word*/
   while (sofar < numofwords)
   {
	 if (StartPageAddress >= 0x080FFFFF)
	 {
		 return -1;
	 }
	 if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_DOUBLEWORD, StartPageAddress, Data[sofar]) == HAL_OK)
	 {
		 StartPageAddress += 8;  // use StartPageAddress += 2 for half word and 8 for double word
		 sofar++;
	 }
	 else
	 {
	   /* Error occurred while writing data in Flash memory*/
		 return -1;
	 }
   }

   /* Lock the Flash to disable the flash control register access (recommended
	  to protect the FLASH memory against possible unwanted operation) *********/
   HAL_FLASH_Lock();

   return StartPageAddress;
}

void Flash_Read_Data (uint32_t StartPageAddress, int64_t *RxBuf, int numofwords)
{
	while (1)
	{
		*RxBuf = *(__IO int64_t *)StartPageAddress;
		StartPageAddress += 8;
		RxBuf++;
		if (!(--numofwords - 1))
			break;
	}
}
float complexABS(float real, float compl) {
	return sqrtf(real*real+compl*compl);
}
void CalculateFFT(void)
{
	arm_rfft_fast_f32(&FFTHandler, FFTInBuffer, FFTOutBuffer, 0);

	int Freqs[FFT_SAMPLES];
	int FreqPoint = 0;
	int Offset = 45; // variable noise floor offset

	// calculate abs values and linear-to-dB
	for (int i = 0; i < FFT_SAMPLES; i = i+2)
	{
		Freqs[FreqPoint] = (int)(20*log10f(complexABS(FFTOutBuffer[i], FFTOutBuffer[i+1]))) - Offset;

		if(Freqs[FreqPoint] < 0)
		{
			Freqs[FreqPoint] = 0;
		}
		FreqPoint++;

	}
	OutFreqArray[0] = (uint8_t)Freqs[1]; // 22 Hz
	OutFreqArray[1] = (uint8_t)Freqs[2]; // 63 Hz
	OutFreqArray[2] = (uint8_t)Freqs[3]; // 125 Hz
	OutFreqArray[3] = (uint8_t)Freqs[6]; // 250 Hz
	OutFreqArray[4] = (uint8_t)Freqs[12]; // 500 Hz
	OutFreqArray[5] = (uint8_t)Freqs[23]; // 1000 Hz
	OutFreqArray[6] = (uint8_t)Freqs[51]; // 2200 Hz
	OutFreqArray[7] = (uint8_t)Freqs[104]; // 4500 Hz
	OutFreqArray[8] = (uint8_t)Freqs[207]; // 9000 Hz
	OutFreqArray[9] = (uint8_t)Freqs[344]; // 15000 Hz

	// write to flash
	Flash_Write_Data(0x08080000, OutFreqArray, 10);
}

void AudioDriver_Init(void) {

  /* Enable SAI to generate clock used by audio driver */
  __HAL_SAI_ENABLE(&hsai_BlockA1);

  /* Initialize audio driver */
  if(CS43L22_ID != cs43l22_drv.ReadID(AUDIO_I2C_ADDRESS))
  {
	Error_Handler();
  }
  audio_drv = &cs43l22_drv;
  audio_drv->Reset(AUDIO_I2C_ADDRESS);
  if(0 != audio_drv->Init(AUDIO_I2C_ADDRESS, OUTPUT_DEVICE_HEADPHONE, volume, AUDIO_FREQUENCY_44K))
  {
	Error_Handler();
  }
}



/**
  * @brief  Half regular conversion complete callback.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
void HAL_DFSDM_FilterRegConvHalfCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  DmaRecHalfBuffCplt = 1;
}

/**
  * @brief  Regular conversion complete callback.
  * @note   In interrupt mode, user has to read conversion value in this function
            using HAL_DFSDM_FilterGetRegularValue.
  * @param  hdfsdm_filter : DFSDM filter handle.
  * @retval None
  */
void HAL_DFSDM_FilterRegConvCpltCallback(DFSDM_Filter_HandleTypeDef *hdfsdm_filter)
{
  DmaRecBuffCplt = 1;
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
	while (1)
	{
	  /* Toggle LED4 with a period of one second */
	  HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);
	  HAL_Delay(1000);
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
