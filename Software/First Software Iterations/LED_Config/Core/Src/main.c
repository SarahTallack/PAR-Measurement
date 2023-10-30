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
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define MAX_LED 27
#define USE_BRIGHTNESS 1

#define PI 3.14159265
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t LED_Data[MAX_LED][4];
uint8_t LED_Mod[MAX_LED][4];	//for brightness

uint16_t pwmData[(24*MAX_LED)+48];
uint8_t datasentflag = 0;
					// Violet,		Indigo,		Blue,		Cyan,		Green,			Yellow,		Orange,			Red,		Pink,
//int rainbow[9][3] = {{148, 0, 211},{75, 0, 130},{0, 0, 255},{0,255,255},{0, 255, 92},{0, 255, 0},{255, 255, 0},{255, 0, 0},{255, 192, 203}};
int rainbow[MAX_LED][3];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void Set_LED(int LEDnum, int Red, int Green, int Blue);
void Set_Brightness(int brightness);

void WS2812_Send (void);

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
  MX_USART2_UART_Init();
  MX_TIM1_Init();
  /* USER CODE BEGIN 2 */

  Set_LED(0, 61, 0, 61); //380nm
  Set_LED(1, 83, 0, 181); //400nm
  Set_LED(2, 0, 46, 255); //450nm

  Set_LED(3, 0, 255, 92); //500nm

  Set_LED(4, 163, 255, 0); //550nm
  Set_LED(5, 255, 190, 0); //600nm
  Set_LED(6, 255, 0, 0); //650nm

  Set_LED(7, 255, 0, 0); //700nm

  Set_LED(8, 161, 0, 0); //750nm

  // Fill each row with {0, 0, 255}
  for (int i = 0; i < MAX_LED; i++) {
      rainbow[i][0] = 255;		//blue
      rainbow[i][1] = 0;		//red
      rainbow[i][2] = 0;		//green
  }

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */

	Set_Brightness(22);

	for (int i = 0; i<MAX_LED; i++)
	{
		Set_LED(i,rainbow[i][1],rainbow[i][2],rainbow[i][3]);
	}
	WS2812_Send();
	HAL_Delay (500);

//	int temp[3];
//	for (j = 0; j < 3; j++)
//	{
//		temp[j] = rainbow[0][j];
//	}
//
//	for(i=0; i<9; i++) //all rows
//	{
//		for (j = 0; j < 3; j++)
//		{
//			rainbow[i+1][j]  = temp[j];
//		}
//
//	}

	 int temp[3];
	    int i;

	    // Store the last value in a temporary array
	    for (i = 0; i < 3; i++) {
	        temp[i] = rainbow[MAX_LED -1][i];
	    }

	    // Shift every value up by one position
	    for (i = MAX_LED - 1; i > 0; i--) {
	        for (int j = 0; j < 3; j++) {
	            rainbow[i][j] = rainbow[i - 1][j];
	        }
	    }

	    // Move the temporary array to the front
	    for (i = 0; i < 3; i++) {
	        rainbow[0][i] = temp[i];
	    }
	HAL_GPIO_TogglePin(GPIOA, LD2_Pin);

//	  for (int i=0; i<46; i++)
//	  	  {
//	  		  Set_Brightness(i);
//	  		  WS2812_Send();
//	  		  HAL_Delay (50);
//	  	  }
//
//	  	  for (int i=45; i>=0; i--)
//	  	  {
//	  		  Set_Brightness(i);
//	  		  WS2812_Send();
//	  		  HAL_Delay (50);
//	  	  }
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
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
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
}

/* USER CODE BEGIN 4 */

void Set_LED(int LEDnum, int Red, int Green, int Blue)
{
	LED_Data[LEDnum][0] = LEDnum;
	LED_Data[LEDnum][1] = Green;
	LED_Data[LEDnum][2] = Red;
	LED_Data[LEDnum][3] = Blue;
}

void Set_Brightness(int brightness)
{
	#if USE_BRIGHTNESS

		if (brightness > 45) brightness = 45;
		for (int i=0; i<MAX_LED; i++)
		{
			LED_Mod[i][0] = LED_Data[i][0];
			for (int j=1; j<4; j++)
			{
				float angle = 90-brightness;  // in degrees
				angle = angle*PI / 180;  // in rad
				LED_Mod[i][j] = (LED_Data[i][j])/(tan(angle));
			}
		}

	#endif
}

void WS2812_Send (void)
{
	uint32_t indx=0;
	uint32_t color;


	for (int i= 0; i<MAX_LED; i++)
	{
		#if USE_BRIGHTNESS
				color = ((LED_Mod[i][1]<<16) | (LED_Mod[i][2]<<8) | (LED_Mod[i][3]));
		#else
				color = ((LED_Data[i][1]<<16) | (LED_Data[i][2]<<8) | (LED_Data[i][3]));
		#endif

		for (int i=23; i>=0; i--)
		{
			if (color&(1<<i))
			{
				pwmData[indx] = 68;  // 2/3 of 90
			}

			else pwmData[indx] = 32;  // 1/3 of 90

			indx++;
		}

	}

	for (int i=0; i<48; i++)
	{
		pwmData[indx] = 0;
		indx++;
	}

	HAL_TIM_PWM_Start_DMA(&htim1, TIM_CHANNEL_1, (uint32_t *)pwmData, indx);
	while (!datasentflag){};
	datasentflag = 0;
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	HAL_TIM_PWM_Stop_DMA(&htim1, TIM_CHANNEL_1);
	datasentflag=1;
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
