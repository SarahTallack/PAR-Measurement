/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * Author: Sarah Tallack
  * Date: 28 September 2023
  * Description: This program is designed to read and transmit sensor data from
  * both the VEML6040 and AS7341 sensors
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "inttypes.h"
#include "string.h"
#include "stdio.h"
#include "stdbool.h"

#include "Config_Parameters.h"

#include "VEML6040.h"

#include "Waveshare_AS7341.h"
#include "DEV_Config.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

uint8_t ATIME;
uint16_t ASTEP;

uint16_t redValue, greenValue, blueValue, whiteValue;

uint16_t rgbw_data[4];
uint32_t lastDebounceTime = 0;

VEML6040_Handle veml6040;

uint32_t t_int = 40;
int i = 0;
int j = 0;


sModeOneData_t data1;
sModeTwoData_t data2;

rgb_t VEML_data;


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void VEML6040_Read_RGBW(uint16_t *rgbw_data);
void VEML6040_Start();
void AS7341_Start();

void start_meas();
void data_output();
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
  HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_I2C3_Init();
  MX_I2C1_Init();
  /* USER CODE BEGIN 2 */

  VEML6040_Start();
  AS7341_Start();

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
	  /*
	  while (HAL_GPIO_ReadPin(B1_GPIO_Port, B1_Pin) == GPIO_PIN_SET) // Wait while the button is not pressed
	  {
		  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		  HAL_Delay(100);
	  }
	  start_meas();*/
	  data_output();
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

/******************************************************************************
function:	Configure VEML6040
info：		Set the I2C channel, and set integration time
******************************************************************************/
void VEML6040_Start()
{
	  uint8_t VEML6040_IT = VEML6040_IT_160MS;
//	  printf("Configuring VEML6040");
	  // Initialize the VEML6040 sensor
		switch(t_int)
		{
		case 40:
			VEML6040_IT = VEML6040_IT_40MS;
			break;
		case 80:
			VEML6040_IT = VEML6040_IT_80MS;
			break;
		case 160:
			VEML6040_IT = VEML6040_IT_160MS;
			break;
		case 320:
			VEML6040_IT = VEML6040_IT_320MS;
			break;
		case 640:
			VEML6040_IT = VEML6040_IT_640MS;
			break;
		case 1280:
			VEML6040_IT = VEML6040_IT_1280MS;
			break;
		}
	  VEML6040_SetConfiguration(&hi2c1, VEML6040_IT | VEML6040_AF_AUTO | VEML6040_SD_ENABLE);
	  // Set the sensor configuration (e.g., VEML6040_IT_160MS)
//	  printf("Configuring VEML6040 done\r\n ------------------------\r\n");
}

/******************************************************************************
function:	Configure AS7341
info：		Set the sensing mode, enable or disable interrupts, set integration
			time, sensor gain and enable/disable LED.
******************************************************************************/
void AS7341_Start()
{
	DEV_ModuleInit();

	AS7341_Init(MODE);
	AS7341_EnableSpectralInterrupt(INT);
	AS7341_AGAIN_config(AGAIN);
	AS7341_EnableLED(LED_AS7341);

	/* t_int = (ATIME + 1)*(ASTEP + 1)*2.78e-6
	 * max t_int = 50s */
	switch(t_int)
	{
	case 40:
		ATIME = 29;
		ASTEP = 479;
		break;
	case 80:
		ATIME = 59;
		ASTEP = 479;
		break;
	case 160:
		ATIME = 59;
		ASTEP = 958;
		break;
	case 320:
		ATIME = 59;
		ASTEP = 1917;
		break;
	case 640:
		ATIME = 29;
		ASTEP = 7673;
		break;
	case 1280:
		ATIME = 119;
		ASTEP = 3826;
		break;
	default:
		ATIME = 29; // 50 ms
		ASTEP = 599;
		break;
	}

	AS7341_ATIME_config(ATIME);
	AS7341_ASTEP_config(ASTEP);
}

/******************************************************************************
function:	Toggle pins when timer has rolled over
info：		Callback: timer as rolled over. Toggle LED and GPIO output
******************************************************************************/
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim16 )
  {
	  HAL_GPIO_TogglePin(AS7341_GPIO_GPIO_Port, AS7341_GPIO_Pin);
	  HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
  }

  switch(t_meas)
    {
    case(100):
		TIM16->ARR = 100-1;
    	  break;
    case(500):
		TIM16->ARR = 500-1;
    	  break;
    case(1000):
		TIM16->ARR = 1000-1;
    	  break;
    case(5000):
		TIM16->ARR = 5000-1;
    	  break;
    case(10000):
		TIM16->ARR = 10000-1;
    	  break;
    }
}

void start_meas()
{
	t_int = 40;
	VEML6040_Start();
	AS7341_Start();
	for (i = 1; i <= 6; i++)
	{
		j = 0;
		while (j < 10)
		{
			AS7341_startMeasure(eF1F4ClearNIR);
			data1 = AS7341_ReadSpectralDataOne();
			printf("%d,%d,%d,%d,", data1.channel1, data1.channel2, data1.channel3, data1.channel4);

			AS7341_startMeasure(eF5F8ClearNIR);
			data2 =AS7341_ReadSpectralDataTwo();
			printf("%d,%d,%d,%d,%d,%d,", data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR);

			VEML_data = VEML_GetData(&hi2c1);
			printf("%d,%d,%d,%d,%ld\r\n", VEML_data.r, VEML_data.g, VEML_data.b, VEML_data.w, t_int);
			HAL_Delay(t_int/4);
			j++;
		}
		HAL_Delay(200);
		t_int = t_int*2;
		VEML6040_Start();
		AS7341_Start();
	}
//	j = 0;
//	while (j < 20)
//	{
//		AS7341_startMeasure(eF1F4ClearNIR);
//		data1 = AS7341_ReadSpectralDataOne();
//		printf("%d,%d,%d,%d,", data1.channel1, data1.channel2, data1.channel3, data1.channel4);
//
//		AS7341_startMeasure(eF5F8ClearNIR);
//		data2 =AS7341_ReadSpectralDataTwo();
//		printf("%d,%d,%d,%d,%d,%d,", data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR);
//
//		VEML_data = VEML_GetData(&hi2c1);
//		printf("%d,%d,%d,%d,%ld\r\n", VEML_data.r, VEML_data.g, VEML_data.b, VEML_data.w, t_int);
//		HAL_Delay(t_int/4);
//		j++;
//	}
//	HAL_Delay(200);
}

void data_output()
{
	AS7341_startMeasure(eF1F4ClearNIR);
	data1 = AS7341_ReadSpectralDataOne();
	printf("%d,%d,%d,%d,", data1.channel1, data1.channel2, data1.channel3, data1.channel4);

	AS7341_startMeasure(eF5F8ClearNIR);
	data2 =AS7341_ReadSpectralDataTwo();
	printf("%d,%d,%d,%d,%d,%d,", data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR);

	VEML_data = VEML_GetData(&hi2c1);
	printf("%d,%d,%d,%d,%ld\r\n", VEML_data.b, VEML_data.g, VEML_data.r, VEML_data.w, t_int);

	HAL_Delay(100);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t currentTime = HAL_GetTick();

	if (currentTime - lastDebounceTime >= 100){
		if (GPIO_Pin == B1_Pin)
		{
			if (t_int == 1280)
			{
				t_int = 40;
			}
			else
			{
				t_int = t_int*2;
			}
			VEML6040_Start();
			AS7341_Start();
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
		lastDebounceTime = currentTime;
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
