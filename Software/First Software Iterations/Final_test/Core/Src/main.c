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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "inttypes.h"
#include "string.h"
#include "stdio.h"
#include <stdbool.h>

#include "Waveshare_AS7341.h"
#include "DEV_Config.h"

#include "Config_Parameters.h"

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

//uint16_t t_meas;
//uint16_t t_int;
//uint8_t gain;
uint8_t ATIME;
uint16_t ASTEP;

int start_meas, start_meas2, end_meas, tot_meas;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void AS7341_Start();
void AS7341_EnableSleepAfterInterrupt(int flag);
void AS7341_ClearStatus();

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
  /* USER CODE BEGIN 2 */

  HAL_TIM_Base_Start_IT(&htim16);

//  t_meas = 5000; 	//ms, set time between measurements. Options are: 100, 500, 1000, 5000, 10000
//  t_int = 10; 		//ms, set integration time of sensor. Options are: 10, 20, 50, 100, 200, 500, 1000, 2000, else default to 50ms
//  gain = 6; 		// [Value: Gain] = [0: 0.5x, 1:1x, 2:2x, 3:4x, 4:8x, 5:16x, 7:64x, 8:128x, 9:256x, 10:512x

//  if (t_int>t_meas)
//  {
//#define t_meas = 100;
//	  t_int = 50;
//	  printf("t_int>t_meas\r\n");
//  }

  //  AS7341_Start(eSyns, true, t_int, gain, false); //void AS7341_Start(eMode_t MODE, bool INT, uint16_t t_int, uint8_t AGAIN, bool LED);
  AS7341_Start();
  printf("Integration Time: %d \r\nMeasurement Time: %d\r\n",t_int,t_meas);

  printf("Time,Channel1,Channel2,Channel3,Channel4,Channel5,Channel6,Channel7,Channel8,Clear,NIR,t_int,t_meas\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  printf("HI");
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//		uint8_t data = I2C_Read_Byte(AS7341_ATIME);
//		printf("ATIME %d\r\n",data);
//		data = I2C_Read_Byte(AS7341_ASTEP_L);
//		printf("ASTEP_L %d\r\n",data);
//		data = I2C_Read_Byte(AS7341_ASTEP_H);
//		printf("ASTEP_H %d\r\n",data);

	  	sModeOneData_t data1;
		sModeTwoData_t data2;

		AS7341_ClearInterrupt();// Interrupt must be cleared

//		start_meas = HAL_GetTick();
		AS7341_startMeasure(eF1F4ClearNIR);
//		start_meas2 = HAL_GetTick();
		AS7341_startMeasure(eF5F8ClearNIR);

		while(!AS7341_MeasureComplete()); // wait for measurement to finish
//		end_meas = HAL_GetTick();
//		tot_meas = end_meas- start_meas;
//		printf("Total measurement time: %d\r\n", tot_meas);
//		tot_meas = end_meas- start_meas2;
//		printf("Total measurement time 2: %d\r\n", tot_meas);
		data1 = AS7341_ReadSpectralDataOne();
		data2 =AS7341_ReadSpectralDataTwo();
		printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d \r\n", data1.channel1, data1.channel2, data1.channel3, data1.channel4, data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR, t_int, t_meas);
//		end_meas = HAL_GetTick();
//		tot_meas = end_meas- start_meas;
//		printf("Total measurement and transmission time: %d\r\n", tot_meas);
//		printf("------------------------------------------------------\r\n");
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
function:	Configure AS7341
info：		Set the sensing mode, enable or disable interrupts, set integration
			time, sensor gain and enable/disable LED.
******************************************************************************/
void AS7341_Start()
{
	printf("Configuring AS7341");
	DEV_ModuleInit();

	AS7341_Init(MODE);
	AS7341_EnableSpectralInterrupt(INT);
	AS7341_AGAIN_config(AGAIN);
	AS7341_EnableLED(LED);

	//t_int = (ATIME + 1)*(ASTEP + 1)*2.78e-6
	//max t_int = 50s
	switch(t_int)
	{
	case 10:
		ATIME = 29;
		ASTEP = 119;
		break;
	case 20:
		ATIME = 29;
		ASTEP = 239;
		break;
	case 50:
		ATIME = 29;
		ASTEP = 599;
		break;
	case 100:
		ATIME = 59;
		ASTEP = 599;
		break;
	case 200:
		ATIME = 59;
		ASTEP = 1198;
		break;
	case 500:
		ATIME = 59;
		ASTEP = 2997;
		break;
	case 1000:
		ATIME = 39;
		ASTEP = 8992;
		break;
	case 2000:
		ATIME = 29;
		ASTEP = 23980;
		break;
	default:
		ATIME = 29;
		ASTEP = 599;
		break;
	}

	AS7341_ATIME_config(ATIME);
	AS7341_ASTEP_config(ASTEP);
	printf("Configuring AS7341 done\r\n ------------------------\r\n");
}

/******************************************************************************
function:	clear status
info：		This register is self-clearing, meaning that writing a "1" to any bit in the
	register clears that status bit.
******************************************************************************/
void AS7341_ClearStatus()
{

//  I2C_Write_Byte(AS7341_STATUS_6,0xff);
	UBYTE data;
	data = I2C_Read_Byte(AS7341_CONTROL);
	data = data | (1);
	I2C_Write_Byte(AS7341_CONTROL,data);
}

/******************************************************************************
function:	enable sleep after interrupt
info：
******************************************************************************/
void AS7341_EnableSleepAfterInterrupt(int flag)
{
  UBYTE data;
  data = I2C_Read_Byte(AS7341_CFG_3);
  if(flag == true)
  {
    data = data | (1<<4);
    I2C_Write_Byte(AS7341_CFG_3,data);
  }
  else{
    data = data & (~(1<<4));
    I2C_Write_Byte(AS7341_CFG_3,data);
  }
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
	  HAL_GPIO_TogglePin(GPIOA, GPIO_Pin);
	  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
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
