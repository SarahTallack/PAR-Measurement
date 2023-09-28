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

//#include "test.h"
#include "Waveshare_AS7341.h"
#include "DEV_Config.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
FILE *fptr;
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
int start_meas, end_meas, tot_meas;
uint8_t SAI, Clear_status_bit, SAI_Active;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

void AS7341_EnableSleepAfterInterrupt(int flag);
void AS7341_ClearStatus();
/* USER CODE BEGIN PFP */

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
  MX_USART2_UART_Init();
  MX_TIM16_Init();
  MX_I2C3_Init();
  /* USER CODE BEGIN 2 */

//  printf("AS7341 Spectral Sensor Code...\r\n");
  DEV_ModuleInit();

  AS7341_Init(eSyns);
  AS7341_EnableSpectralInterrupt(true);
//  AS7341_EnableSleepAfterInterrupt(false);
  AS7341_ATIME_config(29);
  AS7341_ASTEP_config(599);
  // gives t_int = (ATIME + 1)*(ASTEP + 1)*278e-6 = 50ms
  // ADC_fullscale = (ATIME + 1)*(ASTEP + 1)
  AS7341_AGAIN_config(6);
  AS7341_EnableLED(false);// LED Enable

  // Start timer
  HAL_TIM_Base_Start_IT(&htim16);

  start_meas = HAL_GetTick();

//  char buf[0x100];
//  snprintf(buf, sizeof(buf), "C:\\Users\\sarah\\Documents\\1. UNIVERSITY\\University\\EEE4022S\\PAR-Measurement\\Testing\\%d.txt", start_meas);
//  FILE *f = fopen(buf, "w");

//  fptr = fopen("C:\\Users\\sarah\\Documents\\1. UNIVERSITY\\University\\EEE4022S\\PAR-Measurement\\Testing\\newprogram.txt","w");

  printf("Time,Channel1,Channel2,Channel3,Channel4,Channel5,Channel6,Channel7,Channel8,Clear,NIR\r\n");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	AS7341_ControlLed(true,10);//Turn on or off the LED and set the brightness of the LED
	sModeOneData_t data1;
	sModeTwoData_t data2;

//	AS7341_ClearStatus();
	AS7341_ClearInterrupt();//Interrupt must be cleared

//	printf("~~~~~~~~~~~~~~~~~~~~~~~~~~\r\n");
//
//	if(DEV_Digital_Read(INT_PIN) == 1)
//		printf("pinINT is high\r\n");
//	else
//		printf("pinINT is low\r\n");
//	printf("--------------------------\r\n");
//
//	SAI = I2C_Read_Byte(0xAC);
//	Clear_status_bit = I2C_Read_Byte(0xFA);
//	SAI_Active = I2C_Read_Byte(0xA7);
//
//	printf("SAI: %x\r\nclear status bit: %x\r\nSAI_Active: %x\r\n", SAI, Clear_status_bit, SAI_Active);

//	printf("--------------------------\r\n");

//	printf("Waiting for the GPIO signal...\r\n");
	start_meas = HAL_GetTick();
	AS7341_startMeasure(eF1F4ClearNIR);
	AS7341_startMeasure(eF5F8ClearNIR);

	while(!AS7341_MeasureComplete());
	end_meas = HAL_GetTick();
	tot_meas = end_meas- start_meas;
//	printf("Total measurement time: %d\r\n", tot_meas);

//	AS7341_startMeasure(eF1F4ClearNIR);
	data1 = AS7341_ReadSpectralDataOne();

//	printf("channel1(405-425nm):\r\n");
//	printf("%d\r\n",data1.channel1);
//	printf("channel2(435-455nm):\r\n");
//	printf("%d\r\n",data1.channel2);
//	printf("channel3(470-490nm):\r\n");
//	printf("%d\r\n",data1.channel3);
//	printf("channel4(505-525nm):\r\n");
//	printf("%d\r\n",data1.channel4);
//	printf(data1, data2);

//	AS7341_ClearInterrupt();//Interrupt must be cleared

	data2 =AS7341_ReadSpectralDataTwo();
//	printf("channel5(545-565nm):\r\n");
//	printf("%d\r\n",data2.channel5);
//	printf("channel6(580-600nm):\r\n");
//	printf("%d\r\n",data2.channel6);
//	printf("channel7(620-640nm):\r\n");
//	printf("%d\r\n",data2.channel7);
//	printf("channel8(670-690nm):\r\n");
//	printf("%d\r\n",data2.channel8);
//	printf("Clear:\r\n");
//	printf("%d\r\n",data2.CLEAR);
//	printf("NIR:\r\n");
//	printf("%d\r\n",data2.NIR);

	printf("%d,%d,%d,%d,%d,%d,%d,%d,%d,%d \r\n", data1.channel1, data1.channel2, data1.channel3, data1.channel4, data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR);

//	if(DEV_Digital_Read(INT_PIN) == 1)
//		printf("pinINT is high\r\n");
//	else
//		printf("pinINT is low\r\n");
//	printf("--------------------------\r\n");
//	SAI = I2C_Read_Byte(0xAC);
//	Clear_status_bit = I2C_Read_Byte(0xFA);
//	SAI_Active = I2C_Read_Byte(0xA7);
//
//	printf("SAI: %x\r\nclear status bit: %x\r\nSAI_Active: %x\r\n", SAI, Clear_status_bit, SAI_Active);
//
//	printf("--------------------------\r\n");
//	printf("--------------------------\r\n");

//	DEV_Delay_ms(500);

//	  pinINT_test();

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


// Callback: timer has rolled over
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  // Check which version of the timer triggered this callback and toggle LED
  if (htim == &htim16 )
  {

//	  AS7341_ClearStatus();
//	  AS7341_ClearInterrupt();
//	  SAI = I2C_Read_Byte(0xAC);
//	  Clear_status_bit = I2C_Read_Byte(0xFA);
//	  SAI_Active = I2C_Read_Byte(0xA7);
//
//	  printf("SAI: %x\r\nclear status bit: %x\r\nSAI_Active: %x\r\n", SAI, Clear_status_bit, SAI_Active);

	  HAL_GPIO_TogglePin(GPIOA, GPIO_Pin);
	  HAL_GPIO_TogglePin(GPIOA, LD2_Pin);
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
