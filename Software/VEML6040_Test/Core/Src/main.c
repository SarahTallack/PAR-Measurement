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
#include "VEML6040.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define VEML6040_ADDRESS 0x10 // Slave address of VEML6040 color sensor
#define RED_DATA_REGISTER 0x08 // Register address for red data value
#define GREEN_DATA_REGISTER 0x09 // Register address for green data value
#define BLUE_DATA_REGISTER 0x0A // Register address for blue data value
#define CONFIG_REGISTER 0x00 // Register address for configuration

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

// Create an instance of the VEML6040 sensor
VEML6040_HandleTypeDef veml6040;

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
  uint16_t redData, greenData, blueData;
  uint8_t data[6]; // Array to store RGB data values
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

//void VEML6040_Configuration(void);
//void VEML6040_ReadRedData(uint16_t *redData);
//void VEML6040_ReadRGBData(uint16_t *redData, uint16_t *greenData, uint16_t *blueData);

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
  printf("Configuring device");
  // Configure VEML6040 color sensor
//  VEML6040_Configuration();


  // Check if VEML6040 color sensor is ready
  while (HAL_I2C_IsDeviceReady(&hi2c3, VEML6040_ADDRESS, 10, HAL_MAX_DELAY) != HAL_OK);
  printf("Device ready!");
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {

	  // Check if the sensor exists
	  if (veml6040.sensorExists) {
		  printf("YAY");
		  // Set the sensor configuration (e.g., integration time)
		  VEML6040_setConfiguration(VEML6040_IT_80MS);

		  while (1) {
			  // Read sensor data
			  uint16_t redValue = VEML6040_getRed();
			  uint16_t greenValue = VEML6040_getGreen();
			  uint16_t blueValue = VEML6040_getBlue();
			  uint16_t whiteValue = VEML6040_getWhite();
			  float ambientLight = VEML6040_getAmbientLight();
//			  uint16_t cctValue = VEML6040_getCCT();

			  // Use the sensor data as needed
			  printf("R: %d\r\n", redValue);
			  printf("G: %d\r\n", greenValue);
			  printf("B: %d\r\n", blueValue);
			  printf("W: %d\r\n", whiteValue);
			  printf("Ambient: %f\r\n", ambientLight);
//			  printf("CCT: %d\r\n", cctValue);

			  // Add a delay or use an interrupt-driven approach
			  HAL_Delay(1000);
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
//void VEML6040_ReadRGBData(uint16_t *redData, uint16_t *greenData, uint16_t *blueData)
//{
//  // Send read command to VEML6040 color sensor
//  HAL_I2C_Mem_Read(&hi2c3, VEML6040_ADDRESS, RED_DATA_REGISTER, I2C_MEMADD_SIZE_8BIT, data, 6, HAL_MAX_DELAY);
//
//  // Combine MSB and LSB to get 16-bit red data value
//  *redData = (data[1] << 8) | data[0];
//
//  // Combine MSB and LSB to get 16-bit green data value
//  *greenData = (data[3] << 8) | data[2];
//
//  // Combine MSB and LSB to get 16-bit blue data value
//  *blueData = (data[5] << 8) | data[4];
//}

//void VEML6040_Configuration(void)
//{
//  uint16_t configData = 0x0000; // Configuration data value
//
//  // Set integration time to 100ms (bits 11:6)
//  configData |= (0x08 << 6);
//
//  // Set gain to 1/8 (bits 5:4)
//  configData |= (0x01 << 4);
//
//  // Set power saving mode to normal (bits 3:0)
//  configData |= 0x00;
//
//    // Send write command to VEML6040 color sensor
//    HAL_I2C_Mem_Write(&hi2c3, VEML6040_ADDRESS, CONFIG_REGISTER, I2C_MEMADD_SIZE_8BIT, (uint8_t *)&configData, 2, HAL_MAX_DELAY);
//}

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
