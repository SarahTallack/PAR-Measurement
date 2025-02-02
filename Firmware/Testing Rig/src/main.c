/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2024 STMicroelectronics.
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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdlib.h"
#include "string.h"
#include "stdio.h"
#include "math.h"

#include "WS2812.h"

#include "VEML6040.h"
#include "Waveshare_AS7341.h"

#include "TCA9548APWR.h"
#include "stm32l4xx_hal.h"
#include "lcd_stm32f0.h"
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
uint8_t i = 0;
uint8_t ret;
rgb_t VEML_data;

uint8_t ATIME;
uint16_t ASTEP;

sModeOneData_t data1;
sModeTwoData_t data2;

// uint8_t t_int;

uint8_t received_byte;
uint8_t my_variable;

#define RX_BUFFER_SIZE 50

uint8_t rx_buffer[RX_BUFFER_SIZE];
int r, g, b, brightness, mode, pos;
uint16_t rx_index = 0; 

char buffer[100];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void AS7341_Start(void);

void I2C_Scan_Multiplexer(void);
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
  MX_DMA_Init();
  MX_TIM2_Init();
  MX_I2C2_Init();
  MX_USART2_UART_Init();
  /* USER CODE BEGIN 2 */
  // t_int = 40;
  printf("Starting\n\r");
  HAL_UART_Receive_IT(&huart2, &received_byte, 1);
  r = 0;
  g = 0;
  b = 0;
  // lcd_putstring("HELLO");
  RGB(0, 0, 255, 50, 0, 0);

  TCA9548A_Initialise();
  TCA9548A_Channel_Select(&hi2c2, TCA_ADDRESS_1, 3);
  AS7341_Start();

  TCA9548A_Channel_Select(&hi2c2, TCA_ADDRESS_1, 4);
  AS7341_Start();
  
  printf("All devices configured\r\n");  
  HAL_Delay(100);

  // printf("%d, %d, %d\r\n", r, g, b);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    
    // I2C_Scan_Multiplexer();

    

    // printf("Received string: %s\n\r", rx_buffer);


    TCA9548A_Channel_Select(&hi2c2, TCA_ADDRESS_1, 3);

    AS7341_startMeasure(eF1F4ClearNIR);
    data1 = AS7341_ReadSpectralDataOne();
    printf("Channel 3: %d,%d,%d,%d,", data1.channel1, data1.channel2, data1.channel3, data1.channel4);
    
    AS7341_startMeasure(eF5F8ClearNIR);
    data2 = AS7341_ReadSpectralDataTwo();
    // sprintf("Channel 3: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n\r", data1.channel1, data1.channel2, data1.channel3, data1.channel4, data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR);
    // HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    
    printf("%d,%d,%d,%d,%d,%d\n\r", data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR);

    HAL_Delay(10);

    TCA9548A_Channel_Select(&hi2c2, TCA_ADDRESS_1, 4);

    AS7341_startMeasure(eF1F4ClearNIR);
    data1 = AS7341_ReadSpectralDataOne();
    printf("Channel 4: %d,%d,%d,%d,", data1.channel1, data1.channel2, data1.channel3, data1.channel4);

    AS7341_startMeasure(eF5F8ClearNIR);
    data2 = AS7341_ReadSpectralDataTwo();
    // sprintf("Channel 4: %d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n\r", data1.channel1, data1.channel2, data1.channel3, data1.channel4, data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR);
    // HAL_UART_Transmit(&huart2, (uint8_t *)buffer, strlen(buffer), HAL_MAX_DELAY);
    printf("%d,%d,%d,%d,%d,%d\n\r", data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR);

    HAL_Delay(100);

    // TCA9548A_Channel_Select(&hi2c2, TCA_ADDRESS_1, TCA_CHANNEL_3);

    // AS7341_startMeasure(eF1F4ClearNIR);
    // data1 = AS7341_ReadSpectralDataOne();
    // printf("%d,%d,%d,%d,", data1.channel1, data1.channel2, data1.channel3, data1.channel4);

    // AS7341_startMeasure(eF5F8ClearNIR);
    // data2 = AS7341_ReadSpectralDataTwo();
    // printf("%d,%d,%d,%d,%d,%d,", data2.channel5, data2.channel6, data2.channel7, data2.channel8, data2.CLEAR, data2.NIR);

    // TCA9548A_Channel_Select(&hi2c2, TCA_ADDRESS_1, TCA_CHANNEL_3);
    // VEML_data = VEML_GetData(&hi2c2);

	  // printf("%d,%d,%d,%d \r\n", VEML_data.r, VEML_data.g, VEML_data.b, VEML_data.w);
    // RGB(250, 125, 125, 100, 0, 0);

  

    
    
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

void I2C_Scan_Multiplexer(void) {
    char msg[64];
    HAL_StatusTypeDef result;
    uint8_t i, channel;

    printf("Scanning I2C bus through multiplexer...\n");

    // Iterate over all 8 channels of the I2C multiplexer
    for (channel = 0; channel < 8; channel++) {
        // Select the channel by writing to the multiplexer
        uint8_t channel_select = (1 << channel); // Activate only the current channel
        result = HAL_I2C_Master_Transmit(&hi2c2, (TCA_ADDRESS_1 << 1), &channel_select, 1, HAL_MAX_DELAY);

        if (result != HAL_OK) {
            sprintf(msg, "Failed to select channel %d\n", channel);
            printf("%s", msg);
            continue;
        }

        printf("Scanning channel %d...\n", channel);

        // Scan the active channel for devices
        for (i = 1; i < 128; i++) { // I2C addresses are 7-bit (0x01 to 0x7F)
            result = HAL_I2C_IsDeviceReady(&hi2c2, (uint16_t)(i << 1), 1, HAL_MAX_DELAY);
            if (result == HAL_OK) { // If the device responds with an ACK
                sprintf(msg, "Device found on channel %d at address 0x%02X\n", channel, i);
                printf("%s", msg);
            }
        }
    }

    printf("I2C multiplexer scan complete.\n");
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)  // Check if the interrupt is for USART2
    {    
      // Store the received byte in the buffer
      rx_buffer[rx_index++] = received_byte;

      // Check for end of message (newline) or buffer overflow
      if (received_byte == '\n' || rx_index >= RX_BUFFER_SIZE)
      {
          // Process the complete message here
          printf("Received message: %s\n", rx_buffer);
          sscanf((char *)rx_buffer, "%d,%d,%d,%d,%d,%d", &r, &g, &b, &brightness, &mode, &pos);
          RGB(r, g, b, brightness, mode, pos);
          // Reset the buffer index for the next message
          rx_index = 0;
          memset(rx_buffer, 0, RX_BUFFER_SIZE);
      }

      // Re-enable UART to receive the next byte
      HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
      HAL_UART_Receive_IT(&huart2, &received_byte, 1);
    }
}

void AS7341_Start(){

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
