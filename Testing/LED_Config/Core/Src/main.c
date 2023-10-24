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

#include "string.h"
#include "stdio.h"
#include "math.h"

#include "lcd_stm32f0.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef union
{
  struct
  {
    uint8_t b;
    uint8_t r;
    uint8_t g;
  } color;
  uint32_t data;
} PixelRGB_t;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

#define NEOPIXEL_ZERO 	32 	// (ARR+)(0.32) = (100*0.32) = 32
#define NEOPIXEL_ONE	68	// (ARR+)(0.68) = (100*0.68) = 68

#define NUM_PIXELS		3*9
#define DMA_BUFF_SIZE	(NUM_PIXELS * 24) + 1

#define SWEEP 0
#define PI 3.14159265
#define MODE 0 //0 = normal, 1 and 2 for different button functions (single leds)
//#define COLOUR 0 //1 (red), 2 (green), 3 (blue), or 4 (white), else 0 (other)

#define R 255
#define G 0
#define B 128
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

PixelRGB_t pixel[NUM_PIXELS] = {0};
uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};
uint32_t *pBuff;
int i, j, k;
uint16_t stepSize;

uint8_t COLOUR = 7; //1 (red), 2 (green), 3 (blue), or 4 (white), else 0 (other)
float brightness = 4.5;
uint32_t lastDebounceTime = 0;

uint8_t pos = 4;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

void rainbow(void);
void rgb(uint8_t r, uint8_t g, uint8_t b);
void Set_Brightness(float brightness);

void sweep(uint8_t r, uint8_t g, uint8_t b);

void WS2812_send(void);

void LCD_Print();
char buff[16];
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_3);
}

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
  MX_USART2_UART_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */

  k = 0;
  stepSize = 4;

  init_LCD();
  LCD_Print();
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
//	rainbow();

	switch(COLOUR)
	{
	case(1):
			rgb(255, 0, 0); //red
			break;
	case(2):
			rgb(255, 255, 0); //yellow
			break;
	case(3):
			rgb(0, 255, 0); //green
			break;
	case(4):
			rgb(0, 255, 255); //cyan
			break;
	case(5):
			rgb(0, 0, 255); //blue
			break;
	case(6):
			rgb(255, 0, 255); //magenta
			break;
	case(7):
			rgb(255, 255, 255); //white
			break;
	case(8):
			rainbow();
			break;
	default:
			rgb(20, 20, 20);
			break;
	}

	Set_Brightness(brightness);
	WS2812_send();

    HAL_Delay(100);
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
void LCD_Print(){
    lcd_command(CLEAR);

    int b_percent = (brightness*100)/45;
    sprintf(buff, "BRIGHTNESS: %d", b_percent);
    lcd_putstring(buff);

    lcd_command(LINE_TWO); //1 (red), 2 (green), 3 (blue), 4 (white), or 5 (rainbow), else 0 (RGB)
	switch(COLOUR)
	{
	case(1):
			lcd_putstring("COLOUR: RED"); //red
			break;
	case(2):
			lcd_putstring("COLOUR: YELLOW");
			break;
	case(3):
			lcd_putstring("COLOUR: GREEN");
			break;
	case(4):
			lcd_putstring("COLOUR: CYAN");
			break;
	case(5):
			lcd_putstring("COLOUR: BLUE");
			break;
	case(6):
			lcd_putstring("COLOUR: MAGENTA");
			break;
	case(7):
			sprintf(buff, "COLOUR: WHITE %d", (pos+1));
			lcd_putstring(buff);
			break;
	}
}

void Set_Brightness(float brightness)
{
	if (brightness > 45) brightness = 45;
	float angle = 90-brightness;  // in degrees
	angle = angle*PI / 180;  // in rad
	for (int i= 0; i<NUM_PIXELS; i++)
	{
		pixel[i].color.g = pixel[i].color.g/(tan(angle));
		pixel[i].color.r = pixel[i].color.r/(tan(angle));
		pixel[i].color.b = pixel[i].color.b/(tan(angle));
	}
}

void rainbow(void)
{
	for (i = (NUM_PIXELS - 1); i > 0; i--)
	{
		pixel[i].data = pixel[i-1].data;
	}

	if (k < 255)
	{
		pixel[0].color.g = 254 - k; //[254, 0]
		pixel[0].color.r = k + 1;
		pixel[0].color.b = 0;
	}
	else if (k < 510)
	{
		pixel[0].color.g = 0;
		pixel[0].color.r = 509 - k; //[254, 0]
		pixel[0].color.b = k - 254; //[1, 255]
		j++;
	}
	else if (k < 765)
	{
		pixel[0].color.g = k - 509; //[1, 255];
		pixel[0].color.r = 0;
		pixel[0].color.b = 764 - k; //[254, 0]
	}
	k = (k + stepSize) % 765;
}

/******************************************************************************
function:	Set all LEDs to RGB value
info：		Loop through all pixel values and set to RGB colour.
******************************************************************************/
void rgb(uint8_t r, uint8_t g, uint8_t b)
{
#if MODE
	for (i = 0; i < NUM_PIXELS; i++)
	{
		if (i == pos)
		{
			pixel[i].color.r = r;
			pixel[i].color.g = g;
			pixel[i].color.b = b;
		}
//		else if (i == pos - 1)
//		{
//			pixel[i].color.r = r;
//			pixel[i].color.g = g;
//			pixel[i].color.b = b;
//		}
//		else if (i == pos + 1)
//		{
//			pixel[i].color.r = r;
//			pixel[i].color.g = g;
//			pixel[i].color.b = b;
//		}
		else
		{
			pixel[i].color.r = 0;
			pixel[i].color.g = 0;
			pixel[i].color.b = 0;
		}

	}
#else
	for (i = 0; i < NUM_PIXELS; i++)
	{
		pixel[i].color.r = r;
		pixel[i].color.g = g;
		pixel[i].color.b = b;
	}
#endif
}

/******************************************************************************
function:	Set first LED to RGB value then sweep through
info：		Loop through all pixel values and set to RGB colour.
******************************************************************************/
void sweep(uint8_t r, uint8_t g, uint8_t b)
{
    // Move pixel data
    for (int i = (NUM_PIXELS - 1); i > 0; i--)
    {
        pixel[i].data = pixel[i - 1].data;
    }

    // Set the first pixel's color and set the rest to 0
    if (k == 0)
    {
        pixel[0].color.g = g; //[254, 0]
        pixel[0].color.r = r;
        pixel[0].color.b = b;
    }
    else
    {
        pixel[0].color.g = 0;
        pixel[0].color.r = 0;
        pixel[0].color.b = 0;
    }

    k++;
    if (k == NUM_PIXELS)
    {
    	k = 0;
    }
}

void WS2812_send(void)
{
    pBuff = dmaBuffer;
    for (i = 0; i < NUM_PIXELS; i++)
    {
       for (j = 23; j >= 0; j--)
       {
         if ((pixel[i].data >> j) & 0x01)
         {
           *pBuff = NEOPIXEL_ONE;
         }
         else
         {
           *pBuff = NEOPIXEL_ZERO;
         }
         pBuff++;
     }
    }
    dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0!

    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_3, dmaBuffer, DMA_BUFF_SIZE);
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
	uint32_t currentTime = HAL_GetTick();

	if (currentTime - lastDebounceTime >= 100){
		if (GPIO_Pin == B_DOWN_Pin)
		{
			switch(MODE)
			{
			default:
					if (brightness == 0)
					{
						brightness = brightness;
					}
					else
					{
						brightness = brightness - 4.5;
					}
					break;
			case(2):
					pos = pos + 1;
					if (pos > 8) pos = 0;
					break;
			}
		}
		else if (GPIO_Pin == B_UP_Pin)
		{
			switch(MODE)
			{
			default:
					if (brightness == 45)
					{
						brightness = brightness;
					}
					else
					{
						brightness = brightness + 4.5;
					}
					break;
			case(1):
					pos++;
					if (pos == 3) pos = 0;
					break;
			}

		}
		else if (GPIO_Pin == B1_Pin)
		{
			if (COLOUR == 7)
			{
				COLOUR = 1;
			}
			else
			{
				COLOUR = COLOUR + 1;
			}
			HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
		}
		LCD_Print();
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
