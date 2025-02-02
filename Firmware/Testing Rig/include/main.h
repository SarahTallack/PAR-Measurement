/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define B1_Pin GPIO_PIN_13
#define B1_GPIO_Port GPIOC
#define B1_EXTI_IRQn EXTI15_10_IRQn
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define LCD_D7_Pin GPIO_PIN_7
#define LCD_D7_GPIO_Port GPIOA
#define CURRENT_SENSOR_Pin GPIO_PIN_4
#define CURRENT_SENSOR_GPIO_Port GPIOC
#define LCD_RS_Pin GPIO_PIN_10
#define LCD_RS_GPIO_Port GPIOB
#define LED_CONTROL_Pin GPIO_PIN_11
#define LED_CONTROL_GPIO_Port GPIOB
#define SCL_MASTER_Pin GPIO_PIN_13
#define SCL_MASTER_GPIO_Port GPIOB
#define SDA_MASTER_Pin GPIO_PIN_14
#define SDA_MASTER_GPIO_Port GPIOB
#define I2C_RESET_Pin GPIO_PIN_15
#define I2C_RESET_GPIO_Port GPIOB
#define LCD_D5_Pin GPIO_PIN_7
#define LCD_D5_GPIO_Port GPIOC
#define LCD_E_Pin GPIO_PIN_8
#define LCD_E_GPIO_Port GPIOA
#define LCD_D4_Pin GPIO_PIN_9
#define LCD_D4_GPIO_Port GPIOA
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define UB2_Pin GPIO_PIN_10
#define UB2_GPIO_Port GPIOC
#define UB1_Pin GPIO_PIN_12
#define UB1_GPIO_Port GPIOC
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define LCD_D6_Pin GPIO_PIN_6
#define LCD_D6_GPIO_Port GPIOB
#define UB3_Pin GPIO_PIN_7
#define UB3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
