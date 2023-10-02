/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

extern int brightness;

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
#define L9_Pin GPIO_PIN_0
#define L9_GPIO_Port GPIOC
#define L8_Pin GPIO_PIN_1
#define L8_GPIO_Port GPIOC
#define R2_Pin GPIO_PIN_0
#define R2_GPIO_Port GPIOA
#define R3_Pin GPIO_PIN_1
#define R3_GPIO_Port GPIOA
#define USART_TX_Pin GPIO_PIN_2
#define USART_TX_GPIO_Port GPIOA
#define USART_RX_Pin GPIO_PIN_3
#define USART_RX_GPIO_Port GPIOA
#define B_UP_Pin GPIO_PIN_4
#define B_UP_GPIO_Port GPIOA
#define B_UP_EXTI_IRQn EXTI4_IRQn
#define LD2_Pin GPIO_PIN_5
#define LD2_GPIO_Port GPIOA
#define L1_Pin GPIO_PIN_6
#define L1_GPIO_Port GPIOA
#define L2_Pin GPIO_PIN_7
#define L2_GPIO_Port GPIOA
#define L7_Pin GPIO_PIN_0
#define L7_GPIO_Port GPIOB
#define R1_Pin GPIO_PIN_10
#define R1_GPIO_Port GPIOB
#define L4_Pin GPIO_PIN_7
#define L4_GPIO_Port GPIOC
#define L6_Pin GPIO_PIN_8
#define L6_GPIO_Port GPIOA
#define L5_Pin GPIO_PIN_9
#define L5_GPIO_Port GPIOA
#define B_DOWN_Pin GPIO_PIN_10
#define B_DOWN_GPIO_Port GPIOA
#define B_DOWN_EXTI_IRQn EXTI15_10_IRQn
#define TMS_Pin GPIO_PIN_13
#define TMS_GPIO_Port GPIOA
#define TCK_Pin GPIO_PIN_14
#define TCK_GPIO_Port GPIOA
#define SWO_Pin GPIO_PIN_3
#define SWO_GPIO_Port GPIOB
#define L3_Pin GPIO_PIN_6
#define L3_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
