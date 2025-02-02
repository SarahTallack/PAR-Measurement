/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    spi.h
  * @brief   This file contains all the function prototypes for
  *          the spi.c file
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
#ifndef __SPI_H__
#define __SPI_H__

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern SPI_HandleTypeDef hspi2;

/* USER CODE BEGIN Private defines */
#define CMD_WRITE_STATUS_REG   0x01
#define CMD_PAGE_PROGRAM       0x02
#define CMD_READ_DATA          0x03
#define CMD_WRITE_DISABLE      0x04//not tested
#define CMD_READ_STATUS_REG    0x05
#define CMD_WRITE_ENABLE       0x06
#define CMD_READ_HIGH_SPEED    0x0B//not tested
#define CMD_SECTOR_ERASE       0x20//not tested
#define CMD_BLOCK32K_ERASE     0x52//not tested
#define CMD_RESET_DEVICE       0xF0//<<-different from winbond
#define CMD_READ_ID            0x9F
#define CMD_RELEASE_POWER_DOWN 0xAB//not tested
#define CMD_POWER_DOWN         0xB9//not tested
#define CMD_CHIP_ERASE         0xC7
#define CMD_BLOCK64K_ERASE     0xD8//not tested
/* USER CODE END Private defines */

void MX_SPI2_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif

#endif /* __SPI_H__ */

