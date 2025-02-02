/*
 * TCA9548APWR.c
 *
 *  Created on: Aug 20, 2024
 *      Author: sarah
 */
#include "TCA9548APWR.h"
#include "stm32l4xx_hal.h"
#include "gpio.h"

void TCA9548A_Initialise(){
	HAL_GPIO_WritePin(I2C_RESET_GPIO_Port, I2C_RESET_Pin, GPIO_PIN_SET);
}

void TCA9548A_Reset(){
	HAL_GPIO_WritePin(I2C_RESET_GPIO_Port, I2C_RESET_Pin, GPIO_PIN_RESET);
	HAL_Delay(50);
	HAL_GPIO_WritePin(I2C_RESET_GPIO_Port, I2C_RESET_Pin, GPIO_PIN_SET);
}

void TCA9548A_Channel_Select(I2C_HandleTypeDef *hi2c, uint8_t TCA_ADDRESS, uint8_t TCA_CHANNEL){
	HAL_I2C_Master_Transmit(hi2c, TCA_ADDRESS << 1, TCA_CHANNEL, 1, HAL_MAX_DELAY);
}
