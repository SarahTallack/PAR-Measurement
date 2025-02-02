/*
 * TCA9548APWR.h
 *
 *  Created on: Aug 20, 2024
 *      Author: sarah
 */

#ifndef INC_TCA9548APWR_H_
#define INC_TCA9548APWR_H_

//====================================================================
// REGISTER VALUES
//====================================================================
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "usart.h"
#include "i2c.h"

#define TCA_ADDRESS_1				(0x70)
#define TCA_ADDRESS_2				(0x71)
#define TCA_ADDRESS_3				(0x72)
#define TCA_ADDRESS_4				(0x73)
#define TCA_ADDRESS_5				(0x74)
#define TCA_ADDRESS_6				(0x75)
#define TCA_ADDRESS_7				(0x76)
#define TCA_ADDRESS_8				(0x77)

#define TCA_CHANNEL_1	         	(0x01)
#define TCA_CHANNEL_2         	(0x02)
#define TCA_CHANNEL_3        	 	(0x04)
#define TCA_CHANNEL_4	         	(0x08)
#define TCA_CHANNEL_5       	  (0x10)
#define TCA_CHANNEL_6      		  (0x20)
#define TCA_CHANNEL_7	      	  (0x40)
#define TCA_CHANNEL_8      		 	(0x80)

void TCA9548A_Initialise();
void TCA9548A_Reset();
void TCA9548A_Channel_Select(I2C_HandleTypeDef *hi2c, uint8_t TCA_ADDRESS, uint8_t TCA_CHANNEL);
#endif /* INC_TCA9548APWR_H_ */
