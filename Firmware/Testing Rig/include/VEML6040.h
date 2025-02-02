#ifndef VEML6040_H
#define VEML6040_H

#include "stm32l4xx_hal.h" // Include the appropriate STM32 HAL library

// VEML6040 I2C ADDRESS
#define VEML6040_I2C_ADDRESS   0x10

// REGISTER CONF (00H) SETTINGS
#define VEML6040_IT_40MS       0x00
#define VEML6040_IT_80MS       0x10
#define VEML6040_IT_160MS      0x20
#define VEML6040_IT_320MS      0x30
#define VEML6040_IT_640MS      0x40
#define VEML6040_IT_1280MS     0x50

#define VEML6040_TRIG_DISABLE  0x00
#define VEML6040_TRIG_ENABLE   0x04

#define VEML6040_AF_AUTO       0x00
#define VEML6040_AF_FORCE      0x02

#define VEML6040_SD_ENABLE     0x00
#define VEML6040_SD_DISABLE    0x01

// COMMAND CODES
#define COMMAND_CODE_CONF      0x00
#define COMMAND_CODE_RED       0x08
#define COMMAND_CODE_GREEN     0x09
#define COMMAND_CODE_BLUE      0x0A
#define COMMAND_CODE_WHITE     0x0B

// G SENSITIVITY
#define VEML6040_GSENS_40MS    0.25168
#define VEML6040_GSENS_80MS    0.12584
#define VEML6040_GSENS_160MS   0.06292
#define VEML6040_GSENS_320MS   0.03146
#define VEML6040_GSENS_640MS   0.01573
#define VEML6040_GSENS_1280MS  0.007865

typedef struct {
    I2C_HandleTypeDef *hi2c;
    uint8_t lastConfiguration;
} VEML6040_Handle;

typedef struct{
  uint16_t r;/**<red channel data>*/
  uint16_t g;/**<green channel data>*/
  uint16_t b;/**<blue channel data>*/
  uint16_t w;/**<white channel data>*/
}rgb_t;

void VEML6040_Init(I2C_HandleTypeDef *hi2c);
void VEML6040_SetConfiguration(I2C_HandleTypeDef *hi2c, uint8_t config);
uint16_t VEML6040_GetRed(I2C_HandleTypeDef *hi2c);
uint16_t VEML6040_GetGreen(I2C_HandleTypeDef *hi2c);
uint16_t VEML6040_GetBlue(I2C_HandleTypeDef *hi2c);
uint16_t VEML6040_GetWhite(I2C_HandleTypeDef *hi2c);
rgb_t VEML_GetData(I2C_HandleTypeDef *hi2c);
//uint16_t VEML6040_GetCCT(I2C_HandleTypeDef *hi2c, float offset);
//float VEML6040_GetAmbientLight(I2C_HandleTypeDef *hi2c);

#endif
