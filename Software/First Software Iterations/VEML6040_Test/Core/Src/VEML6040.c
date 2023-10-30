#include "VEML6040.h"
#include "stm32l4xx_hal.h"
#include "math.h"

static uint16_t VEML6040_Read(I2C_HandleTypeDef *hi2c, uint8_t reg) {
    uint8_t data[2];
    HAL_I2C_Mem_Read(hi2c, VEML6040_I2C_ADDRESS << 1, reg, I2C_MEMADD_SIZE_8BIT, data, 2, HAL_MAX_DELAY);
    return (uint16_t)((data[1] << 8) | data[0]);
}

void VEML6040_Init(I2C_HandleTypeDef *hi2c) {
    // Initialize the VEML6040 sensor here, if needed
}

void VEML6040_SetConfiguration(I2C_HandleTypeDef *hi2c, uint8_t config) {
    uint8_t configData[2] = {COMMAND_CODE_CONF, config};
    HAL_I2C_Master_Transmit(hi2c, VEML6040_I2C_ADDRESS << 1, configData, 2, HAL_MAX_DELAY);
}

uint16_t VEML6040_GetRed(I2C_HandleTypeDef *hi2c) {
    return VEML6040_Read(hi2c, COMMAND_CODE_RED);
}

uint16_t VEML6040_GetGreen(I2C_HandleTypeDef *hi2c) {
    return VEML6040_Read(hi2c, COMMAND_CODE_GREEN);
}

uint16_t VEML6040_GetBlue(I2C_HandleTypeDef *hi2c) {
    return VEML6040_Read(hi2c, COMMAND_CODE_BLUE);
}

uint16_t VEML6040_GetWhite(I2C_HandleTypeDef *hi2c) {
    return VEML6040_Read(hi2c, COMMAND_CODE_WHITE);
}

//float VEML6040_GetAmbientLight(I2C_HandleTypeDef *hi2c) {
//    uint16_t sensorValue;
//    float ambientLightInLux;
//
//    sensorValue = VEML6040_Read(hi2c, COMMAND_CODE_GREEN);
//
//    switch (lastConfiguration & 0x70) {
//        case VEML6040_IT_40MS:
//            ambientLightInLux = sensorValue * VEML6040_GSENS_40MS;
//            break;
//        case VEML6040_IT_80MS:
//            ambientLightInLux = sensorValue * VEML6040_GSENS_80MS;
//            break;
//        case VEML6040_IT_160MS:
//            ambientLightInLux = sensorValue * VEML6040_GSENS_160MS;
//            break;
//        case VEML6040_IT_320MS:
//            ambientLightInLux = sensorValue * VEML6040_GSENS_320MS;
//            break;
//        case VEML6040_IT_640MS:
//            ambientLightInLux = sensorValue * VEML6040_GSENS_640MS;
//            break;
//        case VEML6040_IT_1280MS:
//            ambientLightInLux = sensorValue * VEML6040_GSENS_1280MS;
//            break;
//        default:
//            ambientLightInLux = -1;
//            break;
//    }
//    return ambientLightInLux;
//}
//
//uint16_t VEML6040_GetCCT(I2C_HandleTypeDef *hi2c, float offset) {
//    uint16_t red, blue, green;
//    float cct, ccti;
//
//    red = VEML6040_Read(hi2c, COMMAND_CODE_RED);
//    green = VEML6040_Read(hi2c, COMMAND_CODE_GREEN);
//    blue = VEML6040_Read(hi2c, COMMAND_CODE_BLUE);
//
//    ccti = ((float)red - (float)blue) / (float)green;
//    ccti = ccti + offset;
//    cct = 4278.6 * pow(ccti, -1.2455);
//
//    return ((uint16_t)cct);
//}
