#include "VEML6040.h"

void VEML6040_Init(VEML6040_Handle *handle, I2C_HandleTypeDef *hi2c) {
    handle->hi2c = hi2c;
    handle->lastConfiguration = 0;
}

HAL_StatusTypeDef VEML6040_Begin(VEML6040_Handle *handle) {
    HAL_StatusTypeDef status = HAL_OK;
    uint8_t data = 0;

    status = HAL_I2C_IsDeviceReady(handle->hi2c, VEML6040_I2C_ADDRESS << 1, 2, HAL_MAX_DELAY);

    return status;
}

void VEML6040_SetConfiguration(VEML6040_Handle *handle, uint8_t configuration) {
    uint8_t data[3] = {COMMAND_CODE_CONF, configuration, 0};

    HAL_I2C_Master_Transmit(handle->hi2c, VEML6040_I2C_ADDRESS << 1, data, sizeof(data), HAL_MAX_DELAY);
    handle->lastConfiguration = configuration;
}

uint16_t VEML6040_Read(VEML6040_Handle *handle, uint8_t commandCode) {
    uint16_t data = 0;
    uint8_t rxData[2];

    HAL_I2C_Master_Transmit(handle->hi2c, VEML6040_I2C_ADDRESS << 1, &commandCode, 1, HAL_MAX_DELAY);
    HAL_I2C_Master_Receive(handle->hi2c, VEML6040_I2C_ADDRESS << 1, rxData, sizeof(rxData), HAL_MAX_DELAY);

    data = (uint16_t)(rxData[1] << 8) | rxData[0];

    return data;
}

uint16_t VEML6040_GetRed(VEML6040_Handle *handle) {
    return VEML6040_Read(handle, COMMAND_CODE_RED);
}

uint16_t VEML6040_GetGreen(VEML6040_Handle *handle) {
    return VEML6040_Read(handle, COMMAND_CODE_GREEN);
}

uint16_t VEML6040_GetBlue(VEML6040_Handle *handle) {
    return VEML6040_Read(handle, COMMAND_CODE_BLUE);
}

uint16_t VEML6040_GetWhite(VEML6040_Handle *handle) {
    return VEML6040_Read(handle, COMMAND_CODE_WHITE);
}

float VEML6040_GetAmbientLight(VEML6040_Handle *handle) {
    uint16_t sensorValue;
    float ambientLightInLux;

    sensorValue = VEML6040_Read(handle, COMMAND_CODE_GREEN);

    switch (handle->lastConfiguration & 0x70) {
        case VEML6040_IT_40MS:    ambientLightInLux = (float)sensorValue * VEML6040_GSENS_40MS; break;
        case VEML6040_IT_80MS:    ambientLightInLux = (float)sensorValue * VEML6040_GSENS_80MS; break;
        case VEML6040_IT_160MS:   ambientLightInLux = (float)sensorValue * VEML6040_GSENS_160MS; break;
        case VEML6040_IT_320MS:   ambientLightInLux = (float)sensorValue * VEML6040_GSENS_320MS; break;
        case VEML6040_IT_640MS:   ambientLightInLux = (float)sensorValue * VEML6040_GSENS_640MS; break;
        case VEML6040_IT_1280MS:  ambientLightInLux = (float)sensorValue * VEML6040_GSENS_1280MS; break;
        default:                  ambientLightInLux = -1.0f; break;
    }
    return ambientLightInLux;
}

uint16_t VEML6040_GetCCT(VEML6040_Handle *handle, float offset) {
    uint16_t red, blue, green;
    float cct, ccti;

    red = VEML6040_GetRed(handle);
    green = VEML6040_GetGreen(handle);
    blue = VEML6040_GetBlue(handle);

    ccti = ((float)red - (float)blue) / (float)green;
    ccti = ccti + offset;
    cct = 4278.6f * powf(ccti, -1.2455f);

    return (uint16_t)cct;
}
