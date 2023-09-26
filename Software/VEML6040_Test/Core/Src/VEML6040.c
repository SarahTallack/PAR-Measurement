#include "VEML6040.h"

// Private function to read data from the VEML6040 sensor
uint16_t VEML6040_read(uint8_t commandCode) {
    uint8_t data[2];

    HAL_I2C_Mem_Read(&hi2c3, VEML6040_I2C_ADDRESS, commandCode, 1, data, 2, HAL_MAX_DELAY);

    uint16_t result = (data[1] << 8) | data[0];
    return result;
}

//void VEML6040_init(void) {
//    // Initialize the I2C peripheral (assuming it's already configured)
//    hi2c3.Instance = I2C1; // Adjust the I2C instance as needed
//}

void VEML6040_setConfiguration(uint8_t configuration) {
    HAL_I2C_Mem_Write(&hi2c3, VEML6040_I2C_ADDRESS, COMMAND_CODE_CONF, 1, &configuration, 1, HAL_MAX_DELAY);
}

uint16_t VEML6040_getRed(void) {
    return VEML6040_read(COMMAND_CODE_RED);
}

uint16_t VEML6040_getGreen(void) {
    return VEML6040_read(COMMAND_CODE_GREEN);
}

uint16_t VEML6040_getBlue(void) {
    return VEML6040_read(COMMAND_CODE_BLUE);
}

uint16_t VEML6040_getWhite(void) {
    return VEML6040_read(COMMAND_CODE_WHITE);
}

float VEML6040_getAmbientLight(void) {
    uint16_t sensorValue = VEML6040_read(COMMAND_CODE_GREEN);
    float ambientLightInLux;

    switch (VEML6040_read(COMMAND_CODE_CONF) & 0x70) {
        case VEML6040_IT_40MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_40MS;
            break;
        case VEML6040_IT_80MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_80MS;
            break;
        case VEML6040_IT_160MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_160MS;
            break;
        case VEML6040_IT_320MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_320MS;
            break;
        case VEML6040_IT_640MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_640MS;
            break;
        case VEML6040_IT_1280MS:
            ambientLightInLux = sensorValue * VEML6040_GSENS_1280MS;
            break;
        default:
            ambientLightInLux = -1;
            break;
    }
    return ambientLightInLux;
}

//uint16_t VEML6040_getCCT(float offset) {
//    uint16_t red, blue, green;
//    float cct, ccti;
//
//    red = VEML6040_getRed();
//    green = VEML6040_getGreen();
//    blue = VEML6040_getBlue();
//
//    ccti = ((float)red - (float)blue) / (float)green;
//    ccti = ccti + offset;
//    cct = 4278.6 * powf(ccti, -1.2455);
//
//    return (uint16_t)cct;
//}
