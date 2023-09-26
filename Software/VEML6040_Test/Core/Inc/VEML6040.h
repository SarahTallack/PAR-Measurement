#ifndef __VEML6040_H
#define __VEML6040_H

// Define the I2C address of the VEML6040 sensor
#define VEML6040_I2C_ADDRESS 0x10

// Define command codes
#define COMMAND_CODE_CONF 0x00
#define COMMAND_CODE_RED 0x08
#define COMMAND_CODE_GREEN 0x09
#define COMMAND_CODE_BLUE 0x0A
#define COMMAND_CODE_WHITE 0x0B

// Define integration times (adjust as needed)
#define VEML6040_IT_40MS 0x00
#define VEML6040_IT_80MS 0x10
#define VEML6040_IT_160MS 0x20
#define VEML6040_IT_320MS 0x30
#define VEML6040_IT_640MS 0x40
#define VEML6040_IT_1280MS 0x50

#define VEML6040_GSENS_40MS 0.25
#define VEML6040_GSENS_80MS 0.125
#define VEML6040_GSENS_160MS 0.0625
#define VEML6040_GSENS_320MS 0.03125
#define VEML6040_GSENS_640MS 0.015625
#define VEML6040_GSENS_1280MS 0.0078125

typedef struct {
  uint8_t sensorExists;
  uint8_t lastConfiguration;
} VEML6040_HandleTypeDef;

static uint16_t VEML6040_read(uint8_t commandCode)
void VEML6040_setConfiguration(uint8_t configuration);
uint16_t VEML6040_getRed(void);
uint16_t VEML6040_getGreen(void);
uint16_t VEML6040_getBlue(void);
uint16_t VEML6040_getWhite(void);
float VEML6040_getAmbientLight(void);
//uint16_t VEML6040_getCCT(float offset);

#endif /* __VEML6040_H */
