/* Includes ------------------------------------------------------------------*/
#include "AS7265x.h"
#include "stm32l4xx_hal.h"
/* Private variables ---------------------------------------------------------*/

extern I2C_HandleTypeDef hi2c2;  // Make sure hi2c2 is defined in your main code

#define AS7265x_I2C_ADDR (0x49 << 1)  // Replace with the actual I2C address of the AS7265x sensor

// Function to check if the AS7265x sensor is connected
uint8_t AS7265x_isConnected() {
    HAL_StatusTypeDef result;
    uint8_t dummy_data = 0x00;

    // Retry up to 100 times, with a 10ms delay each time
    for (uint8_t i = 0; i < 100; i++) {
        // Try to communicate with the sensor by writing a dummy byte
        result = HAL_I2C_Master_Transmit(&hi2c2, AS7265x_I2C_ADDR, &dummy_data, 1, HAL_MAX_DELAY);

        // If the transmission is successful, return true (sensor is connected)
        if (result == HAL_OK) {
            return 1;  // Sensor ACK'd
        }

        // Delay 10ms before retrying
        HAL_Delay(10);
    }

    // If the loop completes without a successful transmission, return false
    return 0;  // Sensor did not ACK
}
