#ifndef WS2812_H
#define WS2812_H
//*******************************************************************
//*                      WS2812 RGB LEDS                            *
//*                          HEADER                           	    *
//*=================================================================*
//* WRITTEN BY:    S. Tallack										*
//* MODIFIED:      01-07-2024                                       *
//*=================================================================*

/* Private includes ----------------------------------------------------------*/
#include "stm32l4xx_hal.h"

/* Private define ------------------------------------------------------------*/
#define 	NEOPIXEL_ZERO 	32 	// (ARR+)(0.32) = (100*0.32) = 32
#define 	NEOPIXEL_ONE	68	// (ARR+)(0.68) = (100*0.68) = 68

#define 	NUM_PIXELS		1*9
#define 	DMA_BUFF_SIZE	(NUM_PIXELS * 24) + 1

#define 	PI 				3.14159265
/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef union{
  struct{
    uint8_t b;
    uint8_t r;
    uint8_t g;
  } color;
  uint32_t data;
} pixelRGB_t;

/* Private function prototypes -----------------------------------------------*/

void RGB(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness, uint8_t mode, uint8_t pos);
void Set_Brightness(float brightness);

void WS2812_Send(void);

//===================================================================

#endif

//*******************************************************************
// END OF PROGRAM
//*******************************************************************
