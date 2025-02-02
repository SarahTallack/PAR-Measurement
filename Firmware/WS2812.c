/* Includes ------------------------------------------------------------------*/
#include "WS2812.h"
#include "string.h"
#include "stdio.h"
#include "math.h"
#include "tim.h"
#include "dma.h"
/* Private variables ---------------------------------------------------------*/
uint32_t *pBuff;

uint32_t dmaBuffer[DMA_BUFF_SIZE] = {0};
pixelRGB_t pixel[NUM_PIXELS] = {0};

//uint8_t mode = 1; // update mode for different LED pattern. 0 - ALL ON; 1 - Single LED with position stored in pos
//uint8_t pos = 13;

//float brightness = 45;
int datasentflag = 0;
/******************************************************************************
function:	Set all LEDs to RGB value
info：		Loop through all pixel values and set to RGB colour.
******************************************************************************/
void RGB(uint8_t r, uint8_t g, uint8_t b, uint8_t brightness, uint8_t mode, uint8_t pos){
	if (mode == 1) {
		for (int i = 0; i < NUM_PIXELS; i++)
		{
			if (i == pos)
			{
				pixel[i].color.r = r;
				pixel[i].color.g = g;
				pixel[i].color.b = b;
			}
			else
			{
				pixel[i].color.r = 0;
				pixel[i].color.g = 0;
				pixel[i].color.b = 0;
			}

		}
	} else {
		for (int i = 0; i < NUM_PIXELS; i++){
			pixel[i].color.r = r;
			pixel[i].color.g = g;
			pixel[i].color.b = b;
		}
	}
	Set_Brightness(brightness);
	WS2812_Send();
}

void Set_Brightness(float brightness)
{
	if (brightness > 45) brightness = 45;
	float angle = 90-brightness;  // in degrees
	angle = angle*PI / 180;  // in rad
	for (int i= 0; i<NUM_PIXELS; i++)
	{
		pixel[i].color.g = pixel[i].color.g/(tan(angle));
		pixel[i].color.r = pixel[i].color.r/(tan(angle));
		pixel[i].color.b = pixel[i].color.b/(tan(angle));
	}
}

void WS2812_Send(void)
{
    pBuff = dmaBuffer;
    for (int i = 0; i < NUM_PIXELS; i++)
    {
       for (int j = 23; j >= 0; j--)
       {
         if ((pixel[i].data >> j) & 0x01)
         {
           *pBuff = NEOPIXEL_ONE;
         }
         else
         {
           *pBuff = NEOPIXEL_ZERO;
         }
         pBuff++;
     }
    }
    dmaBuffer[DMA_BUFF_SIZE - 1] = 0; // last element must be 0!

    HAL_TIM_PWM_Start_DMA(&htim2, TIM_CHANNEL_4, dmaBuffer, DMA_BUFF_SIZE);
    // HAL_Delay(400);
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  HAL_TIM_PWM_Stop_DMA(htim, TIM_CHANNEL_4);
  datasentflag = 1;
}
