#ifndef LCD_STM32F0_H
#define LCD_STM32F0_H
//********************************************************************
//*                      EEE2046F STM32F0                            *
//*                         LCD HEADER                               *
//*==================================================================*
//* WRITTEN BY:    R. Verrinder                                      *
//* MODIFIED:      03-08-2015                                        *
//*==================================================================*

#include "stm32l4xx_hal.h"

//====================================================================
// GLOBAL CONSTANTS - LCD command codes
//====================================================================
#define    CLEAR_DISPLAY           0x01
#define    CURSOR_HOME     0x02
#define    DISPLAY_ON      0x0C
#define    DISPLAY_OFF     0x08
#define    LINE_TWO        0xC0

#define    POWER_UP        0x33
#define    FOURBIT_MODE    0X32
#define    TWOLINE_MODE    0x28

#define    DELAY_COUNT     0x1FF

#define	   DELAY		   500
//====================================================================
// FUNCTION DECLARATIONS
//====================================================================
void init_LCD(void);

void lcd_command(unsigned char command);
void lcd_putchar(unsigned char character);
void lcd_putstring(char *instring);

void delay(unsigned int microseconds);
void pulse_strobe(void);

//====================================================================

#endif

//********************************************************************
// END OF PROGRAM
//********************************************************************
