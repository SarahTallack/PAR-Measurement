/*****************************************************************************
* | File      	:   AS7341.c
* | Author      :   Waveshare team
* | Function    :   AS7341 driver
* | Info        :
*----------------
* |	This version:   V1.0
* | Date        :   2021-1-13
* | Info        :   Basic version
*
******************************************************************************/
#include "Waveshare_AS7341.h"

eMode_t measureMode;

uint8_t DEV_I2C_Device = 0;
uint8_t I2C_ADDR;
/******************************************************************************
function:	Read one byte of data to AS7341 via I2C
parameter:  
            Addr: Register address
Info:
******************************************************************************/
static uint8_t AS7341_Read_Byte(uint8_t Addr)
{
  uint8_t Buf[1]={0};
	HAL_I2C_Mem_Read(&hi2c2, I2C_ADDR+1, Addr, I2C_MEMADD_SIZE_8BIT, Buf, 1, 0x20);
	return Buf[0];
}

/******************************************************************************
function:	Read one word of data to AS7341 via I2C
parameter:
            Addr: Register address
Info:
******************************************************************************/
/*
static uint16_t AS7341_Read_Word(uint8_t Addr)
{
    return I2C_Read_Word(Addr);
}
*/
/******************************************************************************
function:	Send one byte of data to AS7341 via I2C
parameter:
            Addr: Register address
           Value: Write to the value of the register
Info:
******************************************************************************/
static void AS7341_Write_Byte(uint8_t Addr, uint8_t Value)
{
	uint8_t Buf[1] = {0};
	Buf[0] = Value;
	HAL_I2C_Mem_Write(&hi2c2, I2C_ADDR, Addr, I2C_MEMADD_SIZE_8BIT, Buf, 1, 0x20);
}


/******************************************************************************
function:	AS7341 Initialization
parameter:
Info:
******************************************************************************/
uint8_t AS7341_Init(eMode_t mode)
{
	I2C_ADDR =  AS7341_ADDRESS<<1;
  printf("ID = 0x%x\r\n",AS7341_Read_Byte(AS7341_ID));//get AS7341 id	
	AS7341_Enable(true);
	measureMode=mode;
	return 0;
}

/******************************************************************************
function： enable PON
info：	power on
******************************************************************************/
void AS7341_Enable(int flag)
{
	uint8_t data;
	data=AS7341_Read_Byte(AS7341_ENABLE);
    if(flag == true){
    data = data | (1<<0);
    } else {
    data = data & (~1);
    }
	AS7341_Write_Byte(AS7341_ENABLE,data);
//	printf("Initialization is complete !\r\n");
    AS7341_Write_Byte(0x00, 0x30);
}

/******************************************************************************
function： enable Spectral measure
info：	
******************************************************************************/
void AS7341_EnableSpectralMeasure(int flag)//Enable spectral measurement
{
    uint8_t data;
    data=AS7341_Read_Byte(AS7341_ENABLE);
    if(flag == true){
      data = data | (1<<1);
    } else {
      data = data & (~(1<<1));
    }
    AS7341_Write_Byte(AS7341_ENABLE,data);
}

/******************************************************************************
function： enable SMUX
info：	
******************************************************************************/
void AS7341_EnableSMUX(int flag)//Enable multiplexer
/*The other available channels can be accessed by amultiplexer (SMUX) connecting them to one of the internal ADCs.*/
{
  uint8_t data;
    data=AS7341_Read_Byte(AS7341_ENABLE);
  if(flag == true){
    data = data | (1<<4);
  } else {
    data = data & (~(1<<4));
  }
    AS7341_Write_Byte(AS7341_ENABLE,data);
}

/******************************************************************************
function:	enable flicker detection
info：	
******************************************************************************/
void AS7341_EnableFlickerDetection(int flag)
{

  uint8_t data;
  data=AS7341_Read_Byte(AS7341_ENABLE);
  if(flag == true){
    data = data | (1<<6);
  } else {
    data = data & (~(1<<6));
  }
  AS7341_Write_Byte(AS7341_ENABLE,data);
}

/******************************************************************************
function:	choose model for spectral measurement
info：	
******************************************************************************/
void AS7341_Config(eMode_t mode)
{
  uint8_t data;
  AS7341_SetBank(1);
  data=AS7341_Read_Byte(AS7341_CONFIG);
  switch(mode){
    case eSpm : {
      data = (data & (~3)) | eSpm;
    };
	break;
    case eSyns : {
      data = (data & (~3)) | eSyns;
    };
	break;
    case eSynd : {
      data = (data & (~3)) | eSynd;
    };
	break;
    default : break;
  }
  AS7341_Write_Byte(AS7341_CONFIG,data);
  AS7341_SetBank(0);
}

/******************************************************************************
function:	Configure SMUX for sensors F1-4, Clear and NIR
info：	
******************************************************************************/
void F1F4_Clear_NIR() 
{
  AS7341_Write_Byte(0x00, 0x30); 
  AS7341_Write_Byte(0x01, 0x01); 
  AS7341_Write_Byte(0x02, 0x00); 
  AS7341_Write_Byte(0x03, 0x00); 
  AS7341_Write_Byte(0x04, 0x00); 
  AS7341_Write_Byte(0x05, 0x42); 
  AS7341_Write_Byte(0x06, 0x00); 
  AS7341_Write_Byte(0x07, 0x00); 
  AS7341_Write_Byte(0x08, 0x50); 
  AS7341_Write_Byte(0x09, 0x00); 
  AS7341_Write_Byte(0x0A, 0x00); 
  AS7341_Write_Byte(0x0B, 0x00); 
  AS7341_Write_Byte(0x0C, 0x20); 
  AS7341_Write_Byte(0x0D, 0x04); 
  AS7341_Write_Byte(0x0E, 0x00); 
  AS7341_Write_Byte(0x0F, 0x30); 
  AS7341_Write_Byte(0x10, 0x01); 
  AS7341_Write_Byte(0x11, 0x50); 
  AS7341_Write_Byte(0x12, 0x00); 
  AS7341_Write_Byte(0x13, 0x06); 
}
/******************************************************************************
function:	Configure SMUX for sensors F5-8, Clear and NIR
info：	
******************************************************************************/
void F5F8_Clear_NIR() 
{
  AS7341_Write_Byte(0x00, 0x00); 
  AS7341_Write_Byte(0x01, 0x00); 
  AS7341_Write_Byte(0x02, 0x00); 
  AS7341_Write_Byte(0x03, 0x40); 
  AS7341_Write_Byte(0x04, 0x02); 
  AS7341_Write_Byte(0x05, 0x00); 
  AS7341_Write_Byte(0x06, 0x10); 
  AS7341_Write_Byte(0x07, 0x03); 
  AS7341_Write_Byte(0x08, 0x50); 
  AS7341_Write_Byte(0x09, 0x10); 
  AS7341_Write_Byte(0x0A, 0x03); 
  AS7341_Write_Byte(0x0B, 0x00); 
  AS7341_Write_Byte(0x0C, 0x00); 
  AS7341_Write_Byte(0x0D, 0x00); 
  AS7341_Write_Byte(0x0E, 0x24); 
  AS7341_Write_Byte(0x0F, 0x00); 
  AS7341_Write_Byte(0x10, 0x00); 
  AS7341_Write_Byte(0x11, 0x50); 
  AS7341_Write_Byte(0x12, 0x00); 
  AS7341_Write_Byte(0x13, 0x06); 
}
/******************************************************************************
function:	Configure SMUX for flicker detection
info：	
******************************************************************************/
void FDConfig() 
{

  AS7341_Write_Byte(0x00, 0x00); 
  AS7341_Write_Byte(0x01, 0x00); 
  AS7341_Write_Byte(0x02, 0x00); 
  AS7341_Write_Byte(0x03, 0x00); 
  AS7341_Write_Byte(0x04, 0x00); 
  AS7341_Write_Byte(0x05, 0x00); 
  AS7341_Write_Byte(0x06, 0x00); 
  AS7341_Write_Byte(0x07, 0x00); 
  AS7341_Write_Byte(0x08, 0x00); 
  AS7341_Write_Byte(0x09, 0x00); 
  AS7341_Write_Byte(0x0A, 0x00); 
  AS7341_Write_Byte(0x0B, 0x00); 
  AS7341_Write_Byte(0x0C, 0x00); 
  AS7341_Write_Byte(0x0D, 0x00); 
  AS7341_Write_Byte(0x0E, 0x00); 
  AS7341_Write_Byte(0x0F, 0x00); 
  AS7341_Write_Byte(0x10, 0x00); 
  AS7341_Write_Byte(0x11, 0x00); 
  AS7341_Write_Byte(0x12, 0x00); 
  AS7341_Write_Byte(0x13, 0x60);
}

/******************************************************************************
function:	Start the measurement
info：		This function only handles SPM and SYNS modes.
******************************************************************************/
void AS7341_startMeasure(eChChoose_t mode)
{
	  uint8_t data=0;
	  data = AS7341_Read_Byte(AS7341_CFG_0);  
	  data = data & (~(1<<4));

	  AS7341_Write_Byte(AS7341_CFG_0,data);
	  
	  AS7341_EnableSpectralMeasure(false);
	  AS7341_Write_Byte(0xAF,0x10);//SMUX Command config
	  
	  if(mode  == eF1F4ClearNIR)
      F1F4_Clear_NIR();
	  else if(mode  == eF5F8ClearNIR)
	  F5F8_Clear_NIR();
	  AS7341_EnableSMUX(true);
	  if(measureMode == eSyns){
	  AS7341_SetGpioMode(INPUT);
      AS7341_Config(eSyns);
      }
	  else if(measureMode == eSpm){
      AS7341_Config(eSpm);
	  }
	  AS7341_EnableSpectralMeasure(true);
      if(measureMode == eSpm){
        while(!AS7341_MeasureComplete()){
        HAL_Delay(1);
        }
      }
}
/******************************************************************************
function:  read flicker data
info：		
******************************************************************************/
uint8_t AS7341_ReadFlickerData()
{
	  uint8_t flicker;
	  uint8_t data=0;
	  data = AS7341_Read_Byte(AS7341_CFG_0);
	  data = data & (~(1<<4));
	  AS7341_Write_Byte(AS7341_CFG_0,data);
	  AS7341_EnableSpectralMeasure(false);
	  AS7341_Write_Byte(0xAF,0x10);
	  FDConfig();
	  AS7341_EnableSMUX(true);
	  AS7341_EnableSpectralMeasure(true);
	  //AS7341_Write_Byte(0xDA,0x00);	
//	  uint8_t retry = 100;
//	  if(retry == 0) printf(" data access error");
	  AS7341_EnableFlickerDetection(true);
	  HAL_Delay(600);
	  flicker = AS7341_Read_Byte(AS7341_STATUS);
//	  printf("flicker: %d \r\n",flicker);
	  AS7341_EnableFlickerDetection(false);
	  switch(flicker){
		case 37:
		  flicker = 100;
		  break;
		case 40:
		  flicker = 0;
		  break;
		case 42:
		  flicker = 120;
		  break;		  
		case 44:
		  flicker = 1;
		  break;		  
		case 45:
		  flicker = 2;
		  break;		  
		default:
		  flicker = 2;
	  }
	  return flicker;
}

/******************************************************************************
function:  Determine whether the measurement is complete
info：		
******************************************************************************/

int AS7341_MeasureComplete(){
	uint8_t status;
	status = AS7341_Read_Byte(AS7341_STATUS_2); 
	if((status & (1<<6))){
		return true;
	}
	else{
		return false;
	}
}

/******************************************************************************
function:  Gets data for all channels
info：		
******************************************************************************/
uint16_t AS7341_GetChannelData(uint8_t channel)
{
  uint16_t data[2];
  uint16_t channelData = 0x0000;
  data[0] = AS7341_Read_Byte(AS7341_CH0_DATA_L + channel*2); 
  data[1] = AS7341_Read_Byte(AS7341_CH0_DATA_H + channel*2); 
  channelData = data[1];
  channelData = (channelData<<8) | data[0];
  HAL_Delay(50);
  return channelData;
}

/******************************************************************************
function:  Use SMUX to read data from the low channel
info：		
******************************************************************************/

sModeOneData_t AS7341_ReadSpectralDataOne()
{
  sModeOneData_t data;
  data.channel1 = AS7341_GetChannelData(0);
  data.channel2 = AS7341_GetChannelData(1);
  data.channel3 = AS7341_GetChannelData(2);
  data.channel4 = AS7341_GetChannelData(3);
  data.CLEAR = AS7341_GetChannelData(4);
  data.NIR = AS7341_GetChannelData(5);
  return data;
}

/******************************************************************************
function:  Use SMUX to read data from the high channel
info：		
******************************************************************************/

sModeTwoData_t AS7341_ReadSpectralDataTwo()
{
  sModeTwoData_t data;
  data.channel5 = AS7341_GetChannelData(0);
  data.channel6 = AS7341_GetChannelData(1);
  data.channel7 = AS7341_GetChannelData(2);
  data.channel8 = AS7341_GetChannelData(3);
  data.CLEAR = AS7341_GetChannelData(4);
  data.NIR = AS7341_GetChannelData(5);
  return data;
}

/******************************************************************************
function:	Set GPIO to input or output mode
info：
******************************************************************************/
void AS7341_SetGpioMode(uint8_t mode)
{
  uint8_t data;

  data = AS7341_Read_Byte(AS7341_GPIO_2);  
  if(mode == INPUT){
     data = data | (1<<2);
  }
  
  if(mode == OUTPUT){
     data = data & (~(1<<2));
  }
  AS7341_Write_Byte(AS7341_GPIO_2,data);
}

/******************************************************************************
function:	Configure the ATIME register
info：
******************************************************************************/
void AS7341_ATIME_config(uint8_t value)
{
  AS7341_Write_Byte(AS7341_ATIME,value);
}

/******************************************************************************
function:	Configure the ASTEP register
info：
******************************************************************************/
void AS7341_ASTEP_config(uint16_t value)
{
  uint8_t highValue,lowValue;
  lowValue = value & 0x00ff;
  highValue = value >> 8 ;
  AS7341_Write_Byte(AS7341_ASTEP_L,lowValue);
  AS7341_Write_Byte(AS7341_ASTEP_H,highValue);
}

/******************************************************************************
function:	Configure the AGAIN register
value:    0    1    2    3    4    5      6     7     8     9      10
gain:   X0.5 | X1 | X2 | X4 | X8 | X16 | X32 | X64 | X128 | X256 | X512
******************************************************************************/
void AS7341_AGAIN_config(uint8_t value)
{
  if(value > 10) value = 10;
  AS7341_Write_Byte(AS7341_CFG_1,value);
}

/******************************************************************************
function:	enable led
info：true		: Enable
			false		: Anergy
******************************************************************************/
void AS7341_EnableLED(int flag)
{
  uint8_t data=0;
  uint8_t data1=0;
  AS7341_SetBank(1);
  data = AS7341_Read_Byte(AS7341_CONFIG);
  data1 = AS7341_Read_Byte(AS7341_LED);
  if(flag== true)
	{
    data = data | 0x08;
	} 
	else 
	{
    data = data & 0xf7;
    data1 = data1 & 0x7f;
		AS7341_Write_Byte(AS7341_LED,data1);
  }
  AS7341_Write_Byte(AS7341_CONFIG,data);
  AS7341_SetBank(0);
}
/******************************************************************************
function:	set REG_BANK
info：	0: Register access to register 0x80 and above
		1: Register access to register 0x60 to 0x74
******************************************************************************/
void AS7341_SetBank(uint8_t addr)
{
  uint8_t data=0;
  data = AS7341_Read_Byte(AS7341_CFG_0);
  if(addr == 1){
  
    data = data | (1<<4);
  }
  
  if(addr == 0){
  
    data = data & (~(1<<4));
  }
  AS7341_Write_Byte(AS7341_CFG_0,data);
}
/******************************************************************************
function:	Control the brightness of the LED
info：   LED 			 :   true  : LED ON
											 false : LED OFF
         current   :  intensity control
******************************************************************************/
void AS7341_ControlLed(uint8_t LED,uint8_t current)
{
  uint8_t data=0;
  if(current < 1) current = 1;
    current--;
  if(current > 19) current = 19;
  AS7341_SetBank(1); 
	if(LED == true)	
		data = 0x80 | current;
	else	
		data = current;
  AS7341_Write_Byte(AS7341_LED,data);
  HAL_Delay(100);
  AS7341_SetBank(0);
}

/******************************************************************************
function:	Determine whether the threshold setting is exceeded
info：Spectral interruptions occur when the set threshold is exceeded
******************************************************************************/
void AS7341_INTerrupt()
{
  uint8_t data = AS7341_Read_Byte(AS7341_STATUS_1);
  if(data & 0x80){
//     printf("Spectral interrupt generation ！\r\n");
  } else {
    return ;
  }
}

/******************************************************************************
function:	clear interrupt
info：		This register is self-clearing, meaning that writing a "1" to any bit in the
	register clears that status bit. 
******************************************************************************/
void AS7341_ClearInterrupt()
{
	
  AS7341_Write_Byte(AS7341_STATUS_1,0xff);

}
/******************************************************************************
function:	enable spectral interrupt
info：
******************************************************************************/
void AS7341_EnableSpectralInterrupt(int flag)
{
  uint8_t data;
  data = AS7341_Read_Byte(AS7341_INTENAB);
  if(flag == true)
  {
    data = data | (1<<3);
    AS7341_Write_Byte(AS7341_INTENAB,data);
  }
  else{
    data = data & (~(1<<3));
    AS7341_Write_Byte(AS7341_INTENAB,data);
  }
  
}
/******************************************************************************
function:Spectral Interrupt Persistence
	value:      CHANNEL:
	0			Every spectral cycle generates aninterrupt
	1			1
	2			2
	3			3
	4			5
	5			10
	...			5*(value-3)
	14			55
	15			60
******************************************************************************/
void AS7341_SetInterruptPersistence(uint8_t value)
{
	uint8_t data;
	data= value;
	AS7341_Write_Byte(AS7341_PERS,data);
	data = AS7341_Read_Byte(AS7341_PERS);
}
/******************************************************************************
function:	Set the interrupt threshold up and 
info：
******************************************************************************/
void AS7341_SetThreshold(uint16_t lowThre,uint16_t highThre)
{
  if(lowThre >= highThre)return ;
  else
  
  AS7341_Write_Byte(AS7341_SP_TH_L_LSB,lowThre);
  AS7341_Write_Byte(AS7341_SP_TH_L_MSB,lowThre>>8);
  
  AS7341_Write_Byte(AS7341_SP_TH_H_LSB,highThre);
  AS7341_Write_Byte(AS7341_SP_TH_H_MSB,highThre>>8); 
  
  HAL_Delay(20);
}
/******************************************************************************
function:	Set the Spectral Threshold Channel
		VALUE 			CHANNEL
		0 				CH0
		1 				CH1
		2 				CH2
		3 				CH3
		4 				CH4
******************************************************************************/
void AS7341_SetSpectralThresholdChannel(uint8_t value)
{
	AS7341_Write_Byte(AS7341_CFG_12,value);
}


/******************************************************************************
function:	get low threshold
info：
******************************************************************************/

uint16_t AS7341_GetLowThreshold()
{
  uint16_t data; 
  data = AS7341_Read_Byte(AS7341_SP_TH_L_LSB);  
  data = (AS7341_Read_Byte(AS7341_SP_TH_L_MSB)<<8) | data; 
  return data ;
}

/******************************************************************************
function:	get high threshold
info：
******************************************************************************/
uint16_t AS7341_GetHighThreshold()
{
  uint16_t data;
  data = AS7341_Read_Byte(AS7341_SP_TH_H_LSB);
  data = (AS7341_Read_Byte(AS7341_SP_TH_H_MSB)<<8) | data;
  return data ;
}

/******************************************************************************
function:	syns interrupt set
info：
******************************************************************************/
void AS7341_SynsINT_sel()
{
	AS7341_Write_Byte(AS7341_CONFIG,0x05);
}


/******************************************************************************
function:	disable power,spectral reading, flicker detection  
info：
******************************************************************************/
void AS7341_disableALL()
{
	AS7341_Write_Byte(AS7341_ENABLE ,0x02);
}
