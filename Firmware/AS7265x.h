/*
 * AS7265x.h
 *
 *  Created on: Jul 2, 2024
 *      Author: sarah
 */

#ifndef AS7265X_H_
#define AS7265X_H_

//====================================================================
// REGISTER VALUES
//====================================================================

#define 	HW_VERSION_H			(0x00)
#define 	HW_VERSION_L         	(0x01)
#define 	FW_VERSION_H         	(0x02)
#define 	FW_VERSION_L         	(0x03)

#define 	AS7265x_CONFIG       	(0x04)
#define 	AS7265x_ITIME		 	(0X05)
#define 	AS7265x_TEMP		 	(0X06)
#define 	AS7265x_LED_CONFIG	 	(0X07)

#define 	RAW_VALUE_RGA_H    	 	(0x08)
#define 	RAW_VALUE_RGA_L		 	(0x09)
#define 	RAW_VALUE_SHB_H      	(0x0A)
#define 	RAW_VALUE_SHB_L      	(0x0B)
#define 	RAW_VALUE_TIC_H      	(0x0C)
#define 	RAW_VALUE_TIC_L      	(0x0D)
#define 	RAW_VALUE_UJD_H      	(0x0E)
#define 	RAW_VALUE_UJD_L      	(0x0F)
#define 	RAW_VALUE_VKE_H      	(0x10)
#define 	RAW_VALUE_VKE_L      	(0x11)
#define 	RAW_VALUE_WLF_H      	(0x12)
#define 	RAW_VALUE_WLF_L      	(0x13)

#define 	CAL_RGA_0          	 	(0x14)
#define 	CAL_RGA_1        	 	(0x15)
#define 	CAL_RGA_2	         	(0x16)
#define 	CAL_RGA_3 	         	(0x17)

#define 	CAL_SHB_0  	         	(0x18)
#define 	CAL_SHB_1    	     	(0x19)
#define 	CAL_SHB_2      	     	(0x1A)
#define 	CAL_SHB_3    	     	(0x1B)

#define 	CAL_TIC_0          	 	(0x1C)
#define 	CAL_TIC_1            	(0x1D)
#define 	CAL_TIC_2            	(0x1E)
#define 	CAL_TIC_3            	(0x1F)

#define 	CAL_UJD_0            	(0x20)
#define 	CAL_UJD_1            	(0x21)
#define 	CAL_UJD_2            	(0x22)
#define 	CAL_UJD_3            	(0x23)

#define 	CAL_VKE_0            	(0x24)
#define 	CAL_VKE_1            	(0x25)
#define 	CAL_VKE_2            	(0x26)
#define 	CAL_VKE_3            	(0x27)

#define 	CAL_WLF_0            	(0x28)
#define 	CAL_WLF_1            	(0x29)
#define 	CAL_WLF_2            	(0x2A)
#define 	CAL_WLF_3            	(0x2B)


#define 	FW_CONTROL           	(0x48)
#define 	FW_BYTE_COUNT_H      	(0x49)
#define 	FW_BYTE_COUNT_L      	(0x4A)
#define 	FW_PAYLOAD           	(0x4B)

#define 	DEV_SEL              	(0x4F)

#define 	COEF_DATA_0          	(0x50)
#define 	COEF_DATA_1          	(0x51)
#define 	COEF_DATA_2          	(0x52)
#define 	COEF_DATA_3          	(0x53)
#define 	COEF_READ            	(0x54)
#define 	COEF_WRITE           	(0x55)

#endif /* AS7265X_H_ */
