/*
 * Config_Parameters.h
 *
 *  Created on: Sep 21, 2023
 *      Author: Sarah
 */

#ifndef INC_CONFIG_PARAMETERS_H_
#define INC_CONFIG_PARAMETERS_H_

#define t_meas	1000 	//Time between measurements, in ms
						//Options are: 100, 500, 1000, 5000, 10000

/* Parameter settings for VEML6040 */
//Set integration time of VEML6040, which leads to corresponding G_Sensitivity and Max Detectable Lux
/*	INTEGRATION TIME 	G SENSITIVITY MAX. 	DETECTABLE LUX
	 40 ms 				0.25168 			16 496
	 80 ms 				0.12584 			8248
	 160 ms 			0.06292 			4124
	 320 ms 			0.03146 			2062
	 640 ms 			0.01573 			1031
	 1280 ms 			0.007865 			515.4
 */

/* Parameter settings for AS7341 */

#define MODE 	eSpm	//eSpm -> default setting, spectral measurement, no sync
						//eSyns -> integration with external start, spectral measurement, start sync

//#define t_int 	160		//Integration time, in ms
						//Options are: 10, 20, 50, 100, 200, 500, 1000, 2000
						//40, 80, 160, 320, 640, 1280
						//default to 50ms

#define AGAIN 	6		//Gain between 0 and 10, corresponds to following values
						//[0 -> 0.5x, 1 -> 1x, 2 -> 2x, 3 -> 4x, 4 -> 8x, 5 -> 16x, 6 - 32x, 7 -> 64x, 8 -> 128x, 9 -> 256x, 10 -> 512x

#define INT 	false 	//enable or disable interrupt mode (generates interrupt once measurement complete
#define LED_AS7341		false	//enable or disable LEDS



#endif /* INC_CONFIG_PARAMETERS_H_ */


