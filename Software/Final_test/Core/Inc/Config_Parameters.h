/*
 * Config_Parameters.h
 *
 *  Created on: Sep 21, 2023
 *      Author: Sarah
 */

#ifndef INC_CONFIG_PARAMETERS_H_
#define INC_CONFIG_PARAMETERS_H_

#define t_meas	500 	//Time between measurements, in ms
						//Options are: 100, 500, 1000, 5000, 10000

#define MODE 	eSpm	//eSpm -> default setting, spectral measurement, no sync
						//eSyns -> integration with external start, spectral measurement, start sync

#define t_int 	2000	//Integration time, in ms
						//Options are: 10, 20, 50, 100, 200, 500, 1000, 2000
						//default to 50ms

#define AGAIN 	6		//Gain between 0 and 10, corresponds to following values
						//[0 -> 0.5x, 1 -> 1x, 2 -> 2x, 3 -> 4x, 4 -> 8x, 5 -> 16x, 7 -> 64x, 8 -> 128x, 9 -> 256x, 10 -> 512x

#define INT 	false 	//enable or disable interrupt mode (generates interrupt once measurement complete
#define LED		false	//enable or disable LEDS



#endif /* INC_CONFIG_PARAMETERS_H_ */


