/*
 * main.h
 *
 *  Created on: Mar 4, 2022
 *      Author: Laurent
 */

#ifndef INC_MAIN_H_
#define INC_MAIN_H_


// MCU Header
#include "stm32f7xx.h"


// BSP Headers
#include "bsp.h"
#include "delay.h"
#include "usb.h"


// APP Headers


// Global functions
int 	my_printf	(const char *format, ...);
int 	my_sprintf	(char *out, const char *format, ...);



#endif /* INC_MAIN_H_ */
