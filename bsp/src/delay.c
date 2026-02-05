/*
 * delay.c
 *
 *  Created on: 6 ao�t 2017
 *      Author: Laurent
 */

#include "delay.h"

/*
 *  Basic delay functions
 */

void delay_ms(uint32_t delay)
{
	uint32_t	i;
	for(i=0; i<(delay*8500); i++);		// Tuned for ms @ 216MHz
}

void delay_us(uint32_t delay)
{
	uint32_t	i;
	for(i=0; i<(delay*8); i++);			// Tuned for �s
}

