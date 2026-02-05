/*
 * bsp.h
 *
 *  Created on: Mar 4, 2022
 *      Author: Laurent
 */

#ifndef INC_BSP_H_
#define INC_BSP_H_


// LED functions

#define	LEDN_GREEN	0
#define LEDN_BLUE	1
#define	LEDN_RED	2

void 	BSP_LED_Init			(void);
void 	BSP_LED_On				(uint8_t n);
void 	BSP_LED_Off				(uint8_t n);
void 	BSP_LED_Toggle			(uint8_t n);

// Console functions

void 	BSP_Console_Init		(void);

// Push-button functions

void 	BSP_PB_Init				(void);
uint8_t	BSP_PB_GetState			(void);

// ADC init
void BSP_ADC_Init				(void);



#endif /* INC_BSP_H_ */
