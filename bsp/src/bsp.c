/*
 * bsp.c
 *
 *  Created on: Mar 4, 2022
 *      Author: Laurent
 */


#include "stm32f7xx.h"
#include "bsp.h"


/*
 * BSP_LED_Init()
 * Initialize LED pin
 *
 * Nucleo LEDS
 * GREEN 	-> PB0
 * BLUE		-> PB7
 * RED		-> PB14
 *
 * Set LED initial state to OFF
 */

void BSP_LED_Init()
{
	// Start GPIOs Clocks
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOBEN;

	// Configure PB0, PB7, PB14 as high-speed PP outputs with no pull resistors
	GPIOB->MODER &= ~(GPIO_MODER_MODER0_Msk | GPIO_MODER_MODER7_Msk | GPIO_MODER_MODER14_Msk);
	GPIOB->MODER   |= (1U <<GPIO_MODER_MODER0_Pos) | (1U <<GPIO_MODER_MODER7_Pos) | (1U <<GPIO_MODER_MODER14_Pos);
	GPIOB->OSPEEDR |= (3U <<GPIO_OSPEEDR_OSPEEDR0_Pos) | (3U <<GPIO_OSPEEDR_OSPEEDR7_Pos) | (3U <<GPIO_OSPEEDR_OSPEEDR14_Pos);
	GPIOB->PUPDR &= ~(GPIO_PUPDR_PUPDR0_Msk | GPIO_PUPDR_PUPDR7_Msk | GPIO_PUPDR_PUPDR14_Msk);

	// Initial  state if off
	GPIOB->BSRR = GPIO_BSRR_BR0;
	GPIOB->BSRR = GPIO_BSRR_BR7;
	GPIOB->BSRR = GPIO_BSRR_BR14;
}




/*
 * BSP_LED_On()
 * Turn LED On
 */

void BSP_LED_On(uint8_t n)
{
	switch (n)
	{
		case 0:
		{
			GPIOB->BSRR = GPIO_BSRR_BS0;
			break;
		}

		case 1:
		{
			GPIOB->BSRR = GPIO_BSRR_BS7;
			break;
		}

		case 2:
		{
			GPIOB->BSRR = GPIO_BSRR_BS14;
			break;
		}
	}
}


/*
 * BSP_LED_Off()
 * Turn LED Off
 */

void BSP_LED_Off(uint8_t n)
{
	switch (n)
	{
		case 0:
		{
			GPIOB->BSRR = GPIO_BSRR_BR0;
			break;
		}

		case 1:
		{
			GPIOB->BSRR = GPIO_BSRR_BR7;
			break;
		}

		case 2:
		{
			GPIOB->BSRR = GPIO_BSRR_BR14;
			break;
		}
	}
}


/*
 * BSP_LED_Toggle()
 * Toggle LED
 */

void BSP_LED_Toggle(uint8_t n)
{
	switch (n)
	{
		case 0:
		{
			GPIOB->ODR ^= GPIO_ODR_ODR_0;
			break;
		}

		case 1:
		{
			GPIOB->ODR ^= GPIO_ODR_ODR_7;
			break;
		}

		case 2:
		{
			GPIOB->ODR ^= GPIO_ODR_ODR_14;
			break;
		}
	}
}


/*
 * BSP_PB_Init()
 * Initialize Push-Button pin (PC13) as input without Pull-up/Pull-down
 */

void BSP_PB_Init()
{
	// Enable GPIOC clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

	// Configure PC13 as input
	GPIOC->MODER &= ~GPIO_MODER_MODER13_Msk;
	GPIOC->MODER |= (0x00 <<GPIO_MODER_MODER13_Pos);

	// Disable PC13 Pull-up/Pull-down
	GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR13_Msk;
}


/*
 * BSP_PB_GetState()
 * Returns the state of the button (0=released, 1=pressed)
 */

uint8_t	BSP_PB_GetState()
{
	uint8_t state;

	if ((GPIOC->IDR & GPIO_IDR_IDR_13) == GPIO_IDR_IDR_13)
	{
		state = 1;
	}
	else
	{
		state = 0;
	}

	return state;
}


/*
 * BSP_Console_Init()
 * USART3 @ 115200 Full Duplex
 * 1 start - 8-bit - 1 stop
 * TX -> PD8 (AF7)
 * RX -> PD9 (AF7)
 */

void BSP_Console_Init()
{
	// Enable GPIOD clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;

	// Configure PD8, PD9 as AF mode
	GPIOD->MODER &= ~(GPIO_MODER_MODER8_Msk | GPIO_MODER_MODER9_Msk);
	GPIOD->MODER |=  (0x02 <<GPIO_MODER_MODER8_Pos) | (0x02 <<GPIO_MODER_MODER9_Pos);

	// Connect to USART3 TX (AF7)
	GPIOD->AFR[1] &= ~(0x000000FF);
	GPIOD->AFR[1] |=   0x00000077;

	// Enable USART3 Clock
	RCC->APB1ENR |= RCC_APB1ENR_USART3EN;

	// Disable USART3 and clears CR1 register
	// Default is 1S|8B|1S, no parity
	USART3->CR1 = 0x00000000;
	USART3->CR2 = 0x00000000;
	USART3->CR3 = 0x00000000;

	// Enable Receiver and Transmitter mode
	USART3->CR1 |= USART_CR1_RE | USART_CR1_TE;

	// Baudrate = 115200
	USART3->CR1 &= ~USART_CR1_OVER8;
	USART3->BRR = 0x1D4;

	// Enable USART3
	USART3->CR1 |= USART_CR1_UE;
}

/*
 * ADC_Init()
 * Initialize ADC for a dual channel conversion
 * on channel 14 and 15 -> pin PC4 and PC5
 */

extern uint8_t	adc_dma_buffer[2];

void BSP_ADC_Init()
{
	// Enable GPIOC clock
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOCEN;

    // Configure pin PC0 as analog (ADC channel 10) (joystick_1_X)
    GPIOC->MODER &= ~GPIO_MODER_MODER0_Msk;
    GPIOC->MODER |= (0x03 << GPIO_MODER_MODER0_Pos);
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR0_Msk;

    // Configure pin PC3 as analog (ADC channel 13) (joystick_1_Y)
    GPIOC->MODER &= ~GPIO_MODER_MODER3_Msk;
    GPIOC->MODER |= (0x03 << GPIO_MODER_MODER3_Pos);
    GPIOC->PUPDR &= ~GPIO_PUPDR_PUPDR3_Msk;

	// Enable ADC clock
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

    // Reset ADC configuration
    ADC ->CCR   = 0x00000000;
    ADC1->CR1 	= 0x00000000;
    ADC1->CR2 	= 0x00000000;
    ADC1->SQR3  = 0x00000000;
    ADC1->SQR3  = 0x00000000;
    ADC1->SQR3  = 0x00000000;
    ADC1->SMPR1 = 0x00000000;
    ADC1->SMPR2 = 0x00000000;

    // Enable scan mode (mandatory for multiple channels)
    ADC1->CR1 |= ADC_CR1_SCAN;

    // Enable continuous conversion mode
    ADC1->CR2 |= ADC_CR2_CONT;

    // Enable EOC
    ADC1->CR2 |= ADC_CR2_EOCS;

    // 8-bit resolution
    ADC1->CR1 |= (0x02 <<ADC_CR1_RES_Pos);

    // Select PCLK/2 as ADC clock
    ADC->CCR |= (0x00 << ADC_CCR_ADCPRE_Pos);

    // Set sampling time to ADC clock cycles
    ADC1->SMPR1 |= (0x02 << ADC_SMPR1_SMP10_Pos);
    ADC1->SMPR1 |= (0x02 << ADC_SMPR1_SMP13_Pos);

    // Sequence length = 2 conversions (L = 1 means 2 ranks)
    ADC1->SQR1 = (0x01 << ADC_SQR1_L_Pos);

    /* Rank 1 = channel 10
	   Rank 2 = channel 13 */
    ADC1->SQR3 = (10 << ADC_SQR3_SQ1_Pos) | (13 << ADC_SQR3_SQ2_Pos);

    // Start DMA clock
    RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

    // Reset DMA1 Channel 2 configuration
    DMA2_Stream0->CR = 0x00000000;

    // Configure DMA2 ; Stream 0 ; Channel 0
    // Peripheral -> Memory
    // Peripheral is 8-bit, no increment
    // Memory is 8-bit, increment
    // Circular mode

    // Select Channel 0
    DMA2_Stream0->CR |= (0x00 << DMA_SxCR_CHSEL_Pos);

    // Enable circular mode and memory increment
    DMA2_Stream0->CR |= (DMA_SxCR_CIRC | DMA_SxCR_MINC);

    // Configure sizes and increments
    DMA2_Stream0->CR |= ((0x00 << DMA_SxCR_PSIZE_Pos) | (0x00 << DMA_SxCR_MSIZE_Pos));

    // Peripheral is ADC1 DR
    DMA2_Stream0->PAR = (uint32_t)&ADC1->DR;

    // Memory is adc_dma_buffer
    DMA2_Stream0->M0AR = (uint32_t)adc_dma_buffer;

    // Set Memory Buffer size
    DMA2_Stream0->NDTR = 2;

    // Set data transfer direction
    DMA2_Stream0->CR |= (0x00 << DMA_SxCR_DIR_Pos);

    // Enable DMA2 Channel 0
    DMA2_Stream0->CR |= DMA_SxCR_EN;

    // Enable ADC1 ; DMA and DMA request
    ADC1->CR2 |= (ADC_CR2_ADON | ADC_CR2_DMA | ADC_CR2_DDS);

    // Start conversion on ADC1
    ADC1->CR2 |= ADC_CR2_SWSTART;
}




