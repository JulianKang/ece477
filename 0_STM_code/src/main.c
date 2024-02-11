/**
  ******************************************************************************
  * @file    main.c
  * @author  Weili An
  * @version V1.0
  * @date    Nov 4, 2022
  * @brief   ECE 362 Lab 9 Student template
  ******************************************************************************
*/

#include "stm32f0xx.h"
#include <math.h>   // for M_PI

void nano_wait(int);

//extern const char font[];
// Print an 8-character string on the 8 digits
void print(const char str[]);
// Print a floating-point value.
void printfloat(float f);

void setup_spi_master(void) {
	/* (1) Master selection, BR: Fpclk/256 (due to C27 on the board, SPI_CLK is
	 set to the minimum) CPOL and CPHA at zero (rising first edge) */
	/* (2) Slave select output enabled, RXNE IT, 8-bit Rx fifo */
	/* (3) Enable SPI1 */
	SPI1->CR1 = SPI_CR1_MSTR | SPI_CR1_BR; /* (1) */
	SPI1->CR2 = SPI_CR2_SSOE | SPI_CR2_RXNEIE | SPI_CR2_FRXTH
	 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; /* (2) */
	SPI1->CR1 |= SPI_CR1_SPE; /* (3) */
}

void setup_spi_slave(void) {
	/* nSS hard, slave, CPOL and CPHA at zero (rising first edge) */
	/* (1) RXNE IT, 8-bit Rx fifo */
	/* (2) Enable SPI2 */
	SPI2->CR2 = SPI_CR2_RXNEIE | SPI_CR2_FRXTH
	 | SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; /* (1) */
	SPI2->CR1 |= SPI_CR1_SPE; /* (2) */
}

//===========================================================================
// Initialize the SPI2 peripheral.
//===========================================================================
void init_spi2(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;	// enable the GPIO B clock
    GPIOB -> MODER |= 0x8a << 24;			// set pins 12, 13, and 15 to alternate function mode
    RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;	// enable the SPI2 clock
    SPI2 -> CR1 &= ~SPI_CR1_SPE; 		// clear SPE so we can enable things

    SPI2 -> CR1 |= SPI_CR1_BR; 			// set baud rate to f/256
    SPI2 -> CR1 |= SPI_CR1_RXONLY; 		// receive only mode

	SPI2->CR2 |= SPI_CR2_RXNEIE;		// enable buffer not empty interupt
	SPI2->CR2 |= SPI_CR2_FRXTH;			// RXNE event is generated if the FIFO level is greater than or equal to 1/4 (8-bit)
	SPI2->CR2 |= SPI_CR2_DS_2 | SPI_CR2_DS_1 | SPI_CR2_DS_0; // set data size to 8-bit word

    SPI2 -> CR1 |= SPI_CR1_SPE;			// re-enable SPE now that it's configured

}

void setup_tim1(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
    GPIOA -> MODER &= ~0xff0000;
    GPIOA -> MODER |= 0xaa0000;
    GPIOA -> AFR[1] &= ~0xffff;
    GPIOA -> AFR[1] |= 0x2222;

    RCC -> APB2ENR |= RCC_APB2ENR_TIM1EN;
    TIM1 -> BDTR |= TIM_BDTR_MOE;
    TIM1 -> PSC = 20 - 1;
    TIM1 -> ARR = 48000 - 1;

    TIM1 -> CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM1 -> CCMR2 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

    TIM1 -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM1 -> CR1 |= TIM_CR1_CEN;
}

void move_servo1(uint8_t scale) {
	TIM1 -> CCR1 = 3400;
	nano_wait(5000000000 / scale);
	TIM1 -> CCR1 = 2600;
	nano_wait(5000000000 / scale);

}

//int _write(char ptr, int len)
//{
//  int i=0;
//  for(i=0 ; i<len ; i++)
//    ITM_SendChar((ptr++));
//  return len;
//}

int main(void) {
    setup_tim1();
    init_spi2();

    uint8_t bpm;

    while (1) {
        	bpm = SPI2->DR;
            if (bpm == 0) {
            	// don't let the BPM be 0
            	bpm = 1;
            }

            // run dance routine, wait times scaled by the BPM
            move_servo1(bpm);
            move_servo1(bpm);

            _write(bpm, 8);
        }
}
