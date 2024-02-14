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

#define SYSTICK_LOAD (SystemCoreClock/1000000U)
#define SYSTICK_DELAY_CALIB (SYSTICK_LOAD >> 1)

#define DELAY_US(us) \
    do { \
         uint32_t start = SysTick->VAL; \
         uint32_t ticks = (us * SYSTICK_LOAD)-SYSTICK_DELAY_CALIB;  \
         while((start - SysTick->VAL) < ticks); \
    } while (0)

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
//    GPIOA -> MODER &= ~0xff0000;
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
void setup_tim2(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOAEN;
//    GPIOA -> MODER &= ~0xff000f;
    GPIOA -> MODER |= 0x000008;
    GPIOA -> AFR[0] &= ~0xffff;
    GPIOA -> AFR[0] |= 0x0020;

    RCC -> APB1ENR |= RCC_APB1ENR_TIM2EN;
    TIM2 -> BDTR |= TIM_BDTR_MOE;
    TIM2 -> PSC = 20 - 1;
    TIM2 -> ARR = 48000 - 1;

    TIM2 -> CCMR1 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;
    TIM2 -> CCMR2 |= TIM_CCMR1_OC1M_1 | TIM_CCMR1_OC1M_2 | TIM_CCMR1_OC2M_1 | TIM_CCMR1_OC2M_2;

    TIM2 -> CCER |= TIM_CCER_CC1E | TIM_CCER_CC2E | TIM_CCER_CC3E | TIM_CCER_CC4E;

    TIM2 -> CR1 |= TIM_CR1_CEN;
}

void move_servo1(uint8_t scale, int position) {
	TIM1 -> CCR1 = position;

}

void move_servo2(uint8_t scale) {
	TIM1 -> CCR2 = 1400;
	nano_wait(10000000000 / scale); // 1 second
	TIM1 -> CCR2 = 6000;
	nano_wait(10000000000 / scale);

}

void move_servo3(uint8_t scale) {
	TIM1 -> CCR3 = 1400;
	nano_wait(10000000000 / scale); // 1 second
	TIM1 -> CCR3 = 6000;
	nano_wait(10000000000 / scale);

}

void move_servo4(uint8_t scale) {
	TIM1 -> CCR4 = 1400;
	nano_wait(10000000000 / scale); // 1 second
	TIM1 -> CCR4 = 6000;
	nano_wait(10000000000 / scale);

}

void move_servo5(uint8_t scale, int value) {
	TIM2 -> CCR2 = value;

}

int main(void) {
    setup_tim1();
    setup_tim2();
    init_spi2();

    uint8_t bpm;
    uint8_t start;

    while(1){
		while ((GPIOB->IDR & (0x1 << 7)) == (0x1 << 7)) {
			while(SPI2->DR == 0);
	        	bpm = SPI2->DR;
				if (bpm == 0) {
					// don't let the BPM be 0
					bpm = 1;
				}

				// run dance routine, wait times scaled by the BPM
				move_servo1(bpm, 6000);
				nano_wait(10000000000 / bpm); // 1 second
				move_servo1(bpm, 5000);
				nano_wait(10000000000 / bpm);
	//            move_servo2(bpm);
	//            move_servo3(bpm);
	//            move_servo4(bpm);
				move_servo5(bpm, 1600);
				nano_wait(10000000000 / bpm);
				move_servo5(bpm, 6000);
				nano_wait(10000000000 /bpm);
			}
    }
}
