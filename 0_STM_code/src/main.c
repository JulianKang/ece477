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

void move_servo2(uint8_t scale, int position) {
	TIM1 -> CCR2 = position;

}
void move_servo3(uint8_t scale, int position) {
	TIM1 -> CCR3 = position;

}
void move_servo4(uint8_t scale, int position) {
	TIM1 -> CCR4 = position;

}

void move_servo5(uint8_t scale, int value) {
	TIM2 -> CCR2 = value;

}

//============================================================================
// setup_adc()
//============================================================================
void setup_adc(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOCEN; 			// enable gpio C
    GPIOC -> MODER |= 0x3F; 						// analog mode for gpio C0, C1, C2
    RCC -> APB2ENR |= RCC_APB2Periph_ADC1; 			// ADC clock enable
    RCC -> CR2 |= RCC_CR2_HSI14ON; 					// high speed internal (clock) 14 on
    while(RCC -> CIR & RCC_CIR_HSI14RDYF == 0); 	// hsi 14 ready
    ADC1->CFGR1 &= !(0x3 << 10);					// exten = 0
    ADC1 -> CR |= ADC_CR_ADEN; 						// adc enable
    ADC1->CFGR1 |= (0x1 << 13);						// continuous mode
    ADC1->CFGR1 |= (0x3);							// enable dma circular mode and enable dma
    while(ADC1 -> ISR & ADC_ISR_ADRDY == 0); 		// adc ready
    ADC1 -> CHSELR |= ADC_CHSELR_CHSEL10; 			// channel selection register: select 0
    ADC1 -> CHSELR |= ADC_CHSELR_CHSEL11; 			// channel selection register: select 1
    ADC1 -> CHSELR |= ADC_CHSELR_CHSEL12; 			// channel selection register: select 2
    ADC1->CR |= ADC_CR_ADSTART;
}

void adc_calib(void) {
	if ((ADC1->CR & ADC_CR_ADEN) != 0) /* (1) */
	{
	 ADC1->CR &= (uint32_t)(~ADC_CR_ADEN); /* (2) */
	}
	ADC1->CR |= ADC_CR_ADCAL; /* (3) */
	while ((ADC1->CR & ADC_CR_ADCAL) != 0) /* (4) */
	{
	 /* For robust implementation, add here time-out management */
	}
}

uint16_t visualizer[3] = {0x0, 0x0, 0x0};

void circular_mode_dma(void) {
	/* (1) Enable the peripheral clock on DMA */
	/* (2) Enable DMA transfer on ADC and circular mode */
	/* (3) Configure the peripheral data register address */
	/* (4) Configure the memory address */
	/* (5) Configure the number of DMA tranfer to be performs
	 on DMA channel 1 */
	/* (6) Configure increment, size, interrupts and circular mode */
	/* (7) Enable DMA Channel 1 */

	RCC->AHBENR |= RCC_AHBENR_DMA1EN; /* (1) */
	ADC1->CFGR1 |= ADC_CFGR1_DMAEN | ADC_CFGR1_DMACFG; /* (2) */
	DMA1_Channel1->CPAR = (uint32_t) (&(ADC1->DR)); /* (3) */
	DMA1_Channel1->CMAR = (uint32_t)(&display); /* (4) */
	DMA1_Channel1->CNDTR = 3; /* (5) */
	DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0
	 | DMA_CCR_TEIE | DMA_CCR_CIRC; /* (6) */
	DMA1_Channel1->CCR |= DMA_CCR_EN; /* (7) */
}

int main(void) {
    setup_tim1();
    setup_tim2();
    init_spi2();
    adc_calib();
    circular_mode_dma();
    setup_adc();

    uint8_t bpm;
    uint8_t start;

    while(1){
    	// check for dma ovr bit and reset if needed
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

				move_servo5(bpm, 1600);
				nano_wait(10000000000 / bpm);
				move_servo5(bpm, 6000);
				nano_wait(10000000000 /bpm);
			}
    }
}
