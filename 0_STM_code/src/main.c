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

struct Movement {   // Structure declaration
  int interval_count;
  int current_count;
  int CCR_delta1;
  int CCR_delta2;
  int CCR_delta3;
  int CCR_delta4;
  int CCR_delta5;
}; // End the structure with a semicolon

struct Pose {   // Structure declaration
  int duration;
  int angle1;
  int angle2;
  int angle3;
  int angle4;
  int angle5;
}; // End the structure with a semicolon

void nano_wait(int); varia[0][1]

// Global Variables
int CCR_value1, CCR_value2, CCR_value3, CCR_value4, CCR_value5;
uint16_t visualizer[3] = {0x0, 0x0, 0x0};
int ADC_sigmoid = 0; // fix later
struct Movement window_array[20];
int play_mask = (0x1 << 7);
uint8_t bpm;
int move_index, calc_index;
int ARRAY_LENGTH;

struct Pose dance_file[4];

// --------------------------- INIT FUNCTIONS FOR MAIN -----------------------------------

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

// Initialize the SPI2 peripheral.
void init_spi2(void) {
    RCC -> AHBENR |= RCC_AHBENR_GPIOBEN;	// enable the GPIO B clock
    GPIOB -> MODER |= 0x8a << 24;			// set pins 12, 13, and 15 to alternate function mode
    RCC -> APB1ENR |= RCC_APB1ENR_SPI2EN;	// enable the SPI2 clock
    SPI2 -> CR1 &= ~SPI_CR1_SPE; 		// clear SPE so we can enable things

    SPI2 -> CR1 |= SPI_CR1_BR; 			// set baud rate to f/256
    SPI2 -> CR1 |= SPI_CR1_RXONLY; 		// receive only mode

	SPI2->CR2 |= SPI_CR2_RXNEIE;		// enable buffer not empty interupt
	SPI2->CR2 |= SPI_CR2_FRXTH;			// RXNE event is generated if the FIFO level is greater than or
										// equal to 1/4 (8-bit)
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

// setup_adc()
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
	DMA1_Channel1->CMAR = (uint32_t)(&visualizer); /* (4) */
	DMA1_Channel1->CNDTR = 3; /* (5) */
	DMA1_Channel1->CCR |= DMA_CCR_MINC | DMA_CCR_MSIZE_0 | DMA_CCR_PSIZE_0
	 | DMA_CCR_TEIE | DMA_CCR_CIRC; /* (6) */
	DMA1_Channel1->CCR |= DMA_CCR_EN; /* (7) */
}


/**
 * @brief Set the SysTick interrupt to occur every 1/10 seconds.
 * DOES NOT ENABLE
 */
void init_systick() {
    SysTick -> LOAD = 600000-1; 		// 600000 / 6000000 = 0.1 seconds
    // do not enable here!!! enable separately.
    SysTick -> CTRL |= /*SysTick_CTRL_ENABLE_Msk |*/ SysTick_CTRL_TICKINT_Msk;
}

void init_exti(void) {
    RCC -> APB2ENR |= RCC_APB2ENR_SYSCFGCOMPEN;
    SYSCFG -> EXTICR[1] |= SYSCFG_EXTICR2_EXTI7_PB;
    EXTI -> RTSR |= EXTI_RTSR_TR7;
    EXTI -> FTSR |= EXTI_FTSR_TR7;
    EXTI -> IMR |= EXTI_IMR_MR7;
    NVIC -> ISER[0] |= 0b11100000; // idk what this is doing
}

// ------------------------------- INTERRUPT HANDLERS ---------------------------------------

void EXTI4_15_IRQHandler()
{
    EXTI -> PR &= EXTI_PR_PR7;
    if((GPIOB->IDR & play_mask) == 0x0) {
    	// play is off
    	disable_motor_interrupt();
    }
    else {
    	// wait till we have the bpm
        while(SPI2->DR == 0);
        // if it's the same as the last bpm, restart the motors
        if(SPI2->DR == bpm) {
        	enable_motor_interrupt();
        }
        else {
        	// otherwise, set the bpm to new value and setup the first column so that the motors can run.
        	bpm = SPI2->DR;
        	first_run();
        	manage_dance_file();
        }
    }
}

/**
 * @brief The ISR for the SysTick interrupt. Moves the servos to desired position
 */
void SysTick_Handler() {
	// check if we are using the correct index
	// if we are at the interval count desired, move to next column.
	if(window_array[move_index].current_count == window_array[move_index].interval_count) {
		if(move_index == ARRAY_LENGTH) {
			move_index = 0;
		}
		else {
			move_index = move_index + 1;
		}
	}

	// set the CCR values
	CCR_value1 = get_new_CCR_value(CCR_value1, window_array[move_index].CCR_delta1, ADC_sigmoid);
	CCR_value2 = get_new_CCR_value(CCR_value2, window_array[move_index].CCR_delta2, ADC_sigmoid);
	CCR_value3 = get_new_CCR_value(CCR_value3, window_array[move_index].CCR_delta3, ADC_sigmoid);
	CCR_value4 = get_new_CCR_value(CCR_value4, window_array[move_index].CCR_delta4, ADC_sigmoid);
	CCR_value5 = get_new_CCR_value(CCR_value5, window_array[move_index].CCR_delta5, ADC_sigmoid);

	// move the servos to new CCR
	move_servo(1, CCR_value1);
	move_servo(2, CCR_value2);
	move_servo(3, CCR_value3);
	move_servo(4, CCR_value4);
	move_servo(5, CCR_value5);

//	move_servo(1, window_array[motor_index][0]);
//	move_servo(2, window_array[motor_index][1]);
//	move_servo(3, window_array[motor_index][2]);
//	move_servo(4, window_array[motor_index][3]);
//	move_servo(5, window_array[motor_index][4]);
//
//	motor_index = motor_index + 1;
}

// ------------------------------------ HELPER FUNCTIONS -------------------------------------

void move_servo(int servo, int position) {
	switch(servo) {
		case 1: TIM1 -> CCR1 = position;
				break;
		case 2: TIM1 -> CCR2 = position;
				break;
		case 3: TIM1 -> CCR3 = position;
				break;
		case 4: TIM1 -> CCR4 = position;
				break;
		case 5: TIM2 -> CCR2 = position;
				break;
		default: break;
	}

}

// -------------------------------- DANCE FILE FUNCTIONS -----------------------------

int get_angle_delta(int current_angle, int previous_angle, int time_increments) {
	return current_angle - previous_angle / time_increments;
}

int get_CCR_value(int angle) {
	return 40.448 + (0.037 * angle);
}

int get_new_CCR_value(int CCR_value, int CCR_delta, int ADC_sigmoid) {
	// set new ccr value using delta and sigmoid function
	int new_CCR_value = CCR_value + CCR_delta + ADC_sigmoid;

	// fail safe to not burn out motor
	if(new_CCR_value > 7000) {
		new_CCR_value = 7000;
	}
	else if(new_CCR_value < 1400) {
		new_CCR_value = 1400;
	}

	return new_CCR_value;
}

void manage_section( int converted_time, int previous_angle1, int current_angle1, int previous_angle2,
		int current_angle2, int previous_angle3, int current_angle3, int previous_angle4, int current_angle4,
		int previous_angle5, int current_angle5) {

	window_array[calc_index].interval_count = converted_time / 0.1;
	window_array[calc_index].current_count = 0;

	// motor 1 values
	int angle_delta1 = get_angle_delta(current_angle1, previous_angle1, window_array[calc_index].interval_count);
	window_array[calc_index].CCR_delta1 = get_CCR_value(angle_delta1);
	// motor 2 values
	int angle_delta2 = get_angle_delta(current_angle2, previous_angle2, window_array[calc_index].interval_count);
	window_array[calc_index].CCR_delta2 = get_CCR_value(angle_delta2);
	// motor 3 values
	int angle_delta3 = get_angle_delta(current_angle3, previous_angle3, window_array[calc_index].interval_count);
	window_array[calc_index].CCR_delta3 = get_CCR_value(angle_delta3);
	// motor 4 values
	int angle_delta4 = get_angle_delta(current_angle4, previous_angle4, window_array[calc_index].interval_count);
	window_array[calc_index].CCR_delta4 = get_CCR_value(angle_delta4);
	// motor 5 values
	int angle_delta5 = get_angle_delta(current_angle5, previous_angle5, window_array[calc_index].interval_count);
	window_array[calc_index].CCR_delta5 = get_CCR_value(angle_delta5);

}

// --------------------------------------- MAIN LOOP FUNCTIONS --------------------------------------

void first_run(void) {
	calc_index = 0;
//	manage_section();
	calc_index = 1;
	move_index = 0;

	// enable interrupt
	enable_motor_interrupt();
}

void enable_motor_interrupt(void) {
    SysTick -> CTRL |= SysTick_CTRL_ENABLE_Msk;
}
void disable_motor_interrupt(void) {
    SysTick -> CTRL &= ~SysTick_CTRL_ENABLE_Msk;
}

void manage_dance_file(void) {
	int multiplier = 80 / bpm;
	while(calc_index < ARRAY_LENGTH) {
		manage_section(dance_file[calc_index].duration * multiplier,
				dance_file[calc_index - 1].angle1, dance_file[calc_index].angle1,
				dance_file[calc_index - 1].angle2, dance_file[calc_index].angle2,
				dance_file[calc_index - 1].angle3, dance_file[calc_index].angle3,
				dance_file[calc_index - 1].angle4, dance_file[calc_index].angle4,
				dance_file[calc_index - 1].angle5, dance_file[calc_index].angle5);
	}
}

int main(void) {
    setup_tim1();
    setup_tim2();
    init_spi2();
    adc_calib();
    circular_mode_dma();
    setup_adc();
    init_systick();
    init_exti();

    uint8_t start;

    while(1){
    	//
    	// check for dma ovr bit and reset if needed

//    	move_servo(1, 6000);
//    	move_servo(2, 1600);
//    	move_servo(3, 1600);
//    	nano_wait(10000000000); // 1 second
//
//    	move_servo(1, 5000);
//    	move_servo(2, 6000);
//    	move_servo(3, 2600);
//    	nano_wait(10000000000); // 1 second

//		while ((GPIOB->IDR & (0x1 << 7)) == (0x1 << 7)) {
////			while(SPI2->DR == 0);
//	        	bpm = SPI2->DR;
////				if (bpm == 0) {
////					// don't let the BPM be 0
////					bpm = 1;
////				}
////				if(bpm == 100) {
////					start = 1;
////				}
//
//				// run dance routine, wait times scaled by the BPM
////				move_servo1(bpm, 6000);
////				nano_wait(10000000000 / bpm); // 1 second
////				move_servo1(bpm, 5000);
////				nano_wait(10000000000 / bpm);
////
////				move_servo5(bpm, 1600);
////				nano_wait(10000000000 / bpm);
////				move_servo5(bpm, 6000);
////				nano_wait(10000000000 /bpm);
//			}
    }
}
