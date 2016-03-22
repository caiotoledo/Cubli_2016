/**
 * \file
 *
 * \brief Empty user application template
 *
 */

/**
 * \mainpage User Application template doxygen documentation
 *
 * \par Empty user application template
 *
 * Bare minimum empty user application template
 *
 * \par Content
 *
 * -# Include the ASF header files (through asf.h)
 * -# "Insert system clock initialization code here" comment
 * -# Minimal main function that starts with a call to board_init()
 * -# "Insert application code here" comment
 *
 */

/*
 * Include header files for all drivers that have been imported from
 * Atmel Software Framework (ASF).
 */
/*
 * Support and FAQ: visit <a href="http://www.atmel.com/design-support/">Atmel Support</a>
 */
#include <asf.h>

#define ID_ENC		ID_PIOA
#define GPIO_A		PIO_PA1
#define GPIO_B		PIO_PA2
#define GPIO_C		PIO_PA3

volatile uint32_t g_ul_ms_ticks = 0;

void SysTick_Handler(void){
	g_ul_ms_ticks++;
}

void pin_handler(uint32_t id, uint32_t mask){
	if (id == ID_ENC){
			switch (mask){
				case GPIO_A:
					gpio_toggle_pin(LED0_GPIO);
				break;
				case GPIO_B:
					gpio_toggle_pin(LED1_GPIO);
				break;
				case GPIO_C:
					gpio_toggle_pin(LED2_GPIO);
				break;
			}
	}
}

int main (void)
{
	/* Insert system clock initialization code here (sysclk_init()). */

	sysclk_init();
	board_init();

	/* Insert application code here, after the board has been initialized. */
	
	if (SysTick_Config(sysclk_get_cpu_hz() / 1000)) {
		puts("-E- Systick configuration error\r");
		while (1) {
			/* Capture error */
			gpio_set_pin_low(LED2_GPIO);
		}
	}
	
	pmc_enable_periph_clk(ID_ENC);
	
	//Interrupt for A:
	pio_set_input(PIOA, GPIO_A, PIO_DEFAULT);
	pio_pull_down(PIOA, GPIO_A, ENABLE);
	pio_handler_set(PIOA, ID_ENC, GPIO_A, PIO_IT_RISE_EDGE, pin_handler);
	pio_enable_interrupt(PIOA, GPIO_A);
	
	//Interrupt for B:
	pio_set_input(PIOA, GPIO_B, PIO_DEFAULT);
	pio_pull_down(PIOA, GPIO_B, ENABLE);
	pio_handler_set(PIOA, ID_ENC, GPIO_B, PIO_IT_RISE_EDGE, pin_handler);
	pio_enable_interrupt(PIOA, GPIO_B);
	
	//Interrupt for C:
	pio_set_input(PIOA, GPIO_C, PIO_DEFAULT);
	pio_pull_down(PIOA, GPIO_C, ENABLE);
	pio_handler_set(PIOA, ID_ENC, GPIO_C, PIO_IT_RISE_EDGE, pin_handler);
	pio_enable_interrupt(PIOA, GPIO_C);
	
	//Enable Interrupt GPIO:
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 0);
	NVIC_EnableIRQ(PIOA_IRQn);
	
	gpio_set_pin_low(LED2_GPIO);
	
	while (1){
		
	}
}
