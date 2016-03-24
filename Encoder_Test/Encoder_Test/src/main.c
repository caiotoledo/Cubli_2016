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
#define GPIO_C		PIO_PA4

#define ENC_RES		1024
#define VOLTA_COMP	360

volatile uint32_t g_ul_ms_ticks = 0;
struct ili9225_opt_t g_ili9225_display_opt;

uint32_t aFlag = 0;
uint32_t bFlag = 0;

int32_t round = 0;

float graus = 0;

uint8_t buf[100] = {0};

void SysTick_Handler(void){
	g_ul_ms_ticks++;
}

static void mdelay(uint32_t ul_dly_ticks)
{
	uint32_t ul_cur_ticks;

	ul_cur_ticks = g_ul_ms_ticks;
	while ((g_ul_ms_ticks - ul_cur_ticks) < ul_dly_ticks);
}

/**
 * \brief Override SPI handler.
 */
void SPI_Handler(void)
{
	ili9225_spi_handler();
}

void config_lcd(void){
	
	/* Initialize display parameter */
	g_ili9225_display_opt.ul_width = ILI9225_LCD_WIDTH;
	g_ili9225_display_opt.ul_height = ILI9225_LCD_HEIGHT;
	g_ili9225_display_opt.foreground_color = COLOR_BLACK;
	g_ili9225_display_opt.background_color = COLOR_WHITE;

	/* Switch off backlight */
	aat31xx_disable_backlight();

	/* Initialize LCD */
	ili9225_init(&g_ili9225_display_opt);

	/* Set backlight level */
	aat31xx_set_backlight(AAT31XX_AVG_BACKLIGHT_LEVEL);

	/* Turn on LCD */
	ili9225_display_on();
	
	/* Draw filled rectangle with white color */
	//ili9225_set_foreground_color(COLOR_WHITE);
	ili9225_fill(COLOR_WHITE);
	//ili9225_draw_filled_rectangle(0, 0, ILI9225_LCD_WIDTH, ILI9225_LCD_HEIGHT);
}

void pin_handler(uint32_t id, uint32_t mask){
	if (id == ID_ENC){
			switch (mask){
				case GPIO_A:
				if (bFlag){
					gpio_toggle_pin(LED0_GPIO);
					gpio_set_pin_high(LED1_GPIO);
					round++;
				} else {
					aFlag = 1;
				}
				break;
				case GPIO_B:
				if (aFlag){
					gpio_toggle_pin(LED1_GPIO);
					gpio_set_pin_high(LED0_GPIO);
					round--;
				} else {
					bFlag = 1;
				}
				break;
				case GPIO_C:
					gpio_toggle_pin(LED2_GPIO);
					bFlag = 0;
					aFlag = 0;
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
	
	config_lcd();
	
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
	//pio_handler_set(PIOA, ID_ENC, GPIO_C, PIO_IT_RISE_EDGE, pin_handler);
	pio_handler_set(PIOA, ID_ENC, GPIO_C, PIO_IT_FALL_EDGE, pin_handler);
	pio_enable_interrupt(PIOA, GPIO_C);
	
	//Enable Interrupt GPIO:
	NVIC_DisableIRQ(PIOA_IRQn);
	NVIC_ClearPendingIRQ(PIOA_IRQn);
	NVIC_SetPriority(PIOA_IRQn, 0);
	NVIC_EnableIRQ(PIOA_IRQn);
	
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(10,10, (uint8_t *)"Encoder Test");
	
	while (1){
		ili9225_set_foreground_color(COLOR_WHITE);
		ili9225_draw_filled_rectangle(0,30,ILI9225_LCD_WIDTH,ILI9225_LCD_HEIGHT);
		ili9225_set_foreground_color(COLOR_BLACK);
		graus = ((float)round/ENC_RES)*VOLTA_COMP;
		snprintf(buf, sizeof(buf), "Graus:%.3f", graus );
		ili9225_draw_string(10,50, buf);
		snprintf(buf, sizeof(buf), "Round:%d", round );
		ili9225_draw_string(10,80, buf);
		mdelay(500);
	}
}
