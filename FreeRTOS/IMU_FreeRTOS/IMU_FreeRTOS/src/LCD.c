/*
 * LCD.c
 *
 * Created: 12/04/2016 21:55:31
 *  Author: Caio
 */ 
#include "LCD.h"
#include <string.h>

#define LCD_WAIT		(portMAX_DELAY)

#define TASK_DELAY		(500/portTICK_RATE_MS)

struct ili9225_opt_t g_ili9225_display_opt;
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
	ili9225_fill( (ili9225_color_t) COLOR_WHITE);
	
	/* Queue FreeRTOS Initialization */
	uint8_t i;
	for (i = 0; i < NUM_AXIS; i++){
		xQueueAcel[i] = xQueueCreate(1, sizeof(double));
		if (xQueueAcel[i] == NULL){
			LED_On(LED2_GPIO);
			while(1);
		}
		xQueueAngle[i] = xQueueCreate(1, sizeof(double));
		if (xQueueAngle[i] == NULL){
			LED_On(LED2_GPIO);
			while(1);
		}
		xQueueGyro[i] = xQueueCreate(1, sizeof(double));
		if (xQueueGyro[i] == NULL){
			LED_On(LED2_GPIO);
			while(1);
		}
	}
}

void LCDTask(void *pvParameters){
	UNUSED(pvParameters);
	
	double lcd_acel[3];
	double lcd_angle[3];
	double lcd_gyro[3];
	char lcd_buf[40];
	uint8_t i = 0;
	signed portBASE_TYPE statusQueue;
	
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(5,10, (uint8_t *)"IMU FreeRTOS");
	
	for (;;){
		vTaskDelay(TASK_DELAY);
		
		memset(lcd_acel, 0, sizeof(lcd_acel));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueAcel[i], &(lcd_acel[i]),LCD_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		memset(lcd_angle, 0, sizeof(lcd_angle));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueAngle[i], &(lcd_angle[i]),LCD_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		memset(lcd_gyro, 0, sizeof(lcd_gyro));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueGyro[i], &(lcd_gyro[i]),LCD_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		ili9225_set_foreground_color(COLOR_WHITE);
		ili9225_draw_filled_rectangle(0,30,ILI9225_LCD_WIDTH,ILI9225_LCD_HEIGHT);
		
		ili9225_set_foreground_color(COLOR_BLACK);
		sprintf(lcd_buf, "Acel:\nX=%0.3f mG\nY=%0.3f mG\nGyro:\n%0.3f Graus/s", lcd_acel[0], lcd_acel[1],lcd_gyro[2]);
		ili9225_draw_string(5,30, lcd_buf);
		sprintf(lcd_buf, "Angle Pure:\n%0.3f\nAngle Comp.:\n%0.3f\nKalman:\n%0.3f", lcd_angle[0], lcd_angle[1],lcd_angle[2]);
		ili9225_draw_string(5,110, lcd_buf);
		/*sprintf(lcd_buf,"%u bytes Free", (uint32_t)xPortGetFreeHeapSize());
		ili9225_draw_string(5,200, lcd_buf);*/
	}
}