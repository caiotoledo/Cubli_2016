/*
 * LCD.c
 *
 * Created: 12/04/2016 21:55:31
 *  Author: Caio
 */ 
#include "LCD.h"
#include <string.h>

#define QUEUE_WAIT		(portMAX_DELAY)

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
		xQueueAcel[i] = xQueueCreate(1, sizeof(float));
		if (xQueueAcel[i] == NULL){
			LED_On(LED2_GPIO);
			while(1);
		}
		xQueueGyro[i] = xQueueCreate(1, sizeof(float));
		if (xQueueGyro[i] == NULL){
			LED_On(LED2_GPIO);
			while(1);
		}
	}
}

void LCDTask(void *pvParameters){
	UNUSED(pvParameters);
	
	float lcd_acel[3];
	float lcd_gyro[3];
	char lcd_buf[40];
	uint8_t i = 0;
	signed portBASE_TYPE statusQueue;
	
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(5,10, (uint8_t *)"IMU FreeRTOS");
	
	for (;;){
		vTaskDelay(TASK_DELAY);
		
		memset(lcd_acel, 0, sizeof(lcd_acel));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueueReceive(xQueueAcel[i], &(lcd_acel[i]),QUEUE_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		memset(lcd_gyro, 0, sizeof(lcd_gyro));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueueReceive(xQueueGyro[i], &(lcd_gyro[i]),QUEUE_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		ili9225_set_foreground_color(COLOR_WHITE);
		ili9225_draw_filled_rectangle(0,30,ILI9225_LCD_WIDTH,ILI9225_LCD_HEIGHT);
		
		ili9225_set_foreground_color(COLOR_BLACK);
		sprintf(lcd_buf, "Acel:\nX= %0.3f\nY= %0.3f\nZ= %0.3f", lcd_acel[0], lcd_acel[1], lcd_acel[2]);
		ili9225_draw_string(5,30, lcd_buf);
		sprintf(lcd_buf, "Gyro:\nX= %0.3f\nY= %0.3f\nZ= %0.3f", lcd_gyro[0], lcd_gyro[1], lcd_gyro[2]);
		ili9225_draw_string(5,110, lcd_buf);
		sprintf(lcd_buf,"%u bytes Free", (uint32_t)xPortGetFreeHeapSize());
		ili9225_draw_string(5,190, lcd_buf);
	}
}