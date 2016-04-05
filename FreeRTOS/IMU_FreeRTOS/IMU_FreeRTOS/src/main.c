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
//#include <stdarg.h>
#include "UART_Comm.h"
#include "IMU.h"

#define TASK_IMU_STACK_SIZE				(2048/sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_PRIORITY			(configTIMER_TASK_PRIORITY - 1)
#define TASK_MONITOR_STACK_SIZE         (2048/sizeof(portSTACK_TYPE))
#define TASK_MONITOR_STACK_PRIORITY     (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE             (1024/sizeof(portSTACK_TYPE))
#define TASK_LED_STACK_PRIORITY         (tskIDLE_PRIORITY+1)

#define DELAY_1S							(1000/portTICK_RATE_MS)
#define DELAY_500MS							(500/portTICK_RATE_MS)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
void vTimerCallback(void *pvParameters);

xTimerHandle xTimer;
xSemaphoreHandle xSem;
/*xSemaphoreHandle xMux;*/

/*
void printf_mux( const char * format, ... ){
	xSemaphoreTake(xMux, portMAX_DELAY);
	va_list(args);
	va_start(args, format);
	vprintf(format, args);
	printf("\n");
	va_end(args);
	xSemaphoreGive(xMux);
}*/

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;) {
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

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
}

/**
 * \brief Configure the console UART.
 */
/*
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.stopbits = CONF_UART_STOP_BITS,
		.paritytype = CONF_UART_PARITY,
	};

	/ * Configure console UART. * /
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}*/

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;) {
		printf_mux("--- Number of Tasks: %u", (unsigned int)uxTaskGetNumberOfTasks());
		printf_mux("Name\t\tState\tPrior\tStack\tNum");
		vTaskList((signed portCHAR *)szList);
		printf_mux(szList);
		vTaskDelay(DELAY_1S);
	}
}

static void task_led0(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
		printf_mux("\tLed0!");
		LED_Toggle(LED0_GPIO);
		vTaskDelay(DELAY_500MS);
	}
}

static void task_led1(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
		printf_mux("\tLed1!");
		LED_Toggle(LED1_GPIO);
		xSemaphoreTake(xSem, portMAX_DELAY);
	}
}

void vTimerCallback(void *pvParameters){
	xSemaphoreGive(xSem);
}

int main (void)
{
	/* Initialize Board and Clock */
	sysclk_init();
	board_init();

	/* Initialize the console uart */
	configure_console();
	printf("Console OK!\n");
	
	config_lcd();
	ili9225_set_foreground_color(COLOR_BLACK);
	ili9225_draw_string(10,10, (uint8_t *)"IMU FreeRTOS");
	
	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
	TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
	}
	
	if (xTaskCreate(task_led0, "Led0", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if (xTaskCreate(task_led1, "Led1", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	if (xTaskCreate(IMUTask, "IMU Task", TASK_IMU_STACK_SIZE, NULL,
	TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
	}
	
	xTimer = xTimerCreate("Timer", DELAY_1S , pdTRUE, NULL, vTimerCallback);
	
	xTimerStart(xTimer, 0);
	
	vSemaphoreCreateBinary(xSem);
	
	/*xMux = xSemaphoreCreateMutex();*/
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	return 0;
}
