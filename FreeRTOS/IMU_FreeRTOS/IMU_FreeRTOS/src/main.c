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
#include "UART_Comm.h"
#include "IMU.h"
#include "LCD.h"

//#define TASK_MONITOR

//#define TASK_LCD_STACK_SIZE				(2048/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_SIZE				(1024/sizeof(portSTACK_TYPE))
#define TASK_LCD_STACK_PRIORITY			(tskIDLE_PRIORITY+2)
//#define TASK_IMU_STACK_SIZE				(4096/sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_SIZE				(2048/sizeof(portSTACK_TYPE))
#define TASK_IMU_STACK_PRIORITY			(configTIMER_TASK_PRIORITY - 1)
#define TASK_UART_STACK_SIZE			(2048/sizeof(portSTACK_TYPE))
#define TASK_UART_STACK_PRIORITY		(tskIDLE_PRIORITY+3)
#define TASK_MONITOR_STACK_SIZE         (512)	//Bytes
#define TASK_MONITOR_STACK_PRIORITY     (tskIDLE_PRIORITY)
#define TASK_LED_STACK_SIZE             configMINIMAL_STACK_SIZE
#define TASK_LED_STACK_PRIORITY         (tskIDLE_PRIORITY+1)

#define DELAY_5S						(5000/portTICK_RATE_MS)
#define DELAY_1S						(1000/portTICK_RATE_MS)
#define DELAY_500MS						(500/portTICK_RATE_MS)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
		signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);

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
		LED_On(LED2_GPIO);
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

void vApplicationMallocFailedHook(void)
{
	/* vApplicationMallocFailedHook() will only be called if
	configUSE_MALLOC_FAILED_HOOK is set to 1 in FreeRTOSConfig.h.  It is a hook
	function that will get called if a call to pvPortMalloc() fails.
	pvPortMalloc() is called internally by the kernel whenever a task, queue,
	timer or semaphore is created.  It is also called by various parts of the
	demo application.  If heap_1.c or heap_2.c are used, then the size of the
	heap available to pvPortMalloc() is defined by configTOTAL_HEAP_SIZE in
	FreeRTOSConfig.h, and the xPortGetFreeHeapSize() API function can be used
	to query the size of free heap space that remains (although it does not
	provide information on how the remaining heap might be fragmented). */
	taskDISABLE_INTERRUPTS();
	for (;;) {
	}
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	static uint32_t freeHeap;
	UNUSED(pvParameters);

	for (;;) {
		vTaskDelay(DELAY_5S);
		printf_mux("--- Number of Tasks: %u\n", (unsigned int)uxTaskGetNumberOfTasks());
		printf_mux("Name\t\tState\tPrior\tStack\tNum\n");
		vTaskList((signed portCHAR *)szList);
		printf_mux(szList);
		freeHeap = (uint32_t)xPortGetFreeHeapSize();
		printf_mux("Free Heap: %u\n", freeHeap);
	}
}

static void task_led0(void *pvParameters)
{
	UNUSED(pvParameters);
	for (;;) {
		LED_Toggle(LED0_GPIO);
		vTaskDelay(DELAY_500MS);
	}
}

int main (void)
{
	/* Initialize Board and Clock */
	sysclk_init();
	NVIC_SetPriorityGrouping(0);
	board_init();

	/* Initialize the console uart */
	configure_console();
	printf("Console OK!\n");
	
	config_lcd();
	
#ifdef TASK_MONITOR
	/* Create task to monitor processor activity */
	if (xTaskCreate(task_monitor, "Monitor", TASK_MONITOR_STACK_SIZE, NULL,
	TASK_MONITOR_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create Monitor task\r\n");
		LED_On(LED2_GPIO);
	}
#endif
	
	if (xTaskCreate(task_led0, "Led0", TASK_LED_STACK_SIZE, NULL,
	TASK_LED_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test led task\r\n");
		LED_On(LED2_GPIO);
	}
	
	if (xTaskCreate(IMUTask, "IMU_T", TASK_IMU_STACK_SIZE, NULL,
	TASK_IMU_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test IMUTask\r\n");
		LED_On(LED2_GPIO);
	}

	if (xTaskCreate(LCDTask, "LCD_T", TASK_LCD_STACK_SIZE, NULL,
	TASK_LCD_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test LCDTask\r\n");
		LED_On(LED2_GPIO);
	}
	
	if (xTaskCreate(UARTTXTask, "TX_Task", TASK_UART_STACK_SIZE, NULL,
	TASK_UART_STACK_PRIORITY, NULL) != pdPASS) {
		printf("Failed to create test TX_Task\r\n");
		LED_On(LED2_GPIO);
	}
	
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient memory to create the idle task. */
	LED_On(LED2_GPIO);
	return 0;
}
