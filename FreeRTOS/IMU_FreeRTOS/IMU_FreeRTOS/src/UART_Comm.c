/*
 * UART_Comm.c
 *
 * Created: 03/04/2016 22:58:52
 *  Author: Caio
 */ 

#include "UART_Comm.h"
#include <asf.h>
#include <stdarg.h>
#include <string.h>

#define UART_WAIT	portMAX_DELAY

xSemaphoreHandle xMux;
freertos_uart_if freertos_uart;

uint8_t recBuf[50];
uint32_t sizeRecBuf = sizeof(recBuf);

void printf_mux( const char * format, ... ){
	char buffer[128];
	uint32_t n = 0;
	va_list(args);
	va_start(args, format);
	n = vsprintf(buffer, format, args);
	freertos_uart_write_packet(freertos_uart, buffer, n, UART_WAIT);
	va_end(args);
}

/**
 * \brief Configure the console UART.
 */
void configure_console(void){
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.stopbits = CONF_UART_STOP_BITS,
		.paritytype = CONF_UART_PARITY,
	};
	
	freertos_peripheral_options_t driver_options = {
		recBuf,
		sizeRecBuf,
		configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY,
		UART_RS232,
		(USE_TX_ACCESS_MUTEX | USE_RX_ACCESS_MUTEX)
	};
	
	sam_uart_opt_t uart_settings;
	uart_settings.ul_mck = sysclk_get_peripheral_hz();
	uart_settings.ul_baudrate = CONF_UART_BAUDRATE;
	uart_settings.ul_mode = UART_MR_PAR_NO;

	/* Configure console UART. */
	/*sysclk_enable_peripheral_clock(CONSOLE_UART_ID);*/
	stdio_serial_init(CONF_UART, &uart_serial_options);
	
	freertos_uart = freertos_uart_serial_init(UART0, 
											&uart_settings, 
											&driver_options);
	configASSERT(freertos_uart);
}

/**
 * \UART Task
 */
void UARTTXTask (void *pvParameters){
	UNUSED(pvParameters);
	
	float uart_acel[3];
	float uart_gyro[3];
	char uartBuf[150] = {0};
	uint8_t i = 0;
	signed portBASE_TYPE statusQueue;
	
	status_code_t result = STATUS_ERR_TIMEOUT;
	
	for (;;){
		xSemaphoreTake(xseIMUValues, portMAX_DELAY);
		
		memset(uart_acel, 0, sizeof(uart_acel));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueAcel[i], &(uart_acel[i]),UART_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		memset(uart_gyro, 0, sizeof(uart_gyro));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueGyro[i], &(uart_gyro[i]),UART_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		sprintf(uartBuf, "Acel:\tX = %0.3f\tY = %0.3f\tZ = %0.3f\nGyro:\tX = %0.3f\tY = %0.3f\tZ = %0.3f\n", 
				uart_acel[0], uart_acel[1], uart_acel[2],
				uart_gyro[0], uart_gyro[1], uart_gyro[2]);
		result = freertos_uart_write_packet(freertos_uart, uartBuf, strlen((char *)uartBuf), UART_WAIT);
		if (result != STATUS_OK) LED_Toggle(LED2_GPIO);
	}
}