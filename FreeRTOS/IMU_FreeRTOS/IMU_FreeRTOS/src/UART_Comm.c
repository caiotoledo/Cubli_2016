/*
 * UART_Comm.c
 *
 * Created: 03/04/2016 22:58:52
 *  Author: Caio
 */ 

#include "UART_Comm.h"
#include <asf.h>
#include <stdarg.h>

xSemaphoreHandle xMux;

void printf_mux( const char * format, ... ){
	xSemaphoreTake(xMux, 1000);
	va_list(args);
	va_start(args, format);
	vprintf(format, args);
	printf("\n");
	va_end(args);
	xSemaphoreGive(xMux);
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

	/* Configure console UART. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
	
	xMux = xSemaphoreCreateMutex();
}