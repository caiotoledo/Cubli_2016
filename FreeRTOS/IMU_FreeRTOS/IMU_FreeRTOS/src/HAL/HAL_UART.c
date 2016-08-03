/*
 * HAL_UART.c
 *
 * Created: 02/08/2016 21:09:18
 *  Author: Caio
 */ 

#include "HAL_UART.h"
#include <stdarg.h>
#include <string.h>
#include <ctype.h>

#define UART_WAIT	portMAX_DELAY

freertos_uart_if freertos_uart;

uint8_t recBuf[50];
uint32_t sizeRecBuf = sizeof(recBuf);

status_code_t send_uart(char * buf, size_t len){
	status_code_t status = ERR_ABORTED;
	
	status = freertos_uart_write_packet(freertos_uart, buf, len, UART_WAIT);
	
	return status;
}

uint32_t read_uart(char *buf, uint32_t len){
	uint32_t size = 0;
	
	size = freertos_uart_serial_read_packet(freertos_uart,
											buf,
											len,
											UART_WAIT);

	return size;
}

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
	stdio_base = (void *)CONF_UART;
	ptr_put = (int (*)(void volatile*,char))&usart_serial_putchar;
	ptr_get = (void (*)(void volatile*,char*))&usart_serial_getchar;
	setbuf(stdout, NULL);
	setbuf(stdin, NULL);
	
	freertos_uart = freertos_uart_serial_init(UART0, 
											&uart_settings, 
											&driver_options);
	configASSERT(freertos_uart);
}