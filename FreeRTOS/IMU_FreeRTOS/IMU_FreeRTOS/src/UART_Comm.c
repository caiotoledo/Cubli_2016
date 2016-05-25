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
#include <ctype.h>

#define UART_WAIT	portMAX_DELAY

xSemaphoreHandle xMux;
xSemaphoreHandle xUARTSend;
freertos_uart_if freertos_uart;

uint8_t recBuf[50];
uint32_t sizeRecBuf = sizeof(recBuf);

uint32_t timer = (1000/portTICK_RATE_MS);

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
	
	//Initialize Semaphore
	vSemaphoreCreateBinary(xUARTSend);
	configASSERT(xUARTSend);
	xSemaphoreTake(xUARTSend, 0);
}

/**
 * \UART TX Task
 */
void UARTTXTask (void *pvParameters){
	UNUSED(pvParameters);
	
	double uart_acel[3];
	double uart_angle[3];
	double uart_gyro[3];
	char uartBuf[250] = {0};
	uint8_t i = 0;
	signed portBASE_TYPE statusQueue;
	
	status_code_t result = STATUS_OK;
	
	for (;;){
		xSemaphoreTake(xseIMUValues, portMAX_DELAY);
		
		memset(uart_acel, 0, sizeof(uart_acel));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueAcel[i], &(uart_acel[i]),UART_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		memset(uart_angle, 0, sizeof(uart_angle));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueAngle[i], &(uart_angle[i]),UART_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		memset(uart_gyro, 0, sizeof(uart_gyro));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueGyro[i], &(uart_gyro[i]),UART_WAIT);
			if (statusQueue != pdPASS) vTaskDelete(NULL);
		}
		
		/*sprintf(uartBuf, "Acel:\tX = %0.3f\tY = %0.3f\tZ = %0.3f\r\nGyro:\tX = %0.3f\tY = %0.3f\tZ = %0.3f\r\nAngle:\tP = %0.3f\tC = %0.3f\tK = %0.3f\r\n", 
				uart_acel[0], uart_acel[1], uart_acel[2],
				uart_gyro[0], uart_gyro[1], uart_gyro[2],
				uart_angle[0], uart_angle[1], uart_angle[2]);*/
		/*sprintf(uartBuf, "Acel:\tX = %0.3f\tY = %0.3f\tZ = %0.3f\r\nAngle:\tP = %0.3f\tC = %0.3f\tK = %0.3f\r\n",
				uart_acel[0], uart_acel[1], uart_acel[2],
				uart_angle[0], uart_angle[1], uart_angle[2]);*/
		sprintf(uartBuf, "%0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f %0.4f\r\n", 
				uart_acel[0], uart_acel[1], uart_acel[2],
				uart_gyro[0], uart_gyro[1], uart_gyro[2],
				uart_angle[0], uart_angle[1], uart_angle[2]);
		result = freertos_uart_write_packet(freertos_uart, uartBuf, strlen((char *)uartBuf), UART_WAIT);
		if (result != STATUS_OK) LED_Toggle(LED2_GPIO);
	}
}

static void vTimerTX(void *pvParameters){
	vTaskSuspend(xTXHandler);
}

static uint8_t isNumbers(char *str){
	while(*str){
		if ( (!isdigit(*str)) && ( (*str) != '.' ) ){
			return false;
		}
		str++;
	}
	return true;
}

static void checkMessage(char *buf){
	if (!strcmp(buf,"go")) {
		if (timer){
			xTimerChangePeriod(xTimerTX, timer, 0);
			vTaskResume(xTXHandler);
		} else {
			printf_mux("Timer Error [Value %u]\r\n", timer);
		}
	} else if (isNumbers(buf)) {
		timer = atof(buf)*(1000/portTICK_RATE_MS);
		printf_mux("Timer = %0.3f seconds\r\n", ((float)timer/1000));
	} else if(!strcmp(buf,"t")){
		printf_mux("Timer = %0.3f seconds\r\n", ((float)timer/1000));
	} else {
		printf_mux("Wrong Command\r\n");
	}
}

// Task to receive commands via Serial:
void UARTRXTask(void *pvParameters){
	UNUSED(pvParameters);
	
	uint8_t receive;
	char buf_msg[50] = {0};
	uint32_t countChar = 0;
	
	uint32_t size = 0;
	
	xTimerTX = xTimerCreate("TimerIMU", (1000/portTICK_RATE_MS) , pdFALSE, NULL, vTimerTX);
	
	for (;;){
		//Receive only one character at a time
		size = freertos_uart_serial_read_packet(freertos_uart, &receive, sizeof(receive), portMAX_DELAY);
		if (size == sizeof(receive)){
			//Enter is the end of message:
			if (receive == 13){
				checkMessage(buf_msg);
				memset(buf_msg, 0, sizeof(buf_msg));
				countChar = 0;
			}
			//Keep receiving characters:
			else {
				buf_msg[countChar++] = receive;
				buf_msg[countChar] = '\0';
			}
		}
	}
}