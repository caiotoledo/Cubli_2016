/*
 * UART_Comm.h
 *
 * Created: 03/04/2016 22:59:07
 *  Author: Caio
 */ 


#ifndef UART_COMM_H_
#define UART_COMM_H_

#include <asf.h>
#include "IMU.h"

xQueueHandle xQueueUARTAcel[NUM_AXIS];
xQueueHandle xQueueUARTGyro[NUM_AXIS];

xTaskHandle xTXHandler;
xTimerHandle xTimerTX;

void printf_mux( const char * format, ... );
void configure_console(void);
void UARTTXTask (void *pvParameters);
void UARTRXTask(void *pvParameters);

static void vTimerTX(void *pvParameters);
static uint8_t isNumbers(char *str);
static void checkMessage(char *buf);

#endif /* UART_COMM_H_ */