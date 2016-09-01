/*
 * UART_Comm.h
 *
 * Created: 03/04/2016 22:59:07
 *  Author: Caio
 */ 


#ifndef UART_COMM_H_
#define UART_COMM_H_

#include <asf.h>
#include "HAL/HAL_IMU.h"
#include "Commands.h"

xQueueHandle xQueueUARTAcel[NUM_AXIS];
xQueueHandle xQueueUARTGyro[NUM_AXIS];

xTaskHandle xTXHandler;
xTimerHandle xTimerTX;

void cStartSample(commVar val);
void cTotalTimeTest(commVar val);

void UARTTXTask (void *pvParameters);
void UARTRXTask(void *pvParameters);
void setTimerSample(float value);
void startSample(uint32_t val);

#endif /* UART_COMM_H_ */