/*
 * UART_Comm.c
 *
 * Created: 03/04/2016 22:58:52
 *  Author: Caio
 */ 

#include "UART_Comm.h"
#include "HAL/HAL_UART.h"
#include "Commands.h"
#include "IMU.h"
#include "LCD.h"
#include "HAL/HAL_Encoder.h"
#include <string.h>

uint32_t timer = (1000/portTICK_RATE_MS);

uint8_t enableTX = 0;

static void vTimerTX(void *pvParameters){
	LED_Off(LED1_GPIO);
	enableTX = 1;
	if (xLCDHandler) {
		vTaskResume(xLCDHandler);
	}
}

void cTotalTimeTest(commVar val){
	float value = val.value;
	
	switch (val.type)
	{
		case cSet:
		if (value > 0){
			timer = value*(1000/portTICK_RATE_MS);
			printf_mux("Timer = %0.3f seconds\r\n", ((float)timer/1000));
			} else {
			printf_mux("Timer Value Error [%f]\r\n", value);
		}
		break;
		case cGet:
		printf_mux("Timer = %0.3f seconds\r\n", ((float)timer/1000));
		break;
	}
}

void cStartSample(commVar val){
	if (timer){
		if (xLCDHandler) {
			vTaskSuspend(xLCDHandler);
		}
				
		LED_On(LED1_GPIO);
		xTimerChangePeriod(xTimerTX, timer, portMAX_DELAY);
		enableTX = 2;
		
		/* Give Semaphore to UART TX Transfer the last IMU Value: */
		xSemaphoreGive(xseIMUValues);
	}
}

/**
 * \UART TX Task
 */
void UARTTXTask (void *pvParameters){
	UNUSED(pvParameters);
		
	double uart_acel[2][3];
	double uart_angle[3];
	double uart_gyro[2][3];
	uint8_t i = 0;
	signed portBASE_TYPE statusQueue;
	
	status_code_t result = STATUS_OK;
	
	for (;;){
		if (enableTX == 1) {
			/*Identify that the task did stop by vTimerTX.
			  Sends a message to notify MATLAB:*/
			xSemaphoreTake(xseIMUValues, portMAX_DELAY);
			enableTX = 0;
			printf_mux("STOP\r\n");
		}
		
		xSemaphoreTake(xseIMUValues, portMAX_DELAY);
		
		memset(uart_acel, 0, sizeof(uart_acel));
		for (i = 0; i < NUM_AXIS; i++){
			/* Get Acel Values from Low IMU: */
			statusQueue = xQueuePeek(xQueueAcel[0][i], &(uart_acel[0][i]),portMAX_DELAY);
			if (statusQueue != pdPASS) LED_Toggle(LED2_GPIO);
			
			/* Get Acel Values from High IMU: */
			statusQueue = xQueuePeek(xQueueAcel[1][i], &(uart_acel[1][i]),portMAX_DELAY);
			if (statusQueue != pdPASS) LED_Toggle(LED2_GPIO);
		}
		
		memset(uart_angle, 0, sizeof(uart_angle));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueAngle[i], &(uart_angle[i]),portMAX_DELAY);
			if (statusQueue != pdPASS) LED_Toggle(LED2_GPIO);
		}
		
		memset(uart_gyro, 0, sizeof(uart_gyro));
		for (i = 0; i < NUM_AXIS; i++){
			/* Get Acel Values from Low IMU: */
			statusQueue = xQueuePeek(xQueueGyro[0][i], &(uart_gyro[0][i]),portMAX_DELAY);
			if (statusQueue != pdPASS) LED_Toggle(LED2_GPIO);
			
			/* Get Acel Values from High IMU: */
			statusQueue = xQueuePeek(xQueueGyro[1][i], &(uart_gyro[1][i]),portMAX_DELAY);
			if (statusQueue != pdPASS) LED_Toggle(LED2_GPIO);
		}
		
		result = printf_mux("%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;\r\n",
							uart_acel[0][0], uart_acel[0][1], uart_acel[0][2],
							uart_acel[1][0], uart_acel[1][1], uart_acel[1][2],
							uart_gyro[0][0], uart_gyro[0][1], uart_gyro[0][2],
							uart_gyro[1][0], uart_gyro[1][1], uart_gyro[1][2],
							uart_angle[0], uart_angle[1], uart_angle[2],
							getAngleEncoder(true));
		if (result != STATUS_OK) LED_Toggle(LED2_GPIO);
	}
}

// Task to receive commands via Serial:
void UARTRXTask(void *pvParameters){
	UNUSED(pvParameters);
	
	char receive;
	char buf_msg[50] = {0};
	uint32_t countChar = 0;
	
	uint32_t size = 0;
	
	xTimerTX = xTimerCreate( (const signed char *) "TimerIMU", (1000/portTICK_RATE_MS) , pdFALSE, NULL, vTimerTX);
	
	for (;;){
		//Receive only one character at a time
		size = read_uart(&receive, sizeof(receive));
		if (size == sizeof(receive)){
			//Enter is the end of message:
			if (receive == 13){
				receiveCMD(buf_msg);
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