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

Bool enableTX = false;

static void vTimerTX(void *pvParameters){
	LED_Off(LED1_GPIO);
	//vTaskSuspend(xTXHandler);
	enableTX = false;
	vTaskResume(xLCDHandler);
	//printf_mux("STOP\r");
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
		vTaskSuspend(xLCDHandler);
		LED_On(LED1_GPIO);
		xTimerChangePeriod(xTimerTX, timer, portMAX_DELAY);
		enableTX = true;
		//vTaskResume(xTXHandler);
	}
}

/**
 * \UART TX Task
 */
void UARTTXTask (void *pvParameters){
	UNUSED(pvParameters);
		
	double uart_acel[3];
	double uart_angle[3];
	double uart_gyro[3];
	//char uartBuf[100] = {0};
	uint8_t i = 0;
	signed portBASE_TYPE statusQueue;
	
	status_code_t result = STATUS_OK;
	
	for (;;){
		if (!enableTX) {
			/*Identify that the task did stop by vTimerTX.
			  Sends a message to notify MATLAB: */
			printf_mux("STOP\r");
		}
		xSemaphoreTake(xseIMUValues, portMAX_DELAY);
		
		memset(uart_acel, 0, sizeof(uart_acel));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueAcel[i], &(uart_acel[i]),portMAX_DELAY);
			if (statusQueue != pdPASS) LED_Toggle(LED2_GPIO);
		}
		
		memset(uart_angle, 0, sizeof(uart_angle));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueAngle[i], &(uart_angle[i]),portMAX_DELAY);
			if (statusQueue != pdPASS) LED_Toggle(LED2_GPIO);
		}
		
		memset(uart_gyro, 0, sizeof(uart_gyro));
		for (i = 0; i < NUM_AXIS; i++){
			statusQueue = xQueuePeek(xQueueGyro[i], &(uart_gyro[i]),portMAX_DELAY);
			if (statusQueue != pdPASS) LED_Toggle(LED2_GPIO);
		}
		
		/*sprintf(uartBuf, "%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;\r\n", 
				uart_acel[0], uart_acel[1], uart_acel[2],
				uart_gyro[0], uart_gyro[1], uart_gyro[2],
				uart_angle[0], uart_angle[1], uart_angle[2],
				getAngleEncoder(true));
		result = send_uart(uartBuf, strlen((char *)uartBuf));*/
		result = printf_mux("%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;%0.4f;\r\n",
							uart_acel[0], uart_acel[1], uart_acel[2],
							uart_gyro[0], uart_gyro[1], uart_gyro[2],
							uart_angle[0], uart_angle[1], uart_angle[2],
							getAngleEncoder(true));
		if (result != STATUS_OK) LED_Toggle(LED2_GPIO);
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
		size = read_uart(&receive, sizeof(receive));
		if (size == sizeof(receive)){
			//Enter is the end of message:
			if (receive == 13){
				//checkMessage(buf_msg);
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