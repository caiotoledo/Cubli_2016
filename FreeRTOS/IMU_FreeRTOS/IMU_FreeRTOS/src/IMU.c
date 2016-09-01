/*
 * IMU.c
 *
 * Created: 03/04/2016 16:45:34
 *  Author: Caio
 */ 

#include <asf.h>

#include "IMU.h"
#include "HAL/HAL_UART.h"
#include "LCD.h"
#include "Filter/KalmanFilter.h"
#include "Filter/ComplementaryFilter.h"
#include <string.h>

#define xQueueOverwrite(xQueue,pvItemToQueue)	xQueueReset(xQueue);xQueueSendToFront(xQueue,pvItemToQueue,(0))

freertos_twi_if freertos_twi;
char buffer[50];

volatile xQueueHandle xQueueAcel[NUM_AXIS];
volatile xQueueHandle xQueueAngle[NUM_AXIS];
volatile xQueueHandle xQueueGyro[NUM_AXIS];

xSemaphoreHandle xseIMU;
xSemaphoreHandle xSemIMUInt;
xTimerHandle xTimerIMU;

//uint8_t configBWRate = BWrate6_25Hz;
uint8_t configBWRate = BWrate100Hz;
uint32_t lastTickCounter = 0;
double dt = (((double)TWI_TASK_DELAY)/ ((double)configTICK_RATE_HZ));

uint32_t timerIMU = (TWI_TASK_DELAY);

/* ANGLE MEASUREMENT VARIABLES */
static KalmanConst kalmanC;
double acel[3] = { 0 };
double gyro[3] = { 0 };
double anglePure = 0;
double angleComplFilter = 0;
double angleKalman = 0;

static void vTimerIMU(void *pvParameters){
	xSemaphoreGive(xSemIMUInt);
}

void intpin_handler(uint32_t id, uint32_t mask){
	//dt = (double)(g_tickCounter - lastTickCounter)/configTICK_RATE_HZ;
	lastTickCounter = g_tickCounter;
	xSemaphoreGiveFromISR(xSemIMUInt, NULL);
}

void cTaskSample(commVar val){
	float value = val.value;
	
	switch (val.type)
	{
		case cSet:
			if (value > 0){
				timerIMU = (uint32_t)value;
				xTimerChangePeriod(xTimerIMU, timerIMU, 0);
				printf_mux("IMUSample = %u ms\r\n", (timerIMU));
			} else {
				printf_mux("IMUSample Value Error [%f]\r\n", value);
			}
		break;
		case cGet:
			printf_mux("IMUSample = %u ms\r\n", (timerIMU));
		break;
	}
}

static void initializeIMUVariables(){
	memset(acel, 0, sizeof(acel));
	memset(gyro, 0, sizeof(gyro));
	anglePure = 0;
	angleComplFilter = initComplFilter(ADXL_Low);
	angleKalman = angleComplFilter;
	
	//Init Kalman Constants:
	kalmanC.angleInit	=	angleKalman;
	kalmanC.bias		=	0;
	kalmanC.P[0][0]		=	0;
	kalmanC.P[0][1]		=	0;
	kalmanC.P[1][0]		=	0;
	kalmanC.P[1][1]		=	0;
	
	initKalman(&kalmanC);
}

void IMUTask(void *pvParameters){
	UNUSED(pvParameters);
	status_code_t status;
	
	vSemaphoreCreateBinary(xseIMUValues);
	configASSERT(xseIMUValues);
	xSemaphoreTake(xseIMUValues, 0);
	
	vSemaphoreCreateBinary(xSemIMUInt);
	configASSERT(xSemIMUInt);
	xSemaphoreTake(xSemIMUInt, 0);
	
#ifndef INT_PIN
	//Timer Task:
	xTimerIMU = xTimerCreate("TimerIMU", TWI_TASK_DELAY , pdTRUE, NULL, vTimerIMU);
	xTimerStart(xTimerIMU, 0);
#endif
	
	status = configIMU();
	lastTickCounter = g_tickCounter;
	if (status != STATUS_OK){
		printf_mux("Error IMU!");
		LED_On(LED2_GPIO);
		vTaskDelete(NULL);
	}
	
	uint8_t i = 0;
	
	//Start Kalman Constants with standard values:
	kalmanC.Qangle		=	0.001;
	kalmanC.Qbias		=	0.003;
	kalmanC.Rmeasure	=	0.03;
	initializeIMUVariables();
	
	for (;;){
		xSemaphoreTake(xSemIMUInt, portMAX_DELAY);
		
		memset(acel, 0, sizeof(acel));
		getAllAcelValue(ADXL_Low, acel);
		for (i = 0; i < NUM_AXIS; i++){
			xQueueOverwrite(xQueueAcel[i], (void * ) &acel[i]);
		}
		
		memset(gyro, 0, sizeof(gyro));
		getAllGyroValue(ITG_Low, gyro);
		for (i = 0; i < NUM_AXIS; i++){
			xQueueOverwrite(xQueueGyro[i], (void * ) &gyro[i]);
		}
		
		anglePure = getPureAngle(acel);
		xQueueOverwrite(xQueueAngle[0], (void * ) &anglePure);
		getComplFilterAngle(&angleComplFilter, acel, gyro, dt);
		xQueueOverwrite(xQueueAngle[1], (void * ) &angleComplFilter);
		angleKalman = getKalmanAngle(anglePure, gyro[Axis_Z], dt);
		xQueueOverwrite(xQueueAngle[2], (void * ) &angleKalman);
		
		//Give Semaphore to UART Transfer:
		xSemaphoreGive(xseIMUValues);
	}
}

void cKalQAngle(commVar val){
	float *kalmanVar = NULL;
	float value = val.value;
	//Variables to make it easier to copy the same function:
	const char *str = "QAngle";
	kalmanVar = &kalmanC.Qangle;
	
	switch (val.type)
	{
		case cSet:
			if (value > 0){
				(*kalmanVar) = (uint32_t)value;
				printf_mux("%s = %0.5f\r\n",str ,(*kalmanVar));
			} else {
				printf_mux("%s Value Error [%f]\r\n", str, value);
			}
		break;
		case cGet:
			printf_mux("%s = %0.5f\r\n", str, (*kalmanVar));
		break;
	}
}

void cKalQBias(commVar val){
	float *kalmanVar = NULL;
	float value = val.value;
	const char *str = "QBias";
	kalmanVar = &kalmanC.Qbias;
	
	switch (val.type)
	{
		case cSet:
		if (value > 0){
			(*kalmanVar) = (uint32_t)value;
			printf_mux("%s = %0.5f\r\n",str ,(*kalmanVar));
			} else {
			printf_mux("%s Value Error [%f]\r\n", str, value);
		}
		break;
		case cGet:
		printf_mux("%s = %0.5f\r\n", str, (*kalmanVar));
		break;
	}
}

void cKalRMeasure(commVar val){
	float *kalmanVar = NULL;
	float value = val.value;
	const char *str = "RMeasure";
	kalmanVar = &kalmanC.Rmeasure;
	
	switch (val.type)
	{
		case cSet:
		if (value > 0){
			(*kalmanVar) = (uint32_t)value;
			printf_mux("%s = %0.5f\r\n",str ,(*kalmanVar));
			} else {
			printf_mux("%s Value Error [%f]\r\n", str, value);
		}
		break;
		case cGet:
		printf_mux("%s = %0.5f\r\n", str, (*kalmanVar));
		break;
	}
}