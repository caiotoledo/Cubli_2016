/*
 * IMU.c
 *
 * Created: 03/04/2016 16:45:34
 *  Author: Caio
 */ 

#include <asf.h>

#include "IMU.h"
#include "UART_Comm.h"
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

static KalmanConst kalmanC;

static void vTimerIMU(void *pvParameters){
	xSemaphoreGive(xSemIMUInt);
}

void intpin_handler(uint32_t id, uint32_t mask){
	//dt = (double)(g_tickCounter - lastTickCounter)/configTICK_RATE_HZ;
	lastTickCounter = g_tickCounter;
	xSemaphoreGiveFromISR(xSemIMUInt, NULL);
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
	
	double acel[3];
	double gyro[3];
	double anglePure = 0;
	double angleComplFilter = initComplFilter(ADXL_Low);
	double angleKalman = angleComplFilter;
	uint8_t i = 0;
	
	//Init Kalman Constants:
	kalmanC.angleInit	=	angleKalman;
	kalmanC.bias		=	0;
	kalmanC.Qangle		=	0.001;
	kalmanC.Qbias		=	0.003;
	kalmanC.Rmeasure	=	0.03;
	kalmanC.P[0][0]		=	0;
	kalmanC.P[0][1]		=	0;
	kalmanC.P[1][0]		=	0;
	kalmanC.P[1][1]		=	0;
	
	initKalman(&kalmanC);
	
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