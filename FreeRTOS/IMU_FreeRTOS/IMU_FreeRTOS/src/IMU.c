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
#include "UART_Comm.h"
#include "HAL/HAL_Encoder.h"
#include <string.h>

#define xQueueOverwrite(xQueue,pvItemToQueue)	xQueueReset(xQueue);xQueueSendToFront(xQueue,pvItemToQueue,(0))

static double getFinalAngle(double *acel, double *gyro);

static void initIMUQueue(void);
static void initializeIMUVariables(void);

static void cmdHandlerKalman(commVar val, double *Kvalue, const char *valName);
static void cmdHandlerOffset(commVar val, Axis_Op ax, const char *axName);

freertos_twi_if freertos_twi;
char buffer[50];

volatile xQueueHandle xQueueAcel[NUM_AXIS];
volatile xQueueHandle xQueueAngle[NUM_AXIS];
volatile xQueueHandle xQueueGyro[NUM_AXIS];

xSemaphoreHandle xseIMU;
xSemaphoreHandle xSemIMUInt;
xTimerHandle xTimerIMU;

uint32_t lastTickCounter = 0;
double dt = (((double)TWI_TASK_DELAY)/ ((double)configTICK_RATE_HZ));

//Initial Sample Timer
uint32_t timerIMU = (TWI_TASK_DELAY);

/* ANGLE MEASUREMENT VARIABLES */
static KalmanConst kalmanC;
double gAcel[3] = { 0 };
double gGyro[3] = { 0 };
double anglePure = 0;
double angleComplFilter = 0;
double angleKalman = 0;

static void initIMUQueue(void){
	/* Queue FreeRTOS Initialization */
	uint8_t i;
	for (i = 0; i < NUM_AXIS; i++){
		xQueueAcel[i] = xQueueCreate(1, sizeof(double));
		if (xQueueAcel[i] == NULL){
			LED_On(LED2_GPIO);
			while(1);
		}
		xQueueAngle[i] = xQueueCreate(1, sizeof(double));
		if (xQueueAngle[i] == NULL){
			LED_On(LED2_GPIO);
			while(1);
		}
		xQueueGyro[i] = xQueueCreate(1, sizeof(double));
		if (xQueueGyro[i] == NULL){
			LED_On(LED2_GPIO);
			while(1);
		}
	}
}

static void vTimerIMU(void *pvParameters){
	xSemaphoreGive(xSemIMUInt);
}

void intpin_handler(uint32_t id, uint32_t mask){
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
				dt = (((double)timerIMU)/ ((double)configTICK_RATE_HZ));
				xTimerChangePeriod(xTimerIMU, timerIMU, 0);
				printf_mux("IMUSample = %lu ms\r\n", (timerIMU));
			} else {
				printf_mux("IMUSample Value Error [%f]\r\n", value);
			}
		break;
		case cGet:
			printf_mux("IMUSample = %lu ms\r\n", (timerIMU));
		break;
	}
}

static void initializeIMUVariables(void){
	memset(gAcel, 0, sizeof(gAcel));
	memset(gGyro, 0, sizeof(gGyro));
	anglePure = 0;
	angleComplFilter = initComplFilter(IMU_Low);
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

void cResetVariables(commVar val){
	//Stop timer Sample for IMU
	xTimerStop(xTimerIMU, portMAX_DELAY);
	
	//Restart all Variables:
	initializeIMUVariables();
	
	//Reset Encoder:
	//resetCounterEncoder();
	
	//Set Initial Angle Encoder:
	setCounterEncoder(true, angleKalman);
	
	//Start time Sample for IMU
	xTimerStart(xTimerIMU, portMAX_DELAY);
}

void cStartSampleReset(commVar val){	
	//Reset Variables:
	cResetVariables(val);
	
	//Start Serial Task
	cStartSample(val);
}

static double getFinalAngle(double *acel, double *gyro){
	uint32_t statusLow, statusHigh;
	double dAcel[2][NUM_AXIS], dGyro[2][NUM_AXIS];
	double resultAngle = 0;
	uint32_t i;
	
	statusLow = sampleIMU(IMU_Low, &dAcel[0][0], &dGyro[0][0]);
	statusHigh = sampleIMU(IMU_High, &dAcel[1][0], &dGyro[1][0]);
	
	if ( (statusLow | statusHigh) == TWI_SUCCESS ) {
		resultAngle = getPureAngleTwoIMU(&dAcel[IMUADDR_TO_NUM(IMU_Low)][0], &dAcel[IMUADDR_TO_NUM(IMU_High)][0]);
		/* Give the Mean value between two IMU back */
		for (i = 0; i < NUM_AXIS; i++) {
			acel[i] = ( ( dAcel[IMUADDR_TO_NUM(IMU_Low)][i] + dAcel[IMUADDR_TO_NUM(IMU_High)][i] ) / 2 );
			gyro[i] = ( ( dGyro[IMUADDR_TO_NUM(IMU_Low)][i] + dGyro[IMUADDR_TO_NUM(IMU_High)][i] ) / 2 );
		}
	} 
	else if ( statusLow == TWI_SUCCESS ) {
		resultAngle = getPureAngle(&dAcel[IMUADDR_TO_NUM(IMU_Low)][0]);
		for (i = 0; i < NUM_AXIS; i++) {
			acel[i] = dAcel[IMUADDR_TO_NUM(IMU_Low)][i];
			gyro[i] = dGyro[IMUADDR_TO_NUM(IMU_Low)][i];
		}
	}
	else if ( statusHigh == TWI_SUCCESS ) {
		resultAngle = getPureAngle(&dAcel[IMUADDR_TO_NUM(IMU_High)][0]);
		for (i = 0; i < NUM_AXIS; i++) {
			acel[i] = dAcel[IMUADDR_TO_NUM(IMU_High)][i];
			gyro[i] = dGyro[IMUADDR_TO_NUM(IMU_High)][i];
		}
	}

	return resultAngle;
}

void IMUTask(void *pvParameters){
	UNUSED(pvParameters);
	status_code_t status;
	
	//Starting Queues:
	initIMUQueue();
	
	vSemaphoreCreateBinary(xseIMUValues);
	configASSERT(xseIMUValues);
	xSemaphoreTake(xseIMUValues, 0);
	
	vSemaphoreCreateBinary(xSemIMUInt);
	configASSERT(xSemIMUInt);
	xSemaphoreTake(xSemIMUInt, 0);
	
#ifndef INT_PIN
	//Timer Task:
	xTimerIMU = xTimerCreate( (const signed char *) "TimerIMU", TWI_TASK_DELAY , pdTRUE, NULL, vTimerIMU);
	xTimerStart(xTimerIMU, 0);
#endif
	
	/*
	*	PA3 - Data
	*	PA4 - Clock
	*/
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
		
		memset(gAcel, 0, sizeof(gAcel));
		memset(gGyro, 0, sizeof(gGyro));
		anglePure = getFinalAngle(gAcel, gGyro);
		
		/* Send to UART Task: */
		for (i = 0; i < NUM_AXIS; i++){
			xQueueOverwrite(xQueueAcel[i], (void * ) &gAcel[i]);
			xQueueOverwrite(xQueueGyro[i], (void * ) &gGyro[i]);
		}
		
		xQueueOverwrite(xQueueAngle[0], (void * ) &anglePure);
		getComplFilterAngle(&angleComplFilter, gAcel, gGyro, dt);
		xQueueOverwrite(xQueueAngle[1], (void * ) &angleComplFilter);
		angleKalman = getKalmanAngle(anglePure, gGyro[Axis_Z], dt);
		xQueueOverwrite(xQueueAngle[2], (void * ) &angleKalman);
		
		//Check if TX is actived:
		if (enableTX) {
			//Give Semaphore to UART Transfer:
			xSemaphoreGive(xseIMUValues);
		}
		
	}
}

void cRunCalibrationIMU(commVar val){
	const char *str = "Calibration IMU";
	IMU_Addr_Dev dev = val.device;
	uint32_t result;
	
	/* Disable LCD Task to avoid CPU Load */
	vTaskSuspend(xLCDHandler);
	
	/* Stop IMU Timer to not interrupt Calibration Process */
	xTimerStop(xTimerIMU, 100/portTICK_RATE_MS);
	
	/* Run Calibration Process: */
	LED_On(LED1_GPIO);
	printf_mux("%s Starting...\n", str);
	result = runIMUCalibration(dev, Axis_Z, true);
	if (result == TWI_SUCCESS) {
		printf_mux("Calibration Done! [Device %s]\n", ( (dev == IMU_Low) ? "IMU Low" : "IMU High" ));
		vTaskDelay(5/portTICK_RATE_MS);
		printf_mux("New Accel Offsets: [X = %0.3f] [Y = %0.3f] [Z = %0.3f]\n", getOffsetAccelIMU(dev, Axis_X), getOffsetAccelIMU(dev, Axis_Y), getOffsetAccelIMU(dev, Axis_Z));
		vTaskDelay(5/portTICK_RATE_MS);
		printf_mux("New Gyro Offsets: [X = %0.3f] [Y = %0.3f] [Z = %0.3f]\n", getOffsetGyroIMU(dev, Axis_X), getOffsetGyroIMU(dev, Axis_Y), getOffsetGyroIMU(dev, Axis_Z));
	} 
	else {
		vTaskDelay(5/portTICK_RATE_MS);
		printf_mux("IMU Not Found!\r\n");
	}
	LED_Off(LED1_GPIO);
	
	/* Starts again IMU Task */
	xTimerStart(xTimerIMU, 100/portTICK_RATE_MS);
	
	/* Enable LCD Task */
	vTaskResume(xLCDHandler);
}

void cAlphaComplFilter(commVar val){
	const char *str = "Alpha Compl. Filter";	
	
	switch (val.type){
		case cSet:
			if (setAlpha(val.value)) {
				printf_mux("%s = %0.5f\r\n", str, val.value);
			} else {
				printf_mux("%s Value Error [%f]\r\n", str, val.value);
			}
		break;
		case cGet:
			printf_mux("%s = %0.5f\r\n", str, getAlpha());
		break;
	}
}

void cOffsetAccelX(commVar val){
	cmdHandlerOffset(val, Axis_X, "AccelOffsetX");
}

void cOffsetAccelY(commVar val){
	cmdHandlerOffset(val, Axis_Y, "AccelOffsetY");
}

void cOffsetAccelZ(commVar val){
	cmdHandlerOffset(val, Axis_Z, "AccelOffsetZ");
}

void cOffsetGyroX(commVar val){
	cmdHandlerOffset(val, Axis_X, "GyroOffsetX");
}

void cOffsetGyroY(commVar val){
	cmdHandlerOffset(val, Axis_Y, "GyroOffsetY");
}

void cOffsetGyroZ(commVar val){
	cmdHandlerOffset(val, Axis_Z, "GyroOffsetZ");
}

/* Threat all commands about Offset Gyro and Accel (both set and get) */
static void cmdHandlerOffset(commVar val, Axis_Op ax, const char *axName){
	Bool ret;
	IMU_Addr_Dev dev = val.device;
	float offset = val.value;
	Bool offsetType = true;
	
	if (strstr(axName,"Accel")) {
		offsetType = true;
	} else if (strstr(axName,"Gyro")) {
		offsetType = false;
	} else {
		printf_mux("Wrong use cmdHandlerOffset\r\n");
		return;
	}
	
	switch (val.type)
	{
		case cSet:
			if (offsetType) {
				ret = setOffsetAccelIMU(dev,ax,offset);
			} else {
				ret = setOffsetGyroIMU(dev,ax,offset);
			}
			if (ret){
				printf_mux("%s = %0.5f\r\n",axName , offset);
			} else {
				printf_mux("%s Value Error [%f]\r\n", axName, offset);
			}
			break;
		case cGet:
			if (offsetType) {
				offset = getOffsetAccelIMU(dev,ax);
			} else {
				offset = getOffsetGyroIMU(dev,ax);
			}
			printf_mux("%s = %0.5f\r\n", axName, offset);
			break;
	}	
}

void cKalQAngle(commVar val){	
	cmdHandlerKalman(val, &kalmanC.Qangle, "QAngle");
}

void cKalQBias(commVar val){	
	cmdHandlerKalman(val, &kalmanC.Qbias, "QBias");
}

void cKalRMeasure(commVar val){	
	cmdHandlerKalman(val, &kalmanC.Rmeasure, "RMeasure");
}

static void cmdHandlerKalman(commVar val, double *Kvalue, const char *valName){
	float value = val.value;
	
	switch (val.type)
	{
		case cSet:
		if (value > 0){
			(*Kvalue) = value;
			printf_mux("%s = %0.5f\r\n",valName ,(*Kvalue));
		} else {
			printf_mux("%s Value Error [%f]\r\n", valName, value);
		}
		break;
		case cGet:
		printf_mux("%s = %0.5f\r\n", valName, (*Kvalue));
		break;
	}
	
}