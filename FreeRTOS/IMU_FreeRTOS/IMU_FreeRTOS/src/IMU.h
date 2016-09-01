/*
 * IMU.h
 *
 * Created: 03/04/2016 16:45:52
 *  Author: Caio
 */ 


#ifndef IMU_H_
#define IMU_H_

#include "Commands.h"

#define TWI_TASK_DELAY		(10/portTICK_RATE_MS)

//#define INT_PIN				PIO_PA20

extern volatile xQueueHandle xQueueAcel[];
extern volatile xQueueHandle xQueueAngle[];
extern volatile xQueueHandle xQueueGyro[];

volatile uint32_t g_tickCounter;

xSemaphoreHandle xseIMUValues;

void IMUTask(void *pvParameters);

void intpin_handler(uint32_t id, uint32_t mask);

void cTaskSample(commVar val);
void cKalQAngle(commVar val);
void cKalQBias(commVar val);
void cKalRMeasure(commVar val);

#endif /* IMU_H_ */