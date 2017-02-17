/*
 * IMU.h
 *
 * Created: 03/04/2016 16:45:52
 *  Author: Caio
 */ 


#ifndef IMU_H_
#define IMU_H_

#include "Commands.h"

#define TWI_TASK_DELAY		(20/portTICK_RATE_MS)

//#define INT_PIN				PIO_PA20

extern volatile xQueueHandle xQueueAcel[2][NUM_AXIS];
extern volatile xQueueHandle xQueueAngle[];
extern volatile xQueueHandle xQueueGyro[2][NUM_AXIS];

volatile uint32_t g_tickCounter;

xSemaphoreHandle xseIMUValues;

void IMUTask(void *pvParameters);

void intpin_handler(uint32_t id, uint32_t mask);

void cResetVariables(commVar val);
void cStartSampleReset(commVar val);
void cTaskSample(commVar val);

//Calibration Function for IMU
void cRunCalibrationIMU(commVar val);

//Complementary Filter Constant:
void cAlphaComplFilter(commVar val);

//Kalman Filter Constants:
void cKalQAngle(commVar val);
void cKalQBias(commVar val);
void cKalRMeasure(commVar val);

//IMU Offset:
void cOffsetAccelX(commVar val);
void cOffsetAccelY(commVar val);
void cOffsetAccelZ(commVar val);
void cOffsetGyroX(commVar val);
void cOffsetGyroY(commVar val);
void cOffsetGyroZ(commVar val);

#endif /* IMU_H_ */