/*
 * ComplementaryFilter.h
 *
 * Created: 26/07/2016 22:43:07
 *  Author: Caio
 */ 


#ifndef COMPLEMENTARYFILTER_H_
#define COMPLEMENTARYFILTER_H_

#include "HAL/HAL_IMU_MPU6050.h"

Bool setAlpha(double val);
double getAlpha(void);
double initComplFilter(IMU_Addr_Dev dev);
void getComplFilterAngle(double *angle, double angle_measure, double *acel, double *gyro, double dt);

#endif /* COMPLEMENTARYFILTER_H_ */