/*
 * ComplementaryFilter.h
 *
 * Created: 26/07/2016 22:43:07
 *  Author: Caio
 */ 


#ifndef COMPLEMENTARYFILTER_H_
#define COMPLEMENTARYFILTER_H_

#include "HAL/HAL_IMU.h"

double initComplFilter(ADXL_Addr_Dev dev);
void getComplFilterAngle(double *angle, double *acel, double *gyro, double dt);

#endif /* COMPLEMENTARYFILTER_H_ */