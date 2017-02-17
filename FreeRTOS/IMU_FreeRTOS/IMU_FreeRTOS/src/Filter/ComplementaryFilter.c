/*
 * ComplementaryFilter.c
 *
 * Created: 26/07/2016 22:43:36
 *  Author: Caio
 */ 
#include <asf.h>

#include "ComplementaryFilter.h"

double alpha = 0.7143;//0.8333//0.9091

Bool setAlpha(double val){
	Bool resp = false;
	
	if (val >= 0 && val <= 1) {
		alpha = val;
		resp = true;
	}
	
	return resp;
}

double getAlpha(void){
	return alpha;
}

double initComplFilter(IMU_Addr_Dev dev){
	double acelInit[3];
	uint32_t status_dev;
	
	/* Check if IMU exists */
	status_dev = imu_probe(dev);
	if (status_dev != TWI_SUCCESS) {
		if (dev == IMU_Low) {
			dev = IMU_High;
		} else {
			dev = IMU_Low;
		}
	}
	
	getAllAcelValue(dev, acelInit);
	return getPureAngle(acelInit);
}

void getComplFilterAngle(double *angle, double angle_measure, double *acel, double *gyro, double dt){	
	*angle = (*angle + (gyro[Axis_Z]*dt) )*alpha + (1-alpha)*(angle_measure) ;
}