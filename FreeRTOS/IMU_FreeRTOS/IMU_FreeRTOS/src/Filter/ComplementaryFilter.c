/*
 * ComplementaryFilter.c
 *
 * Created: 26/07/2016 22:43:36
 *  Author: Caio
 */ 
#include <asf.h>

#include "ComplementaryFilter.h"

#define ALPHA			0.7143//0.8333//0.9091

double initComplFilter(ADXL_Addr_Dev dev){
	double acelInit[3];
	getAllAcelValue(dev, acelInit);
	return getPureAngle(acelInit);
}

void getComplFilterAngle(double *angle, double *acel, double *gyro, double dt){
	double angle_measure;
	
	angle_measure = getPureAngle(acel);
	
	*angle = (*angle + (gyro[Axis_Z]*dt) )*ALPHA + (1-ALPHA)*angle_measure;
}