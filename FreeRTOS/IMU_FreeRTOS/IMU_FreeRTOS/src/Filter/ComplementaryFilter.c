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

double getAlpha(){
	return alpha;
}

double initComplFilter(ADXL_Addr_Dev dev){
	double acelInit[3];
	getAllAcelValue(dev, acelInit);
	return getPureAngle(acelInit);
}

void getComplFilterAngle(double *angle, double *acel, double *gyro, double dt){
	double angle_measure;
	
	angle_measure = getPureAngle(acel);
	
	*angle = (*angle + (gyro[Axis_Z]*dt) )*alpha + (1-alpha)*angle_measure;
}