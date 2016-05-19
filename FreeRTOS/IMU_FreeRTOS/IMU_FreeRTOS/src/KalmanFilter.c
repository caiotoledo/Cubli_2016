/*
 * KalmanFilter.c
 *
 * Created: 18/05/2016 22:30:07
 *  Author: Caio
 */ 

#include "KalmanFilter.h"

double Qangle = 0;
double Qbias = 0;
double Rmeasure = 0;
double angle = 0;
double bias = 0;

double P[2][2] = {0};

void initKalman(KalmanConst *kalmanInit){
	Qangle		=	kalmanInit->Qangle;
	Qbias		=	kalmanInit->Qbias;
	Rmeasure	=	kalmanInit->Rmeasure;
	
	angle		=	kalmanInit->angleInit;	//Reset Angle
	bias		=	kalmanInit->bias;		//Reset Bias
	
	P[0][0]		=	kalmanInit->P[0][0];	//Initialize Error Covariance Matrix
	P[1][0]		=	kalmanInit->P[1][0];
	P[0][1]		=	kalmanInit->P[0][1];
	P[1][1]		=	kalmanInit->P[1][1];
}

double getKalmanAngle (float newAngle, float newRate, float dt){
	
	double rate = newRate - bias;
	angle += dt*rate;
	
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Qangle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += Qbias * dt;
	
	double S = P[0][0] + Rmeasure;
	
	double K[2];
	K[0] = P[0][0]/S;
	K[1] = P[1][0]/S;
	
	double y = newAngle - angle;
	
	angle += K[0] * y;
	bias += K[1] * y;
	
	double P00_temp = P[0][0];
	double P01_temp = P[0][1];
	
	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;
	
	return angle;
}