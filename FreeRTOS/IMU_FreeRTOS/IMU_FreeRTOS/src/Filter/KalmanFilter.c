/*
 * KalmanFilter.c
 *
 * Created: 18/05/2016 22:30:07
 *  Author: Caio
 */ 

#include "KalmanFilter.h"

double Qangle = 0;
double QgyroBias = 0;
double Rmeasure = 0;
double angle = 0;
double bias = 0;

double P[2][2] = {0};

void initKalman(KalmanConst *kalmanInit){
	Qangle		=	kalmanInit->Qangle;
	QgyroBias	=	kalmanInit->Qbias;
	Rmeasure	=	kalmanInit->Rmeasure;
	
	angle		=	kalmanInit->angleInit;	//Reset Angle
	bias		=	kalmanInit->bias;		//Reset Bias
	
	P[0][0]		=	kalmanInit->P[0][0];	//Initialize Error Covariance Matrix
	P[1][0]		=	kalmanInit->P[1][0];
	P[0][1]		=	kalmanInit->P[0][1];
	P[1][1]		=	kalmanInit->P[1][1];
}

double getKalmanAngle (float newAngle, float newRate, float dt){
	
	// Discrete Kalman filter time update equations - Time Update ("Predict")
	// Update xhat - Project the state ahead
	/* Step 1 */
	double rate = newRate - bias;
	angle += dt*rate;
	
	// Update estimation error covariance - Project the error covariance ahead
	/* Step 2 */
	P[0][0] += dt * (dt*P[1][1] - P[0][1] - P[1][0] + Qangle);
	P[0][1] -= dt * P[1][1];
	P[1][0] -= dt * P[1][1];
	P[1][1] += QgyroBias * dt;
	
	// Discrete Kalman filter measurement update equations - Measurement Update ("Correct")
	// Calculate Kalman gain - Compute the Kalman gain
	/* Step 4 */
	double S = P[0][0] + Rmeasure; // Estimate error
	/* Step 5 */
	double K[2]; // Kalman gain - This is a 2x1 vector
	K[0] = P[0][0]/S;
	K[1] = P[1][0]/S;
	
	// Calculate angle and bias - Update estimate with measurement zk (newAngle)
	/* Step 3 */
	double y = newAngle - angle;
	/* Step 6 */
	angle += K[0] * y;
	bias += K[1] * y;
	
	// Calculate estimation error covariance - Update the error covariance
	/* Step 7 */
	double P00_temp = P[0][0];
	double P01_temp = P[0][1];
	
	P[0][0] -= K[0] * P00_temp;
	P[0][1] -= K[0] * P01_temp;
	P[1][0] -= K[1] * P00_temp;
	P[1][1] -= K[1] * P01_temp;
	
	return angle;
}