/*
 * KalmanFilter.h
 *
 * Created: 18/05/2016 22:30:21
 *  Author: Caio
 */ 


#ifndef KALMANFILTER_H_
#define KALMANFILTER_H_


typedef struct KalmanConst_t{
	double Qangle;
	double Qbias;
	double Rmeasure;
	double angleInit;
	double bias;
	double P[2][2];
}KalmanConst;

void initKalman(KalmanConst *kalmanInit);
double getKalmanAngle (float newAngle, float newRate, float dt);

#endif /* KALMANFILTER_H_ */