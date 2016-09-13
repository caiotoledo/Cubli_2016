/*
 * HAL_Encoder.h
 *
 * Created: 12/09/2016 22:29:36
 *  Author: Caio
 */ 


#ifndef HAL_ENCODER_H_
#define HAL_ENCODER_H_

void resetCounterEncoder();
double getAngleEncoder(Bool degrees);
void configEncoderPin();

#endif /* HAL_ENCODER_H_ */