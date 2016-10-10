/*
 * HAL_Encoder.h
 *
 * Created: 12/09/2016 22:29:36
 *  Author: Caio
 */ 


#ifndef HAL_ENCODER_H_
#define HAL_ENCODER_H_

void resetCounterEncoder(void);
void setCounterEncoder(Bool degrees, double value);
double getAngleEncoder(Bool degrees);
void configEncoderPin(void);

#endif /* HAL_ENCODER_H_ */