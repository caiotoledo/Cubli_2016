/*
 * HAL_Encoder.c
 *
 * Created: 12/09/2016 22:24:58
 *  Author: Caio
 */ 
#include <asf.h>
#include <math.h>

#define ID_ENC		ID_PIOA
#define GPIO_A		PIO_PA16
#define GPIO_B		PIO_PA17
#define GPIO_C		PIO_PA18

#define ENC_RES		1024
#define VOLTA_COMP	360

uint32_t aFlag;
uint32_t bFlag;

int32_t roundEncoder;

void resetCounterEncoder(){
	roundEncoder = 0;
	aFlag = 0;
	bFlag = 0;
}

double getAngleEncoder(Bool degrees){
	double val = -400;
	
	if (degrees){
		val = ((double)roundEncoder/ENC_RES)*VOLTA_COMP;
	} else {
		val = ((double)roundEncoder/ENC_RES)*M_TWOPI;
	}
	
	return val;
}

void pin_handler(uint32_t id, uint32_t mask){
	if (id == ID_ENC){
		switch (mask){
			case GPIO_A:
				if (bFlag){
					roundEncoder++;
				} else {
					aFlag = 1;
				}
			break;
			case GPIO_B:
				if (aFlag){
					roundEncoder--;
				} else {
					bFlag = 1;
				}
			break;
			case GPIO_C:
				bFlag = 0;
				aFlag = 0;
			break;
		}
	}
}

void configEncoderPin(){
	//Interrupt for A:
	pio_set_input(PIOA, GPIO_A, PIO_DEFAULT);
	pio_pull_down(PIOA, GPIO_A, ENABLE);
	pio_handler_set(PIOA, ID_ENC, GPIO_A, PIO_IT_RISE_EDGE, pin_handler);
	pio_enable_interrupt(PIOA, GPIO_A);
	
	//Interrupt for B:
	pio_set_input(PIOA, GPIO_B, PIO_DEFAULT);
	pio_pull_down(PIOA, GPIO_B, ENABLE);
	pio_handler_set(PIOA, ID_ENC, GPIO_B, PIO_IT_RISE_EDGE, pin_handler);
	pio_enable_interrupt(PIOA, GPIO_B);
	
	//Interrupt for C:
	pio_set_input(PIOA, GPIO_C, PIO_DEFAULT);
	pio_pull_down(PIOA, GPIO_C, ENABLE);
	pio_handler_set(PIOA, ID_ENC, GPIO_C, PIO_IT_FALL_EDGE, pin_handler);
	pio_enable_interrupt(PIOA, GPIO_C);
	
	resetCounterEncoder();
}