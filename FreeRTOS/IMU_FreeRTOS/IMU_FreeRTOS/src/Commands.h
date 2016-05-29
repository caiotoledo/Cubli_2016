/*
 * Commands.h
 *
 * Created: 28/05/2016 01:02:33
 *  Author: Caio
 */ 


#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <asf.h>

typedef enum eState {
	sUnknow,
	sGo,
	sTimer,
	sConf,
	sTask,
	sIMU,
	sSample,
	sKalman,
	sComplementary,
	sGet,	
} state;

typedef struct str2State_t {
	char str[10];
	state st;
}str2State;

void receiveCMD(char *buf);

static uint32_t parseCMD(char *buf);
static state cmdToState(char *buf);
static bool executeCMD(void);
static bool setTimer(uint8_t index);
static bool setConf(uint8_t index);
static bool setGo(uint8_t index);
static uint8_t isFloat(char *str);
void sendErrorNumber(uint8_t index);
void sendErrorCMD(char *buf);

#endif /* COMMANDS_H_ */