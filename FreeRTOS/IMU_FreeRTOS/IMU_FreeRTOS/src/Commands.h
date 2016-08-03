/*
 * Commands.h
 *
 * Created: 28/05/2016 01:02:33
 *  Author: Caio
 */ 


#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <asf.h>

typedef void (*funcCommand)(commVar);

typedef struct commVar_t {
	funcCommand func;
	uint8_t type;
	float value;
}commVar;

typedef struct str2Func_t {
	char str[30];
	funcCommand func;
}str2Func;

typedef enum cType_e {
	cGet,
	cSet	
} cType;

void receiveCMD(char *buf);

static commVar parseCMD(char *buf);
static funcCommand cmdToFunc(char *buf);
static uint8_t isFloat(char *str);
void sendErrorCMD(char *buf);
void cUnknowCommand(commVar values);

#endif /* COMMANDS_H_ */