/*
 * Commands.h
 *
 * Created: 28/05/2016 01:02:33
 *  Author: Caio
 */ 


#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <asf.h>

typedef struct commVar_t commVar;

typedef void (*funcCommand)(commVar);

typedef struct commVar_t{
	funcCommand func;
	uint8_t type;
	float value;
}commVar;

typedef enum {
	cGet,
	cSet	
} cType;

void receiveCMD(char *buf);
void sendErrorCMD(char *buf);
void cUnknowCommand(commVar values);

#endif /* COMMANDS_H_ */