/*
 * Commands.h
 *
 * Created: 28/05/2016 01:02:33
 *  Author: Caio
 */ 


#ifndef COMMANDS_H_
#define COMMANDS_H_

#include <asf.h>
#include "HAL/HAL_IMU_MPU6050.h"

typedef enum {
	cGet,
	cSet
} cType;

typedef struct commVar_t commVar;

typedef void (*funcCommand)(commVar);

typedef struct commVar_t{
	funcCommand func;
	cType type;
	float value;
	IMU_Addr_Dev device;
}commVar;

void receiveCMD(char *buf);
void sendErrorCMD(char *buf);
void cUnknowCommand(commVar values);

#endif /* COMMANDS_H_ */