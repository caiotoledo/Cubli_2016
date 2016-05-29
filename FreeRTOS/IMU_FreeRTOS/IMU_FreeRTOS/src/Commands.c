/*
 * Commands.c
 *
 * Created: 28/05/2016 01:02:07
 *  Author: Caio
 */ 

#include "Commands.h"
#include "UART_Comm.h"
#include <string.h>
#include <ctype.h>

const str2State commandsMap[] = {
	{ "go",		sGo},
	{ "timer",	sTimer},
	{ "conf",	sConf},
	{ "task",	sTask},
	{ "imu",	sIMU},
	{ "kal",	sKalman},
	{ "compl",	sComplementary},
	{ "get",	sGet},
	{ "sample",	sSample},
	{ NULL,		sUnknow}
};

static str2State bufferCmd[10];

static uint32_t parseCMD(char *buf){
	uint32_t i = 0;
	char *pch;
	
	if (strstr(buf, ";")){
		pch = strtok(buf, ";");
		while( pch != NULL ){
			strcpy(bufferCmd[i].str, pch);
			bufferCmd[i].st = cmdToState(pch);
			pch = strtok(NULL, ";");
		}
	}
	
	return i;
}

static state cmdToState(char *buf){
	uint32_t i;
	for (i = 0; commandsMap[i].str != NULL; i++){
		if (strcmp(buf, commandsMap[i].str) == 0){
			//Comando encontrado, retornar o estado:
			return commandsMap[i].st;
		}
	}
	
	//Se não encontrado, retornar Unknow
	return sUnknow;
}

void receiveCMD(char *buf){
	uint32_t numCmd = 0;
	uint32_t i = 0;
	
	memset(bufferCmd, 0, sizeof(bufferCmd));
	
	numCmd = parseCMD(buf);
	
	if (!executeCMD()){
		sendErrorCMD(buf);
	}
}

static bool executeCMD(void){
	bool flag = true;
	switch (bufferCmd[0].st)
	{
	case sTimer:
		flag = setTimer(1);
		break;
	case sConf:
		flag = setConf(1);
		break;
	case sGo:
		flag = setGo(1);
		break;
	default:
		flag = false;
		break;
	}
	return flag;
}

static bool setTimer(uint8_t index){
	bool flag = true;
	float num;
	if (!isFloat(bufferCmd[index+1].str)){
		sendErrorNumber(index+1);
		flag = false;
		return flag;
	} else {
		num = atoff(bufferCmd[index+1].str);
	}
	
	switch (bufferCmd[index].st)
	{
	case sSample:
		//Configure Total time of Sampling
		setTimerSample(num);
		break;
	case sTask:
		//Configure Task Sampling time of IMU sensor;
		break;
	case sIMU:
		//Configure IMU Sampling time;
		break;
	default:
		flag = false;
		break;
	}
	return flag;
}

static bool setConf(uint8_t index){
	bool flag = true;
	if (!isFloat(bufferCmd[index+1].str)){
		sendErrorNumber(index+1);
		return false;
	}
	
	switch (bufferCmd[index].st)
	{
	case sKalman:
		if (!isFloat(bufferCmd[index+2].str) || !isFloat(bufferCmd[index+3].str)){
			sendErrorNumber(index+2);
			sendErrorNumber(index+3);
			flag = false;
			break;
		}
		//Configure Kalman Constants
		break;
	case sComplementary:
		//Configure Complementary Filter Constant
		break;
	case sGet:
		//Show All Configurations
		break;
	default:
		flag = false;
		break;
	}
	return flag;
}

static bool setGo(uint8_t index){
	bool flag = true;
	if (!isFloat(bufferCmd[index+1].str)){
		sendErrorNumber(index+1);
		return false;
	}
	
	if (strcmp(bufferCmd[index+1].str, "1")){
		//Execute test reseting the angle values
	} else if (strcmp(bufferCmd[index+1].str, "0")){
		//Execute test with the actual angle values
		startSample(0);
	} else {
		flag = false;
	}
	return flag;
}

static uint8_t isFloat(char *str){
	while(*str){
		if ( (!isdigit(*str)) && ( (*str) != '.' ) ){
			return false;
		}
		str++;
	}
	return true;
}

void sendErrorNumber(uint8_t index){
	printf_mux("\tERROR COMMAND: Value [%s] is not a Number\r\n", bufferCmd[index].str);
}

void sendErrorCMD(char *buf){
	printf_mux("\tERROR COMMAND [%s]\r\n", buf);
}