/*
 * Commands.c
 *
 * Created: 28/05/2016 01:02:07
 *  Author: Caio
 */ 

#include "Commands.h"
#include "UART_Comm.h"
#include "HAL/HAL_UART.h"
#include "IMU.h"
#include <string.h>
#include <ctype.h>

static void parseCMD(char *buf, commVar *cmdParse);
static funcCommand cmdToFunc(char *buf);
static uint8_t isFloat(char *str);

/*
	Mapping all commands, it should follow this Syntax:
	"Command;Type;Value"
*/
const str2Func functionsMap[] = {
	{	"go",			cStartSample},
	{	"resetVar",		cResetVariables},
	{	"goReset",		cStartSampleReset},
	{	"tTotalSample",	cTotalTimeTest},
	{	"tTaskSample",	cTaskSample},
	{	"kalQAngle",	cKalQAngle},
	{	"kalQBias",		cKalQBias},
	{	"kalRMeasure",	cKalRMeasure},
	{	"alphaCFilter",	cAlphaComplFilter},
	{	"offsetAccelX",	cOffsetAccelX},
	{	"offsetAccelY",	cOffsetAccelY},
	{	"offsetAccelZ",	cOffsetAccelZ},
	{	"offsetGyroX",	cOffsetGyroX},
	{	"offsetGyroY",	cOffsetGyroY},
	{	"offsetGyroZ",	cOffsetGyroZ},
	{	"end",			cUnknowCommand},
};

static void parseCMD(char *buf, commVar *cmdParse){
	uint32_t i = 0;
	cmdParse->func = NULL;
	cmdParse->type = 0;
	cmdParse->value = 0;
	char *pch;
	
	if (strstr(buf, ";")){
		pch = strtok(buf, ";");
		while( pch != NULL ){
			switch (i++){
				case 0:
					cmdParse->func = cmdToFunc(pch);
					break;
				case 1:
					if (isFloat(pch)){
						cmdParse->type = atoi(pch);
					} else {
						cmdParse->type = 0;
					}					
					break;
				case 2:
					if (isFloat(pch)){
						cmdParse->value = atoff(pch);
					} else {
						cmdParse->value = 0;
					}
					break;
			}
			pch = strtok(NULL, ";");
		}
	} else {
		cmdParse->func = cmdToFunc(buf);
	}
}

static funcCommand cmdToFunc(char *buf){
	uint32_t i;
	for (i = 0; functionsMap[i].func != cUnknowCommand; i++){
		if (strcmp(buf, functionsMap[i].str) == 0){
			//Comando encontrado, retornar função:
			return functionsMap[i].func;
		}
	}
	
	//Se não encontrado, retornar função de Command Unknow
	return cUnknowCommand;
}

void receiveCMD(char *buf){
	commVar cmdVal;
	
	parseCMD(buf, &cmdVal);
	
	if (cmdVal.func){
		cmdVal.func(cmdVal);
	} else {
		sendErrorCMD(buf);
	}
}

static uint8_t isFloat(char *str){
	while(*str){
		if ( (!isdigit((unsigned char)*str)) && ( (*str) != '.' ) ){
			return false;
		}
		str++;
	}
	return true;
}

void cUnknowCommand(commVar values){
	printf_mux("\tCOMMAND UNKNOW\r\n");
}

void sendErrorCMD(char *buf){
	printf_mux("\tERROR COMMAND [%s]\r\n", buf);
}