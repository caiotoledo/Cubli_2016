/*
 * utils.c
 *
 * Created: 10/11/2016 23:31:40
 *  Author: Caio
 */ 

#include <asf.h>
#include <string.h>
#include "utils.h"

#define MAJOR_VERSION					0
#define MINOR_VERSION					0
#define RELEASE_VERSION					1

const char mon2num[][20] = {
	"Jan",
	"Fev",
	"Mar",
	"Abr",
	"Mai",
	"Jun",
	"Jul",
	"Ago",
	"Set",
	"Out",
	"Nov",
	"Dez",
	" "
};

static uint32_t month2num(char * month){
	
	uint32_t i = 0;
	for (i = 0; strcmp(mon2num[i], " "); i++){
		if (strcmp(mon2num[i], month) == 0){
			return (i+1);
		}
	}
	return i;
}

/* Show the current Version */
void formatVersion(char *buildVersion){
	
	const char *dateNum = __DATE__;
	const char *timeNum = __TIME__;
	
	uint16_t seconds, minutes, hours, month, day, year;
	
	char buf[10] = { 0 };
	
	strncpy(buf, timeNum, 2);
	hours = atoi(buf);
	strncpy(buf, &timeNum[3], 2);
	minutes = atoi(buf);
	strncpy(buf, &timeNum[6], 4);
	seconds = atoi(buf);
	
	strncpy(buf, dateNum, 3);
	month = month2num(buf);
	strncpy(buf, &dateNum[4], 2);
	day = atoi(buf);
	strncpy(buf, &dateNum[7], 4);
	year = atoi(buf)-2000;
	
	sprintf(buildVersion, "%u.%u.%u.%02u%02u%02u%02u%02u%02u", MAJOR_VERSION, MINOR_VERSION, RELEASE_VERSION, year, month, day, hours, minutes, seconds);
}