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
#define RELEASE_VERSION					4

#define __YEAR__ ((((__DATE__ [7] - '0') * 10 + (__DATE__ [8] - '0')) * 10 + (__DATE__ [9] - '0')) * 10 + (__DATE__ [10] - '0'))
#define __MONTH__ ( __DATE__ [2] == 'n' ? (__DATE__ [1] == 'a' ? 1 : 6) : __DATE__ [2] == 'b' ? 2 : __DATE__ [2] == 'r' ? (__DATE__ [0] == 'M' ? 3 : 4) : __DATE__ [2] == 'y' ? 5 : __DATE__ [2] == 'l' ? 7 : __DATE__ [2] == 'g' ? 8 : __DATE__ [2] == 'p' ? 9 : __DATE__ [2] == 't' ? 10 : __DATE__ [2] == 'v' ? 11 : 12)
#define __DAY__ ((__DATE__ [4] == ' ' ? 0 : __DATE__ [4] - '0') * 10 + (__DATE__ [5] - '0'))
#define __HOUR__ ((__TIME__ [0] - '0') * 10 + (__TIME__ [1] - '0'))
#define __MINUTE__ ((__TIME__ [3] - '0') * 10 + (__TIME__ [4] - '0'))
#define __INT_TIMESTAMP__ ( (__YEAR__ - 2000) * 100000000 + __MONTH__ * 1000000 + __DAY__ * 10000 + __HOUR__ * 100 + __MINUTE__)

/* Show the current Version */
void formatVersion(char *buildVersion){
	sprintf(buildVersion, "%u.%u.%u.%d", MAJOR_VERSION, MINOR_VERSION, RELEASE_VERSION, __INT_TIMESTAMP__);
}