/*
 * LCD.h
 *
 * Created: 12/04/2016 21:55:44
 *  Author: Caio
 */ 


#ifndef LCD_H_
#define LCD_H_

#include <asf.h>

#define NUM_AXIS		3

xQueueHandle xQueueAcel[NUM_AXIS];
xQueueHandle xQueueGyro[NUM_AXIS];

void config_lcd(void);
void LCDTask(void *pvParameters);

#endif /* LCD_H_ */