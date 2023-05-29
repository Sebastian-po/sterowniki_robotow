/**
  ******************************************************************************
  * @file           : app.h
  * @brief          : Header for app.c file.
  ******************************************************************************
  */

#ifndef INC_APP_H_
#define INC_APP_H_

#include "stm32l476g_discovery_glass_lcd.h"

typedef enum {
	SELECT,
	LEFT,
	RIGHT,
	UP,
	DOWN,
	NONE
} App_EventTypeDef;

void APP_Init(void);
void APP_HandleEvent(App_EventTypeDef event);

#endif /* INC_APP_H_ */
