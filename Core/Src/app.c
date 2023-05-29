/**
  ******************************************************************************
  * @file           : app.c
  * @brief          : Application functionality
  ******************************************************************************
  */

#include "app.h"
#include "stm32l476g_discovery_glass_lcd.h"
#include "main.h"
#include <stdio.h>


typedef enum {
	MAIN_MENU,
	LOOPBACK,
	ABOUT
} App_StageTypeDef;

typedef enum {
	MAIN_MENU_LOOPBACK,
	MAIN_MENU_ABOUT,
	LOOPBACK_PLAY,
	LOOPBACK_PAUSE,
	LOOPBACK_RECORD,
	ABOUT_DISPLAY
} App_StateTypeDef;

App_StageTypeDef app_stage;
App_StateTypeDef app_state;
char VolumeString[14];

void APP_Init() {
	app_stage = MAIN_MENU;
	app_state = MAIN_MENU_LOOPBACK;
	BSP_LCD_GLASS_Clear();
	BSP_LCD_GLASS_DisplayString((uint8_t *)"LOOP");
}

void APP_HandleEvent(App_EventTypeDef event) {

	switch(event)
	{
	case SELECT:
	case RIGHT:
		if (app_stage == MAIN_MENU) {
			if (app_state == MAIN_MENU_LOOPBACK) {

				app_stage = LOOPBACK;
				app_state = LOOPBACK_PAUSE;

				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"PAUSED");
			}
			else if (app_state == MAIN_MENU_ABOUT) {

				app_stage = ABOUT;
				app_state = ABOUT_DISPLAY;

				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"WRO 23");

			}
		}
		else if (app_stage == LOOPBACK) {

			if (app_state == LOOPBACK_RECORD) {

				app_state = LOOPBACK_PAUSE;

				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"PAUSED");

				audio_drv->Pause(0x08080000);

			}
			else if (app_state == LOOPBACK_PAUSE) {

				app_state = LOOPBACK_PLAY;

				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"PLAY");

				audio_drv->Resume(AUDIO_I2C_ADDRESS);

			}
			else if (app_state == LOOPBACK_PLAY) {

				app_state = LOOPBACK_RECORD;

				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"RECORD");

				audio_drv->Pause(AUDIO_I2C_ADDRESS);
				audio_drv->Resume(0x08080000);

			}
		}
		break;
	case LEFT:
		if (app_stage== LOOPBACK) {
			app_stage = MAIN_MENU;
			app_state = MAIN_MENU_LOOPBACK;
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t *)"LOOP");
		}
		else if (app_stage == ABOUT) {
			app_stage = MAIN_MENU;
			app_state = MAIN_MENU_ABOUT;
			BSP_LCD_GLASS_Clear();
			BSP_LCD_GLASS_DisplayString((uint8_t *)"ABOUT");
		}
		audio_drv->Pause(AUDIO_I2C_ADDRESS);
		break;
	case UP:
		if (app_stage == MAIN_MENU) {
			if (app_state == MAIN_MENU_LOOPBACK) {
				app_state = MAIN_MENU_ABOUT;
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"ABOUT");
			}
			else if (app_state == MAIN_MENU_ABOUT) {
				app_state = MAIN_MENU_LOOPBACK;
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"LOOP");
			}
		}

		if (app_stage == LOOPBACK) {
			volume++;
			if (volume > 100) {
				volume = 100;
			}
			audio_drv->SetVolume(AUDIO_I2C_ADDRESS, volume);
		    sprintf(VolumeString, "VOL%u", volume);
		    BSP_LCD_GLASS_Clear();
		    BSP_LCD_GLASS_DisplayString((uint8_t*)VolumeString);
		}
		break;
	case DOWN:
		if (app_stage == MAIN_MENU) {
			if (app_state == MAIN_MENU_LOOPBACK) {
				app_state = MAIN_MENU_ABOUT;
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"ABOUT");
			}
			else if (app_state == MAIN_MENU_ABOUT) {
				app_state = MAIN_MENU_LOOPBACK;
				BSP_LCD_GLASS_Clear();
				BSP_LCD_GLASS_DisplayString((uint8_t *)"LOOP");
			}
		}
		if (app_stage == LOOPBACK) {
			volume--;
			if (volume < 70) {
				volume = 70;
			}
			audio_drv->SetVolume(AUDIO_I2C_ADDRESS, volume);
		    sprintf(VolumeString, "VOL%u", volume);
		    BSP_LCD_GLASS_Clear();
		    BSP_LCD_GLASS_DisplayString((uint8_t*)VolumeString);
		}
		break;
	default:
		break;
	}


}
