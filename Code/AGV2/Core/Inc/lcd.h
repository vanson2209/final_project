#ifndef __lcd_h
#define __lcd_h
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <string.h>
void Lcd_Init (void);   // initialize lcd

void Lcd_Send_Cmd (char cmd);  // send command to the lcd

void Lcd_Send_Data (char data);  // send data to the lcd

void Lcd_Send_String (char *str);  // send string to the lcd

void Lcd_Clear_Display (void);	//clear display lcd

void Lcd_Goto_XY (int row, int col); //set proper location on screen

void Lcd_Send_String_XY(int row, int col, char *str);
#endif
