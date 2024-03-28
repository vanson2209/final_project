#include "lcd.h"
#include "stm32f1xx.h"
extern I2C_HandleTypeDef hi2c2;  // change your handler here accordingly
extern volatile uint32_t systick_count;
#define SLAVE_ADDRESS_LCD 0x4E // change this according to ur setup

void Lcd_Send_Cmd (char cmd)
{
  char data_u, data_l;
	uint8_t data_t[4];
	data_u = (cmd&0xf0);
	data_l = ((cmd<<4)&0xf0);
	data_t[0] = data_u|0x0C;  //en=1, rs=0
	data_t[1] = data_u|0x08;  //en=0, rs=0
	data_t[2] = data_l|0x0C;  //en=1, rs=0
	data_t[3] = data_l|0x08;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void Lcd_Send_Data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=0
	data_t[1] = data_u|0x09;  //en=0, rs=0
	data_t[2] = data_l|0x0D;  //en=1, rs=0
	data_t[3] = data_l|0x09;  //en=0, rs=0
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

void Lcd_Init (void)
{
	Lcd_Send_Cmd (0x33); /* set 4-bits interface */
	Lcd_Send_Cmd (0x32);
	systick_count = 0;
	while(systick_count < 50);
	Lcd_Send_Cmd (0x28); /* start to set LCD function */
	systick_count = 0;
	while(systick_count < 50);
	Lcd_Send_Cmd (0x01); /* clear display */
	systick_count = 0;
	while(systick_count < 50);
	Lcd_Send_Cmd (0x06); /* set entry mode */
	systick_count = 0;
	while(systick_count < 50);
	Lcd_Send_Cmd (0x0C); /* set display to on */	
	systick_count = 0;
	while(systick_count < 50);
	Lcd_Send_Cmd (0x02); /* move cursor to home and set data address to 0 */
	systick_count = 0;
	while(systick_count < 50);
	Lcd_Send_Cmd (0x80);
}

void Lcd_Send_String (char *str)
{
	while (*str) Lcd_Send_Data (*str++);
}
void Lcd_Clear_Display (void)
{
	Lcd_Send_Cmd (0x01);
//	HAL_Delay(10);
}

void Lcd_Goto_XY (int row, int col)
{
	uint8_t pos_Addr;
	if(row == 1) 
	{
		pos_Addr = 0x80 + row - 1 + col;
	}
	else
	{
		pos_Addr = 0x80 | (0x40 + col);
	}
	Lcd_Send_Cmd(pos_Addr);
//	HAL_Delay(10);
}
void Lcd_Send_String_XY(int row, int col, char *str){
	Lcd_Clear_Display();
	Lcd_Goto_XY(row, col);
	while (*str) Lcd_Send_Data (*str++);
//	HAL_Delay(10);
}
