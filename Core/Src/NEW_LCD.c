/*
 * NEW_LCD.c
 *
 *  Created on: Mar 4, 2023
 *      Author: AMIT
 */

#include "NEW_LCD.h"


extern I2C_HandleTypeDef hi2c2;

void Lcd_init (void)
{
	// 4 bit initialisation
	Lcd_send_cmd(0x00);
	osDelay(50);  // wait for >40ms
	Lcd_send_cmd (0x30);
	osDelay(5);  // wait for >4.1ms
	Lcd_send_cmd (0x30);
	osDelay(1);  // wait for >100us
	Lcd_send_cmd (0x30);
	osDelay(10);
	Lcd_send_cmd (0x20);  // 4bit mode
	osDelay(10);

  // dislay initialisation
	Lcd_send_cmd (0x28); // Function set --> DL=0 (4 bit mode), N = 1 (2 line display) F = 0 (5x8 characters)
	//osDelay(1);
	//Lcd_send_cmd (0x08); //Display on/off control --> D=0,C=0, B=0  ---> display off
	//osDelay(1);
	Lcd_send_cmd (0x10);  // clear display
//	osDelay(1);
//	osDelay(1);
	Lcd_send_cmd (0x0F); //Entry mode set --> I/D = 1 (increment cursor) & S = 0 (no shift)
	//osDelay(1);
	Lcd_send_cmd (0x09); //Display on/off control --> D = 1, C and B = 0. (Cursor and blink, last two bits)
	Lcd_send_cmd(0x02);
	Lcd_send_cmd(0x01);
}

/********* LCD SEND DATA **********/

void Lcd_send_data (char data)
{
	char data_u, data_l;
	uint8_t data_t[4];
	data_u = (data&0xf0);
	data_l = ((data<<4)&0xf0);
	data_t[0] = data_u|0x0D;  //en=1, rs=1
	data_t[1] = data_u|0x09;  //en=0, rs=1
	data_t[2] = data_l|0x0D;  //en=1, rs=1
	data_t[3] = data_l|0x09;  //en=0, rs=1
	HAL_I2C_Master_Transmit (&hi2c2, SLAVE_ADDRESS_LCD,(uint8_t *) data_t, 4, 100);
}

/********* LCD SEND COMMAND **********/
void Lcd_send_cmd (char cmd)
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
/********* LCD SEND STRING  **********/

void Lcd_send_string (char *str)
{
	while (*str) Lcd_send_data (*str++);
}

/********* LCD SET CURSOR  **********/
void LCD_setCursor_xy (char row, char pos)	/* Send string to LCD with xy position */
{
	if (row == 0 && pos<16)
	Lcd_send_cmd((pos & 0x0F)|0x80);	/* Command of first row and required position<16 */
	else if (row == 1 && pos<16)
	Lcd_send_cmd((pos & 0x0F)|0xC0);	/* Command of first row and required position<16 */
			/* Call LCD string function */
}

/********* LCD CLEAR  **********/
void LCD_Clear()
{
	Lcd_send_cmd(0x01);
	osDelay(5);

}
