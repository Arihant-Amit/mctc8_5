/*
 * NEW_LCD.h
 *
 *  Created on: Mar 4, 2023
 *      Author: AMIT
 */


#include "main.h"
#include "cmsis_os.h"
#include "string.h"
#include "stdio.h"

#define SLAVE_ADDRESS_LCD 0x4E

void Lcd_init (void);
void Lcd_send_data (char data);
void Lcd_send_cmd (char cmd);
void Lcd_send_string (char *str);
void LCD_Clear();
void LCD_setCursor_xy (char row, char pos);
