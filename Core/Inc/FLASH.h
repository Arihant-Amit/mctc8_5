/*
 * FLASH.h
 *
 *  Created on: Feb 28, 2023
 *      Author: AMIT
 */

#ifndef INC_FLASH_H_
#define INC_FLASH_H_

#include "main.h"
#include "cmsis_os.h"

#define ADDR_BMASK3     0xff000000
#define ADDR_BMASK2     0x00ff0000
#define ADDR_BMASK1     0x0000ff00
#define ADDR_BMASK0     0x000000ff

#define ADDR_BSHIFT3    24
#define ADDR_BSHIFT2    16
#define ADDR_BSHIFT1    8
#define ADDR_BSHIFT0    0


void SLAVE_CS_ENABLE(void);
void SLAVE_CS_DISABLE(void);
uint8_t SPI_Byte_Write(uint8_t Data);
void SLAVE_Write_Enable(void);
void SLAVE_Write_Disable(void);
void SLAVE_Read_ID(void);
uint8_t SLAVE_Read_StatusReg1(void);
uint8_t SLAVE_Read_StatusReg2(void);
void SLAVE_Wait(void);
void SLAVE_Clear_StatusReg(void);
void SLAVE_Full_Erase(void);
void SLAVE_Erase_4K(int loc);
void SLAVE_Write_Data(int loc,char* Data,int count);
void SLAVE_Read_Data(int loc, char* Buf,int count);

#endif /* INC_FLASH_H_ */
