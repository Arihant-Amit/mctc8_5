/*
 * FLASH.c
 *
 *  Created on: Feb 28, 2023
 *      Author: AMIT
 */

#include "FLASH.h"
#include "stdio.h"

extern SPI_HandleTypeDef hspi1;

void SLAVE_CS_ENABLE(void)
{
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port,FLASH_CS_Pin, GPIO_PIN_RESET);
}

void SLAVE_CS_DISABLE(void)
{
	HAL_GPIO_WritePin(FLASH_CS_GPIO_Port,FLASH_CS_Pin, GPIO_PIN_SET);
}

uint8_t SPI_Byte_Write(uint8_t Data)
{
	uint8_t ret;
	HAL_SPI_TransmitReceive(&hspi1, &Data, &ret, 1, 100);
	return ret;
}

void SLAVE_Write_Enable(void)
{
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x06);
	SLAVE_CS_DISABLE();
	osDelay(10);
}

void SLAVE_Write_Disable(void)
{
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x04);
	SLAVE_CS_DISABLE();
	osDelay(10);
}



void SLAVE_Read_ID(void)
{
	uint8_t m,t,c;
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x9F);
	/*for(int i=0;i<80;i++)
	{
		buf[i] = 	SPI_Byte_Write(0x00);
	}*/
	m = SPI_Byte_Write(0x00);
	t = SPI_Byte_Write(0x00);
	c = SPI_Byte_Write(0x00);

	SLAVE_CS_DISABLE();
	osDelay(10);
	printf("ID: %d Memory Type : %d Capacity : %d\r\n",m,t,c);
}

uint8_t SLAVE_Read_StatusReg1(void)
{
	uint8_t status =0;
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x05);
	status = SPI_Byte_Write(0x00);
	SLAVE_CS_DISABLE();
	return status;
}

uint8_t SLAVE_Read_StatusReg2(void)
{
	uint8_t status =0;
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x07);
	status = SPI_Byte_Write(0x00);
	SLAVE_CS_DISABLE();
	return status;
}

void SLAVE_Wait(void)
{
	uint8_t status =0;
	osDelay(10);
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x05);
	do
	{
		status = SLAVE_Read_StatusReg1();
		osDelay(10);
	}while((status & 0x01)==0x01);
	SLAVE_CS_DISABLE();
}

void SLAVE_Clear_StatusReg(void)
{
	SLAVE_Write_Enable();
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x30);
	SLAVE_CS_DISABLE();
	SLAVE_Write_Disable();
	SLAVE_Wait();
}

void SLAVE_Full_Erase(void)
{
	SLAVE_Wait();
	SLAVE_Write_Enable();
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x60);
	SLAVE_CS_DISABLE();
	SLAVE_Write_Disable();
	SLAVE_Wait();
}

void SLAVE_Erase_4K(int loc)
{
	 SLAVE_Wait();
	 SLAVE_Write_Enable();
	 SLAVE_CS_ENABLE();
	 SPI_Byte_Write(0xDC);
	 SPI_Byte_Write((loc & ADDR_BMASK3) >> ADDR_BSHIFT3);
	 SPI_Byte_Write((loc & ADDR_BMASK2) >> ADDR_BSHIFT2);
	 SPI_Byte_Write((loc & ADDR_BMASK1) >> ADDR_BSHIFT1);
	 SPI_Byte_Write((loc & ADDR_BMASK0) >> ADDR_BSHIFT0);

	 SLAVE_CS_DISABLE();
	 SLAVE_Write_Disable();
	 SLAVE_Wait();

}

void SLAVE_Write_Data(int loc,char* Data,int count)
{
	if(count<1)
			return;
	SLAVE_Write_Enable();
	osDelay(10);
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x12);
	 SPI_Byte_Write((loc & ADDR_BMASK3) >> ADDR_BSHIFT3);
		 SPI_Byte_Write((loc & ADDR_BMASK2) >> ADDR_BSHIFT2);
		 SPI_Byte_Write((loc & ADDR_BMASK1) >> ADDR_BSHIFT1);
		 SPI_Byte_Write((loc & ADDR_BMASK0) >> ADDR_BSHIFT0);
for(int i=0;i<count;i++)
{
       SPI_Byte_Write(Data[i]);
}
       osDelay(10);
    SLAVE_CS_DISABLE();
    SLAVE_Write_Disable();
    SLAVE_Wait();
}


void SLAVE_Read_Data(int loc, char* Buf,int count)
{
	int i;
	if(count<1)
		return;
	SLAVE_CS_ENABLE();
	SPI_Byte_Write(0x13);
	 SPI_Byte_Write((loc & ADDR_BMASK3) >> ADDR_BSHIFT3);
		 SPI_Byte_Write((loc & ADDR_BMASK2) >> ADDR_BSHIFT2);
		 SPI_Byte_Write((loc & ADDR_BMASK1) >> ADDR_BSHIFT1);
		 SPI_Byte_Write((loc & ADDR_BMASK0) >> ADDR_BSHIFT0);

		 for(i=0;i<count;i++)
		 {
	Buf[i]=SPI_Byte_Write(0x00);
		 }
		 Buf[i]='\0';
    SLAVE_CS_DISABLE();
    //SLAVE_Wait();


}


