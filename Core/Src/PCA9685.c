/*
 * PCA9685.c
 *
 *  Created on: Jan 11, 2023
 *      Author: AMIT
 */
#include "main.h"
#include "PCA9685.h"

extern I2C_HandleTypeDef hi2c1;


PCA9685_STATUS PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value)
{
	uint8_t tmp;
	if(Value) Value = 1;

	if(HAL_OK != HAL_I2C_Mem_Read(&hi2c1, 0x80, Register, 1, &tmp, 1, 10))
	{
		return PCA9685_ERROR;
	}
	tmp &= ~((1<<7)|(1<<Bit));
	tmp |= (Value&1)<<Bit;

	if(HAL_OK != HAL_I2C_Mem_Write(&hi2c1, 0x80, Register, 1, &tmp, 1, 10))
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}

PCA9685_STATUS PCA9685_SoftwareReset(void)
{
	uint8_t cmd = 0x6;
	if(HAL_OK == HAL_I2C_Master_Transmit(&hi2c1, 0x80, &cmd, 1, 10))
	{
		return PCA9685_OK;
	}
	return PCA9685_ERROR;
}

PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable)
{
	return PCA9685_SetBit(0x00, 4, Enable);
}

PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable)
{
	return PCA9685_SetBit(0x00, 7, Enable);
}
PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable)
{
	return PCA9685_SetBit(0x00, 5, Enable);
}

PCA9685_STATUS PCA9685_SetPwmFrequency(uint16_t Frequency)
{
	float PrescalerVal;
	uint8_t Prescale;

	if(Frequency >= 1526)
	{
		Prescale = 0x03;
	}
	else if(Frequency <= 24)
	{
		Prescale = 0xFF;
	}
	else
	{
		PrescalerVal = (25000000 / (4096.0 * (float)Frequency)) - 1;
		Prescale = floor(PrescalerVal + 0.5);
	}

	//
	//	To change the frequency, PCA9685 have to be in Sleep mode.
	//
	PCA9685_SleepMode(1);
	HAL_I2C_Mem_Write(&hi2c1, 0x80, 0xFE, 1, &Prescale, 1, 10); // Write Prescale value
	PCA9685_SleepMode(0);
	PCA9685_RestartMode(1);
	return PCA9685_OK;
}
PCA9685_STATUS PCA9685_SetPwm(uint8_t Channel, uint16_t OnTime, uint16_t OffTime)
{
	uint8_t RegisterAddress;
	uint8_t Message[4];

	RegisterAddress = 0x06 + (4 * Channel);
	Message[0] = OnTime & 0xFF;
	Message[1] = OnTime>>8;
	Message[2] = OffTime & 0xFF;
	Message[3] = OffTime>>8;

	if(HAL_OK != HAL_I2C_Mem_Write(&hi2c1, 0x80, RegisterAddress, 1, Message, 4, 10))
	{
		return PCA9685_ERROR;
	}

	return PCA9685_OK;
}
PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert)
{
  if(Value > 4095) Value = 4095;

  if (Invert) {
    if (Value == 0) {
      // Special value for signal fully on.
      return PCA9685_SetPwm(Channel, 4096, 0);
    }
    else if (Value == 4095) {
      // Special value for signal fully off.
    	return PCA9685_SetPwm(Channel, 0, 4096);
    }
    else {
    	return PCA9685_SetPwm(Channel, 0, 4095-Value);
    }
  }
  else {
    if (Value == 4095) {
      // Special value for signal fully on.
    	return PCA9685_SetPwm(Channel, 4096, 0);
    }
    else if (Value == 0) {
      // Special value for signal fully off.
    	return PCA9685_SetPwm(Channel, 0, 4096);
    }
    else {
    	return PCA9685_SetPwm(Channel, 0, Value);
    }
  }
}
PCA9685_STATUS PCA9685_SetDuty(uint8_t Channel, float value)
{
	float Value;
	Value = floor((4095 * value));

	return PCA9685_SetPin(Channel, (uint16_t)Value, 0);
}
