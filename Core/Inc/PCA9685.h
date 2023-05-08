/*
 * PCA9685.h
 *
 *  Created on: Jan 11, 2023
 *      Author: AMIT
 */

#ifndef INC_PCA9685_H_
#define INC_PCA9685_H_


#include "main.h"
#include "math.h"
/*******************************/
/*PWM GENERATOR TYPEDEF*/
typedef enum
{
	PCA9685_OK 		= 0,
	PCA9685_ERROR	= 1
}PCA9685_STATUS;
/*****************************/


PCA9685_STATUS PCA9685_SetBit(uint8_t Register, uint8_t Bit, uint8_t Value);
PCA9685_STATUS PCA9685_SoftwareReset(void);
PCA9685_STATUS PCA9685_SleepMode(uint8_t Enable);
PCA9685_STATUS PCA9685_RestartMode(uint8_t Enable);
PCA9685_STATUS PCA9685_SetPwmFrequency(uint16_t Frequency);
PCA9685_STATUS PCA9685_AutoIncrement(uint8_t Enable);
PCA9685_STATUS PCA9685_SetPwm(uint8_t Channel, uint16_t OnTime, uint16_t OffTime);
PCA9685_STATUS PCA9685_SetPin(uint8_t Channel, uint16_t Value, uint8_t Invert);
PCA9685_STATUS PCA9685_SetDuty(uint8_t Channel, float value);


#endif /* INC_PCA9685_H_ */
