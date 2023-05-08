/*
 * LORA.h
 *
 *  Created on: Feb 24, 2023
 *      Author: AMIT
 */

#ifndef INC_LORA_H_
#define INC_LORA_H_

#include "main.h"
#include "cmsis_os.h"
#include "stdbool.h"
#include "string.h"

bool RADIO_START(uint8_t Device);
void RADIO_SET_MODE(uint8_t modeconfig);
void RADIO_RESET(void);
void RADIO_CHECK_BUSY(void);
bool RADIO_CHECK_DEVICE();
uint8_t RADIO_READ_REG(uint16_t address);
void RADIO_READ_REGISTERS(uint16_t address, uint8_t *buffer, uint16_t size);
void RADIO_WRITE_REG( uint16_t address, uint8_t value );
void RADIO_WRITE_REGISTERS( uint16_t address, uint8_t *buffer, uint16_t size );
void RADIO_SETUP_LORA(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t modParam4);
void RADIO_SET_MODE(uint8_t modeconfig);
void RADIO_SET_REGULATOR_MODE(uint8_t mode);
void RADIO_WRITE_COMMAND(uint8_t Opcode,uint8_t *buffer,uint16_t size);
void RADIO_SET_PA_CONFIG(uint8_t dutycycle, uint8_t hpMax, uint8_t device);
void RADIO_SET_DIO3_TCX0_CTRL(uint8_t tcxovoltage);
void RADIO_CALIBRATE_DEVICE(uint8_t devices);
void RADIO_SET_DIO2_RFSWITCH_CTRL();
void RADIO_CALIBRATE_IMAGE(uint32_t freq);
void RADIO_SET_PACKET_TYPE(uint8_t packettype);
void RADIO_SET_RF_FREQUENCY(uint32_t frequency,int32_t offset);
void RADIO_SET_MODULATION_PARAMS(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t  modParam4);
uint8_t RADIO_RETURN_OPTIMISATION(uint8_t SpreadingFactor,uint8_t Bandwidth);
uint32_t RADIO_RETURN_BANDWIDTH(uint8_t BWregvalue);
float RADIO_CALC_SYMBOL_TIME(float Bandwidth,uint8_t SpreadingFactor);
void RADIO_SET_BUFFER_BASE_ADDRESS(uint8_t txBaseAddress, uint8_t rxBaseAddress);
void RADIO_SET_PACKET_PARAMS(uint16_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5);
void RADIO_SET_DIOIrq_PARAMS(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask );
void RADIO_SET_HIGH_SENSITIVITY();
void RADIO_SET_SYNC_WORD(uint16_t syncword);
void RADIO_SET_RX(uint32_t timeout);
uint8_t RADIO_RECEIVE(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait);
void RADIO_READ_COMMAND(uint8_t Opcode, uint8_t *buffer, uint16_t size);
uint16_t RADIO_READ_IRQ_STATUS();
void RADIO_CLEAR_IRQ_STATUS(uint16_t irqMask);
uint8_t RADIO_TRANSMIT(uint8_t *txbuffer, uint8_t size, uint32_t txtimeout, int8_t txpower, uint8_t wait);
void RADIO_SET_TX(uint32_t timeout);
void RADIO_SET_TX_PARAMS(int8_t TXpower, uint8_t RampTime);




#endif /* INC_LORA_H_ */
