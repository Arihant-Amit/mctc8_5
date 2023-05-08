/*
 * LORA.c
 *
 *  Created on: Feb 24, 2023
 *      Author: AMIT
 */
#include "LORA.h"
#include "math.h"
extern SPI_HandleTypeDef hspi3;



uint8_t RADIO_READ_REGISTER		=0x1D;
uint8_t RADIO_WRITE_REGISTER	=0x0D;
uint8_t RADIO_SET_STANDBY	=	0x80;
uint8_t RADIO_READ_BUFFER =		0x1E;
float    FREQ_STEP                =                0.95367431640625;
uint8_t RADIO_WRITE_BUFFER	=	0x0E;
uint8_t savedModParam2;
void RADIO_WRITE_REGISTERS( uint16_t address, uint8_t *buffer, uint16_t size )
{
	  uint8_t addr_l, addr_h;


	  addr_l = address & 0xff;
	  addr_h = address >> 8;
	  RADIO_CHECK_BUSY();
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi3, (uint8_t *)&RADIO_WRITE_REGISTER, 1,100);
	  HAL_SPI_Transmit(&hspi3,(uint8_t *)& addr_h,1,100);
	  HAL_SPI_Transmit(&hspi3,(uint8_t *)& addr_l,1,100);
	  HAL_SPI_Transmit(&hspi3, buffer,size,100);
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, SET);

}
void RADIO_WRITE_REG( uint16_t address, uint8_t value )
{
	RADIO_WRITE_REGISTERS(address,&value,1);
}
void RADIO_READ_REGISTERS(uint16_t address, uint8_t *buffer, uint16_t size)
{
	  uint8_t addr_l, addr_h;
	  uint8_t dummy_data = 0xFF;
	  addr_h = address >> 8;
	  addr_l = address & 0x00FF;
	  RADIO_CHECK_BUSY();
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi3,(uint8_t*)& RADIO_READ_REGISTER, 1,100);
	  HAL_SPI_Transmit(&hspi3, (uint8_t *)&addr_h,1,100);
	  HAL_SPI_Transmit(&hspi3,(uint8_t *) &addr_l,1,100);
	  HAL_SPI_Transmit(&hspi3, (uint8_t*)&dummy_data,1,100);
	  HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&dummy_data, (uint8_t*)buffer, size, 100);
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, SET);
}
uint8_t RADIO_READ_REG(uint16_t address)
{
	  uint8_t data;
	  RADIO_READ_REGISTERS(address, &data, 1);
	  return data;
}
bool RADIO_CHECK_DEVICE()
{
	  uint8_t Regdata1, Regdata2;
	  Regdata1 = RADIO_READ_REG(0x88e);
	  RADIO_WRITE_REG(0x88e, (Regdata1 + 1));
	  Regdata2 = RADIO_READ_REG(0x88e);
	  RADIO_WRITE_REG(0x88e, Regdata1);

	  if (Regdata2 == (Regdata1 + 1))
	  {
	    return true;
	  }
	  else
	  {
	    return false;
	  }
}
void RADIO_CHECK_BUSY(void)
{
	  uint8_t busy_timeout_cnt;
	  busy_timeout_cnt = 0;
	  while(HAL_GPIO_ReadPin(LORA_BUSY_GPIO_Port, LORA_BUSY_Pin))
	  {
		  osDelay(1);
		  busy_timeout_cnt++;
		  if(busy_timeout_cnt>10)
		  {
			  busy_timeout_cnt =0;
			  RADIO_RESET();
			  RADIO_SET_MODE(MODE_STDBY_RC);

			  break;
		  }
	  }

}
void RADIO_RESET(void)
{
	HAL_GPIO_WritePin(LORA_RESET_GPIO_Port,LORA_RESET_Pin, RESET);
	osDelay(2);
	HAL_GPIO_WritePin(LORA_RESET_GPIO_Port,LORA_RESET_Pin, SET);
	osDelay(25);
	RADIO_CHECK_BUSY();

}
void RADIO_SET_MODE(uint8_t modeconfig)
{
	RADIO_CHECK_BUSY();
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t *)&RADIO_SET_STANDBY, 1,100);
	HAL_SPI_Transmit(&hspi3, &modeconfig, 1,100);
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, SET);
}





bool RADIO_START(uint8_t Device)
{
	RADIO_RESET();
	  if (RADIO_CHECK_DEVICE())
	  {
	    return true;
	  }

	  return false;
}

void RADIO_SETUP_LORA(uint32_t frequency, int32_t offset, uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t modParam4)
{
	RADIO_SET_MODE(MODE_STDBY_RC);
	RADIO_SET_REGULATOR_MODE(USE_DCDC);
	RADIO_SET_PA_CONFIG(0x04,PAAUTO,LORA_DEVICE);
	//RADIO_SET_DIO3_TCX0_CTRL(TCXO_CTRL_3_3V);
	RADIO_CALIBRATE_DEVICE(ALLDevices);
	RADIO_CALIBRATE_IMAGE(frequency);
	RADIO_SET_DIO2_RFSWITCH_CTRL();
	RADIO_SET_PACKET_TYPE(PACKET_TYPE_LORA);
	RADIO_SET_RF_FREQUENCY(frequency, offset);
	RADIO_SET_MODULATION_PARAMS(modParam1, modParam2, modParam3, modParam4);
	RADIO_SET_BUFFER_BASE_ADDRESS(0,0);
	RADIO_SET_PACKET_PARAMS(8,LORA_PACKET_VARIABLE_LENGTH,255,LORA_CRC_ON,LORA_IQ_NORMAL);
	RADIO_SET_DIOIrq_PARAMS(IRQ_RADIO_TX, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
	RADIO_SET_HIGH_SENSITIVITY();
	RADIO_SET_SYNC_WORD(LORA_MAC_PRIVATE_SYNCWORD);

}



void RADIO_SET_REGULATOR_MODE(uint8_t mode)
{
	RADIO_WRITE_COMMAND(RADIO_SET_REGULATORMODE, &mode, 1);
}

void RADIO_WRITE_COMMAND(uint8_t Opcode,uint8_t *buffer,uint16_t size)
{
	RADIO_CHECK_BUSY();
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, RESET);
	HAL_SPI_Transmit(&hspi3, (uint8_t *)&Opcode, 1,100);
	HAL_SPI_Transmit(&hspi3, buffer,size,100);
	HAL_GPIO_WritePin(LORA_CS_GPIO_Port,LORA_CS_Pin, SET);
}

void RADIO_SET_PA_CONFIG(uint8_t dutycycle, uint8_t hpMax, uint8_t device)
{
	  uint8_t buffer[4];

	  if (hpMax == PAAUTO)
	  {
	    if (device == 0x01)
	    {
	      hpMax = 0x00;
	    }
	    if (device == 0x00)
	    {
	      hpMax = 0x07;
	    }
	    if (device == 0x02)
	    {
	      hpMax = 0x07;
	    }
	  }

	  if (device == 0x01)
	  {
	    device = 1;
	  }
	  else
	  {
	    device = 0;
	  }

	  buffer[0] = dutycycle;
	  buffer[1] = hpMax;
	  buffer[2] = device;
	  buffer[3] = 0x01;

	  RADIO_WRITE_COMMAND(RADIO_SET_PACONFIG, buffer, 4);
}

void RADIO_SET_DIO3_TCX0_CTRL(uint8_t tcxovoltage)
{
	  uint8_t buffer[4];

	  buffer[0] = tcxovoltage;
	  buffer[1] = 0x00;
	  buffer[2] = 0x00;
	  buffer[3] = 0x64;
      RADIO_WRITE_COMMAND(RADIO_SET_TCXOMODE, buffer, 4);
}

void RADIO_CALIBRATE_DEVICE(uint8_t devices)
{
	RADIO_WRITE_COMMAND(RADIO_CALIBRATE, &devices, 1);
	osDelay(5);
}

void RADIO_SET_DIO2_RFSWITCH_CTRL()
{
	  uint8_t mode = 0x01;
	  RADIO_WRITE_COMMAND(RADIO_SET_RFSWITCHMODE, &mode, 1);
}
void RADIO_CALIBRATE_IMAGE(uint32_t freq)
{
	  uint8_t calFreq[2];

	  if ( freq > 900000000 )
	  {
	    calFreq[0] = 0xE1;
	    calFreq[1] = 0xE9;
	  }
	  else if ( freq > 850000000 )
	  {
	    calFreq[0] = 0xD7;
	    calFreq[1] = 0xD8;
	  }
	  else if ( freq > 770000000 )
	  {
	    calFreq[0] = 0xC1;
	    calFreq[1] = 0xC5;
	  }
	  else if ( freq > 460000000 )
	  {
	    calFreq[0] = 0x75;
	    calFreq[1] = 0x81;
	  }
	  else if ( freq > 425000000 )
	  {
	    calFreq[0] = 0x6B;
	    calFreq[1] = 0x6F;
	  }
	  RADIO_WRITE_COMMAND( RADIO_CALIBRATEIMAGE, calFreq, 2 );
}

void RADIO_SET_PACKET_TYPE(uint8_t packettype)
{

	RADIO_WRITE_COMMAND(RADIO_SET_PACKETTYPE,&packettype,1);
}

void RADIO_SET_RF_FREQUENCY(uint32_t frequency,int32_t offset)
{
	  uint8_t buffer[4];
	  uint32_t localfrequencyRegs;
	  localfrequencyRegs = frequency + offset;
	  localfrequencyRegs = ( uint32_t )( ( double )localfrequencyRegs / ( double )FREQ_STEP );
	  buffer[0] = (localfrequencyRegs >> 24) & 0xFF; //MSB
	  buffer[1] = (localfrequencyRegs >> 16) & 0xFF;
	  buffer[2] = (localfrequencyRegs >> 8) & 0xFF;
	  buffer[3] = localfrequencyRegs & 0xFF;//LSB
	  RADIO_WRITE_COMMAND(RADIO_SET_RFFREQUENCY, buffer, 4);
}

void RADIO_SET_MODULATION_PARAMS(uint8_t modParam1, uint8_t modParam2, uint8_t  modParam3, uint8_t  modParam4)
{
	  uint8_t buffer[4];
	  savedModParam2 = modParam2;
	  if(modParam4 ==  LDRO_AUTO)
	  {
		  modParam4 = RADIO_RETURN_OPTIMISATION(modParam1, modParam2);
	  }
	  buffer[0] = modParam1;
	  buffer[1] = modParam2;
	  buffer[2] = modParam3;
	  buffer[3] = modParam4;
	  RADIO_WRITE_COMMAND(RADIO_SET_MODULATIONPARAMS, buffer, 4);


}

uint8_t RADIO_RETURN_OPTIMISATION(uint8_t SpreadingFactor,uint8_t Bandwidth)
{
	  uint32_t tempBandwidth;
	  float symbolTime;
	  tempBandwidth = RADIO_RETURN_BANDWIDTH(Bandwidth);
	  symbolTime = RADIO_CALC_SYMBOL_TIME(tempBandwidth,SpreadingFactor);
	  if (symbolTime > 16)
	  {
	    return LDRO_ON;
	  }
	  else
	  {
	    return LDRO_OFF;
	  }
}

uint32_t RADIO_RETURN_BANDWIDTH(uint8_t BWregvalue)
{
	  switch (BWregvalue)
	  {
	    case 0:
	      return 7800;

	    case 8:
	      return 10400;

	    case 1:
	      return 15600;

	    case 9:
	      return 20800;

	    case 2:
	      return 31200;

	    case 10:
	      return 41700;

	    case 3:
	      return 62500;

	    case 4:
	      return 125000;

	    case 5:
	      return 250000;

	    case 6:
	      return 500000;

	    default:
	      break;
	  }
	  return 0xFFFF;
}

float RADIO_CALC_SYMBOL_TIME(float Bandwidth,uint8_t SpreadingFactor)
{
	  float symbolTimemS;
	  symbolTimemS = (Bandwidth / pow(2, SpreadingFactor));
	  symbolTimemS = (1000 / symbolTimemS);
	  return symbolTimemS;
}

void RADIO_SET_BUFFER_BASE_ADDRESS(uint8_t txBaseAddress, uint8_t rxBaseAddress)
{
	  uint8_t buffer[2];
	  buffer[0] = txBaseAddress;
	  buffer[1] = rxBaseAddress;
	  RADIO_WRITE_COMMAND(RADIO_SET_BUFFERBASEADDRESS, buffer, 2);
}


void RADIO_SET_PACKET_PARAMS(uint16_t packetParam1, uint8_t  packetParam2, uint8_t packetParam3, uint8_t packetParam4, uint8_t packetParam5)
{
	  uint8_t preambleMSB, preambleLSB;
	  preambleMSB = packetParam1 >> 8;
	  preambleLSB = packetParam1 & 0xFF;
	  uint8_t buffer[9];
	  buffer[0] = preambleMSB;
	  buffer[1] = preambleLSB;
	  buffer[2] = packetParam2;
	  buffer[3] = packetParam3;
	  buffer[4] = packetParam4;
	  buffer[5] = packetParam5;
	  buffer[6] = 0xFF;
	  buffer[7] = 0xFF;
	  buffer[8] = 0xFF;
	  RADIO_WRITE_COMMAND(RADIO_SET_PACKETPARAMS, buffer, 9);
}

void RADIO_SET_DIOIrq_PARAMS(uint16_t irqMask, uint16_t dio1Mask, uint16_t dio2Mask, uint16_t dio3Mask )
{

	  uint8_t buffer[8];

	  buffer[0] = (uint8_t) (irqMask >> 8);
	  buffer[1] = (uint8_t) (irqMask & 0xFF);
	  buffer[2] = (uint8_t) (dio1Mask >> 8);
	  buffer[3] = (uint8_t) (dio1Mask & 0xFF);
	  buffer[4] = (uint8_t) (dio2Mask >> 8);
	  buffer[5] = (uint8_t) (dio2Mask & 0xFF);
	  buffer[6] = (uint8_t) (dio3Mask >> 8);
	  buffer[7] = (uint8_t) (dio3Mask & 0xFF);
	  RADIO_WRITE_COMMAND(RADIO_CFG_DIOIRQ, buffer, 8);
}

void RADIO_SET_HIGH_SENSITIVITY()
{
	RADIO_WRITE_REG(REG_RX_GAIN,BOOSTED_GAIN);
}
void RADIO_SET_SYNC_WORD(uint16_t syncword)
{
	  RADIO_WRITE_REG( REG_LR_SYNCWORD, ( syncword >> 8 ) & 0xFF );
	  RADIO_WRITE_REG( REG_LR_SYNCWORD + 1, syncword & 0xFF );
}

uint8_t RADIO_RECEIVE(uint8_t *rxbuffer, uint8_t size, uint32_t rxtimeout, uint8_t wait)
{
	  uint8_t RXstart, RXend;
	  uint8_t dummy_dataff = 0xFF;
	  uint8_t dummy_data0 = 0x00;
	  uint16_t regdata;
	  uint8_t buffer[2];
	  uint8_t _RXPacketL;
	  //RADIO_RESET();
	  RADIO_SET_DIOIrq_PARAMS(IRQ_RADIO_RX, (IRQ_RX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
	  RADIO_SET_RX(rxtimeout);
	  if(!wait)
	  {
		  return 0;
	  }
	  while(!HAL_GPIO_ReadPin(DIO1_GPIO_Port,DIO1_Pin));
	  RADIO_SET_MODE(MODE_STDBY_RC);
	  regdata = RADIO_READ_IRQ_STATUS();
	  if ( (regdata & IRQ_HEADER_ERROR) | (regdata & IRQ_CRC_ERROR) | (regdata & IRQ_RX_TX_TIMEOUT ) )
	  {
	    //packet is corrupted somewhere so return 0
	    return 0;
	  }
	  RADIO_READ_COMMAND(RADIO_GET_RXBUFFERSTATUS, buffer, 2);
	  _RXPacketL = buffer[0];
	  if(_RXPacketL >size)
	  {
		  _RXPacketL = size;
	  }
	  RXstart = buffer[1];
	  RXend = RXstart + _RXPacketL;
	  RADIO_CHECK_BUSY();
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi3, (uint8_t*)&RADIO_READ_BUFFER, 1, 100);
	  HAL_SPI_Transmit(&hspi3, (uint8_t*)&RXstart, 1, 100);
	  HAL_SPI_Transmit(&hspi3, (uint8_t*)&dummy_dataff,1,100);
	  HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&dummy_data0, (uint8_t*)rxbuffer, RXend, 100);
	  //HAL_SPI_Receive(&hspi3, (uint8_t*)rxbuffer,RXend, 100);
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port,LORA_CS_Pin, SET);
	  return _RXPacketL;
}

void RADIO_SET_RX(uint32_t timeout)
{
	  uint8_t buffer[3];
	  RADIO_CLEAR_IRQ_STATUS(IRQ_RADIO_ALL);
	  timeout = timeout << 6;
	  buffer[0] = (timeout >> 16) & 0xFF;
	  buffer[1] = (timeout >> 8) & 0xFF;
	  buffer[2] = timeout & 0xFF;
	  RADIO_WRITE_COMMAND(RADIO_SETRX, buffer, 3 );
}

void RADIO_CLEAR_IRQ_STATUS(uint16_t irqMask)
{
	  uint8_t buffer[2];
	  buffer[0] = (uint8_t) (irqMask >> 8);
	  buffer[1] = (uint8_t) (irqMask & 0xFF);
	  RADIO_WRITE_COMMAND(RADIO_CLR_IRQSTATUS, buffer, 2);
}

uint16_t RADIO_READ_IRQ_STATUS()
{
	  uint16_t temp;
	  uint8_t buffer[2];
	  RADIO_READ_COMMAND(RADIO_GET_IRQSTATUS, buffer, 2);
	  temp = ((buffer[0] << 8) + buffer[1]);
	  return temp;
}
void RADIO_READ_COMMAND(uint8_t Opcode, uint8_t *buffer, uint16_t size)
{
	  uint8_t dummy_data = 0xFF;
	  RADIO_CHECK_BUSY();
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi3, (uint8_t *)&Opcode, 1,100);
	  HAL_SPI_Transmit(&hspi3, (uint8_t*)&dummy_data,1,100);
	  HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&dummy_data, (uint8_t*)buffer, size, 100);
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port,LORA_CS_Pin, SET);
}

uint8_t RADIO_TRANSMIT(uint8_t *txbuffer, uint8_t size, uint32_t txtimeout, int8_t txpower, uint8_t wait)
{
	  int dummydata =0;
	  if (size == 0)
	  {
	    return false;
	  }
	  RADIO_SET_MODE(MODE_STDBY_RC);
	  RADIO_SET_BUFFER_BASE_ADDRESS(0,0);
	  RADIO_CHECK_BUSY();
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, RESET);
	  HAL_SPI_Transmit(&hspi3, (uint8_t *)&RADIO_WRITE_BUFFER, 1,100);
	  HAL_SPI_Transmit(&hspi3, (uint8_t *)dummydata,1,100);
	  HAL_SPI_Transmit(&hspi3,(uint8_t *)txbuffer,size,100);
	  HAL_GPIO_WritePin(LORA_CS_GPIO_Port,LORA_CS_Pin, SET);
	  RADIO_WRITE_REG(REG_LR_PAYLOADLENGTH, size);
	  RADIO_SET_TX_PARAMS(txpower, RADIO_RAMP_200_US);
	  RADIO_SET_DIOIrq_PARAMS(IRQ_RADIO_TX, (IRQ_TX_DONE + IRQ_RX_TX_TIMEOUT), 0, 0);
	  RADIO_SET_TX(txtimeout);
	  if(!wait)
	  {
		  return size;
	  }
	  while(!HAL_GPIO_ReadPin(DIO1_GPIO_Port,DIO1_Pin));
	  if(RADIO_READ_IRQ_STATUS()& IRQ_RX_TX_TIMEOUT )
	  {
		  return 0;
	  }
	  else
	  {
		  return size;
	  }
}

void RADIO_SET_TX(uint32_t timeout)
{
	  uint8_t buffer[3];
	  RADIO_CLEAR_IRQ_STATUS(IRQ_RADIO_ALL);
	  timeout = timeout << 6;
	  buffer[0] = (timeout >> 16) & 0xFF;
	  buffer[1] = (timeout >> 8) & 0xFF;
	  buffer[2] = timeout & 0xFF;
	  uint8_t regvalue = RADIO_READ_REG(REG_TX_MODULATION);
	  if (savedModParam2 == LORA_BW_500)
	  {
	    RADIO_WRITE_REG(REG_TX_MODULATION, (regvalue & 0xFB));         //if bandwidth is 500k set bit 2 to 0, see datasheet 15.1.1
	  }
	  else
	  {
		  RADIO_WRITE_REG(REG_TX_MODULATION, (regvalue | 0x04));         //if bandwidth is < 500k set bit 2 to 0 see datasheet 15.1.1
	  }

	  RADIO_WRITE_COMMAND(RADIO_SETTX, buffer, 3 );


}

void RADIO_SET_TX_PARAMS(int8_t TXpower, uint8_t RampTime)
{
	  uint8_t buffer[2];
	  buffer[0] = TXpower;
	  buffer[1] = (uint8_t)RampTime;
	  RADIO_WRITE_COMMAND(RADIO_SET_TXPARAMS, buffer, 2);
}
