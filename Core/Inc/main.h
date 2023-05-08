/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define CURRENT_SCK_Pin GPIO_PIN_2
#define CURRENT_SCK_GPIO_Port GPIOE
#define CURRENT_CS_Pin GPIO_PIN_4
#define CURRENT_CS_GPIO_Port GPIOE
#define CURRENT_MISO_Pin GPIO_PIN_5
#define CURRENT_MISO_GPIO_Port GPIOE
#define CURRENT_MOSI_Pin GPIO_PIN_6
#define CURRENT_MOSI_GPIO_Port GPIOE
#define LCD_SCL_Pin GPIO_PIN_1
#define LCD_SCL_GPIO_Port GPIOF
#define IR_TEMP_Pin GPIO_PIN_6
#define IR_TEMP_GPIO_Port GPIOF
#define HS_R_Pin GPIO_PIN_7
#define HS_R_GPIO_Port GPIOF
#define HS_L_Pin GPIO_PIN_8
#define HS_L_GPIO_Port GPIOF
#define AMBIENT_ADC_Pin GPIO_PIN_3
#define AMBIENT_ADC_GPIO_Port GPIOA
#define LORA_CS_Pin GPIO_PIN_4
#define LORA_CS_GPIO_Port GPIOA
#define FLASH_SCK_Pin GPIO_PIN_5
#define FLASH_SCK_GPIO_Port GPIOA
#define FLASH_MISO_Pin GPIO_PIN_6
#define FLASH_MISO_GPIO_Port GPIOA
#define FLASH_MOSI_Pin GPIO_PIN_7
#define FLASH_MOSI_GPIO_Port GPIOA
#define LORA_MOSI_Pin GPIO_PIN_2
#define LORA_MOSI_GPIO_Port GPIOB
#define LORA_TX_SW_Pin GPIO_PIN_15
#define LORA_TX_SW_GPIO_Port GPIOF
#define LORA_BUSY_Pin GPIO_PIN_1
#define LORA_BUSY_GPIO_Port GPIOG
#define LED_RED_Pin GPIO_PIN_10
#define LED_RED_GPIO_Port GPIOE
#define LED_YELLOW_Pin GPIO_PIN_11
#define LED_YELLOW_GPIO_Port GPIOE
#define LED_GREEN_Pin GPIO_PIN_12
#define LED_GREEN_GPIO_Port GPIOE
#define LCD_SDA_Pin GPIO_PIN_11
#define LCD_SDA_GPIO_Port GPIOB
#define SDA_RX_Pin GPIO_PIN_15
#define SDA_RX_GPIO_Port GPIOB
#define LORA_RESET_Pin GPIO_PIN_2
#define LORA_RESET_GPIO_Port GPIOG
#define DIO1_Pin GPIO_PIN_3
#define DIO1_GPIO_Port GPIOG
#define MODEM_DE_Pin GPIO_PIN_8
#define MODEM_DE_GPIO_Port GPIOG
#define MODEM_TX_Pin GPIO_PIN_6
#define MODEM_TX_GPIO_Port GPIOC
#define MODEM_RX_Pin GPIO_PIN_7
#define MODEM_RX_GPIO_Port GPIOC
#define SLAVE_DE_Pin GPIO_PIN_8
#define SLAVE_DE_GPIO_Port GPIOC
#define ESP_TX_Pin GPIO_PIN_11
#define ESP_TX_GPIO_Port GPIOA
#define SDA_DE_Pin GPIO_PIN_12
#define SDA_DE_GPIO_Port GPIOA
#define FLASH_CS_Pin GPIO_PIN_15
#define FLASH_CS_GPIO_Port GPIOA
#define LORA_MISO_Pin GPIO_PIN_11
#define LORA_MISO_GPIO_Port GPIOC
#define SLAVE_TX_Pin GPIO_PIN_12
#define SLAVE_TX_GPIO_Port GPIOC
#define ESP_RX_Pin GPIO_PIN_1
#define ESP_RX_GPIO_Port GPIOD
#define SLAVE_RX_Pin GPIO_PIN_2
#define SLAVE_RX_GPIO_Port GPIOD
#define LORA_SCK_Pin GPIO_PIN_3
#define LORA_SCK_GPIO_Port GPIOB
#define SDA_TX_Pin GPIO_PIN_6
#define SDA_TX_GPIO_Port GPIOB
#define LED_Pin GPIO_PIN_7
#define LED_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */
#define LORA_DEVICE				0x00
#define MODE_STDBY_RC			0x00
#define	USE_DCDC				0x01
#define PAAUTO					0xFF
#define LORA_DEVICE				0x00
#define TCXO_CTRL_3_3V			0x07
#define ALLDevices				0x7F
#define PACKET_TYPE_LORA		0x01
#define LORA_PACKET_VARIABLE_LENGTH	0x00
#define LORA_CRC_ON				0x01
#define LORA_IQ_NORMAL			0x00
#define IRQ_RADIO_ALL			0xFFFF
#define IRQ_TX_DONE				0x0001
#define IRQ_RX_TX_TIMEOUT		0x0200
#define LORA_MAC_PRIVATE_SYNCWORD	0x1424
#define RADIO_SET_REGULATORMODE	0x96
#define RADIO_SET_PACONFIG		0x95
#define RADIO_SET_TCXOMODE		0x97
#define RADIO_CALIBRATE			0x89
#define RADIO_CALIBRATEIMAGE	0x98
#define RADIO_SET_RFSWITCHMODE	0x9D
#define RADIO_SET_PACKETTYPE	0x8A
#define RADIO_SET_RFFREQUENCY	0x86
#define LDRO_AUTO				0x02
#define LDRO_ON					0x01
#define LDRO_OFF				0x00
#define RADIO_SET_MODULATIONPARAMS	0x8B
#define LORA_SF11				0x0B
#define LORA_BW_125 			4
#define LORA_CR_4_6				0x02
#define RADIO_SET_BUFFERBASEADDRESS	0x8F
#define RADIO_SET_PACKETPARAMS	0x8C
#define RADIO_CFG_DIOIRQ		0x08
#define REG_RX_GAIN				0x08AC
#define BOOSTED_GAIN			0x96
#define REG_LR_SYNCWORD			0x0740
#define WAIT_RX					0x01
#define IRQ_RX_DONE				0x0002
#define RADIO_CLR_IRQSTATUS		0x02
#define RADIO_SETRX				0x82
#define RADIO_GET_IRQSTATUS		0x12
#define IRQ_HEADER_ERROR		0x0020
#define IRQ_CRC_ERROR			0x0040
#define RADIO_GET_RXBUFFERSTATUS	0x13
#define WAIT_TX					0x01
#define TXPower					22
#define REG_LR_PAYLOADLENGTH	0x0702
#define RADIO_RAMP_200_US		0x04
#define REG_TX_MODULATION		0x0889
#define RADIO_SETTX				0x83
#define RADIO_SET_TXPARAMS		0x8E
#define LORA_BW_500				6
#define IRQ_RADIO_TX			0x0201
#define IRQ_RADIO_RX			0xFFFE
#define    LORA_SF5                                 0x05
#define    LORA_SF6                                 0x06
#define    LORA_SF7                                 0x07
#define    LORA_SF8                                 0x08
#define    LORA_SF9                                 0x09
#define    LORA_SF10                                0x0A
#define    LORA_SF11                                0x0B
#define    LORA_SF12                                0x0C

#define    LORA_BW_500                              6      //actual 500000hz
#define    LORA_BW_250                              5      //actual 250000hz
#define    LORA_BW_125                              4      //actual 125000hz
#define    LORA_BW_062                              3      //actual  62500hz
#define    LORA_BW_041                              10     //actual  41670hz
#define    LORA_BW_031                              2      //actual  31250hz
#define    LORA_BW_020                              9      //actual  20830hz
#define    LORA_BW_015                              1      //actual  15630hz
#define    LORA_BW_010                              8      //actual  10420hz
#define    LORA_BW_007                              0      //actual   7810hz

#define    LORA_CR_4_5                              0x01
#define    LORA_CR_4_6                              0x02
#define    LORA_CR_4_7                              0x03
#define    LORA_CR_4_8                              0x04

#define LORA_BIT				0x01
#define MODEM_BIT				0x01
#define	MASTER_BIT				0x02
#define	SLAVE_BIT				0x03
#define	MODEM_REPLY_BIT			0x04
#define SEARCHING_DEVICES 		0x09	//ID TO DETRMINE THAT THAT RCU HAS ASKED FOR NO OF TCU AVAILABLE WITH THEIR ID'S
#define SEARCH_IDs 				0x0A	//ID TO SAY THAT DATA IS ID'S OF TCU'S
#define MASTER_SLAVE_CONFIG		0xBB	//ID for MASTER SLAVE CONFIG RECEIVED
#define I_AM_MASTER				0x0A
#define I_AM_SLAVE				0x0B
#define ALL_OK					0xA0
#define YES_OK					0xA1
#define SEARCH_IDs 				0x0A	// ID'S OF TCU'S
#define DATA_SIZE_FOR_4SYS		0xC8
#define	DATA_SIZE_FOR_3SYS		0x96
#define	DATA_SIZE_FOR_2SYS		0x64
#define	DATA_SIZE_FOR_1SYS		0x32
#define SIZE_UPDATE				0xA2
#define CLOSE_LOOP_START 		0x07
#define CLOSE_LOOP_START_PROFILE 0x77
#define SEND_DATA_TO_SLAVE		0x10
#define PROCESS_DATA			0x11
#define UPADTE_TEMP_OR_POWER 	0x05
#define OPEN_LOOP_START 		0x0D
#define OPEN_LOOP_START_PROFILE 0xDD
#define SLEEP					0xF0
#define KP_KI_KD_DATA			0x2C
#define CLOSE_LOOP				0x01
#define OPEN_LOOP				0x02
#define MODEM_SYSTEM			0x01
#define LOCAL_SYSTEM			0x02
#define WIFI_SYSTEM				0x03
#define MAS_SLAVE_SYSTEM		0x04
#define I_AM_PID				0x05
#define I_AM_DATA_LOGG			0x06
#define I_AM_FOR_MASTER			0x07
#define SDA_TEMPERATURE			0xE1
#define GIVE_DATA				0xCA
#define DATA_TYPE_CURRENT		0x0C
#define SYS_VOLTAGE				0x2B
#define WHOLE_CURRENT			0x2A
#define ACK_NACK_DATA			0x9A
#define ACK						0x61
#define NACK					0x6E
#define CURRENT_DATA			0x0C
#define AMB_TEMP_t				0xFA
#define IR_TEMP_t				0xFB
#define HS_LEFT_t				0xFC
#define HS_RIGHT_t				0xFD
#define RSSI					0x31
#define	ERRORS					0x32
#define GIVE_CURRENT			0xCB
#define ERROR_NO_SDA					0xE2
#define CURRENT_LIMIT			0xCF
#define SDA_CALLIBRATION		0xCE
#define FIRST_TIME				0x00
#define RUNNING					0x01
#define RUNNING_SEARCH			0xAA
#define RESET_MAS_SLAVE			0xED
#define UPDATE_PROFILE			0x55
#define SYSTEM_STATE			0xC6


#define LOGGING_INTERVAL		0xBF
#define FETCH_ENNG_MODE			0xBE
#define FETCH_DATA_REC			0xB0
#define LORA_PARAMETERS			0xBA
#define FETCH_BIST					0xB9
#define SDA_CALLIBRATION_FACTOR		0xB8
#define ENGG_MODE_DATA				0xB7
#define FETCH_DATA					0xB4
#define CLEAR_FLASH					0x4B
#define SDA_ERROR_DATA_t			0xB3
#define TCU_ERROR_DATA_t			0xB2
#define RADIO_ERROR_DATA_t			0xB1

#define STARTING_ADD_OF_ENGG_MODE			0x00001000
#define ENDING_ADD_OF_ENGG_MODE				0x00004FFF

#define STARTING_ADD_OF_PROFILES			0x00006000
#define ENDING_ADD_OF_PROFILES				0x00010000

#define STARTING_ADD_REC_DATA				0x00020000
#define	ENDING_ADD_REC_DATA					0x01FFFFFF

#define ADC_12_CH_DATA				0xC0
#define ADC_13_CH_DATA				0xD0
#define ADC_14_CH_DATA				0xE0
#define ADC_15_CH_DATA				0xF0

#define START_STOP_STATE			0x9A
#define I_AM_WIFI					0x99

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
