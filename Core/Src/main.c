/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "stdio.h"
#include "String.h"
#include "stdbool.h"
#include "LORA.h"
#include "PCA9685.h"
#include "i2c-lcd.h"
#include "FLASH.h"
#include "NEW_LCD.h"
#include "math.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */


#define MY_TCU_ID				0x04
#define MAX_BUFFER_SIZE			0xC8
#define BUFFER_SIZE_ALL			0xC8

typedef struct
{
uint8_t LORA_SEND_DATA[MAX_BUFFER_SIZE];
} LORA_SEND;
LORA_SEND *lora_send;
osPoolDef(lorasendpool, 2, LORA_SEND);
osPoolId  lorasendpool;

typedef struct
{
uint8_t FLASH_DATA_t[MAX_BUFFER_SIZE];
} FLASH_DATA;
FLASH_DATA *flash_data;
osPoolDef(flashpool, 2, FLASH_DATA);
osPoolId  flashpool;

typedef struct
{
uint8_t FLASH_DATA_LOGG[MAX_BUFFER_SIZE];
} FLASH_LOGGING;
FLASH_LOGGING *flash_logging;
osPoolDef(flashlogpool, 2, FLASH_LOGGING);
osPoolId  flashlogpool;

typedef struct
{
uint8_t SLAVE_ID[20];
} SLAVE_ID;
SLAVE_ID *slave_id;

osPoolDef(slaveidpool, 2, SLAVE_ID);
osPoolId  slaveidpool;



typedef struct
{
uint8_t DATA_TO_PROCESS[50];
} SYSTEM_PROCESS;
SYSTEM_PROCESS *system_process;

osPoolDef(syspropool, 2, SYSTEM_PROCESS);
osPoolId  syspropool;





typedef struct
{
uint8_t DATA_FROM_SLAVE_t[50];
} DATA_OF_SLAVE;
DATA_OF_SLAVE *data_of_slave;

osPoolDef(dataofslavepool, 2, DATA_OF_SLAVE);
osPoolId  dataofslavepool;

typedef struct
{
uint8_t SYSTEM_VOLTAGE_t;
uint8_t SYSTEM_CURRENT_t;
uint8_t AMBIENT_TEMPERATURE_t;
uint8_t IR_TEMPERATURE_t;
uint8_t LEFT_HS_TEMPERATURE_t;
uint8_t RIGHT_HS_TEMPERATURE_t;
uint8_t RSSI_t;
uint8_t ERROR_t;
} DATA_ACCU;
DATA_ACCU *data_accu;
osPoolDef(dataaccupool,10,DATA_ACCU);
osPoolId dataaccupool;

typedef struct
{
uint8_t SLAVE_DATA[50];
} SLAVE_SEND;
SLAVE_SEND *slave_send;

osPoolDef(slavesendpool, 2, SLAVE_SEND);
osPoolId  slavesendpool;

typedef struct
{
uint8_t DATA_LOGGING_DATA[50];
} DATA_LOGGING;
DATA_LOGGING *data_logging;

osPoolDef(datalogpool, 2, DATA_LOGGING);
osPoolId  datalogpool;

typedef struct
{
uint8_t CLOSE_CURRENT_DATA[50];
} CLOSE_CURRENT;
CLOSE_CURRENT *close_curent;

osMailQDef (closecurrmail,2,CLOSE_CURRENT);
osMailQId closecurrmail;


typedef struct
{
uint8_t TEMP_DATA_FOR_WIFI[50];
} WIFI_TEMP;
WIFI_TEMP *wifi_temp;
osMailQDef (wifitempmail,2,WIFI_TEMP);
osMailQId wifitempmail;



typedef struct
{
uint8_t CURR_DATA_FOR_WIFI[50];
} WIFI_CURR;
WIFI_CURR *wifi_curr;


osMailQDef (wificurrmail,2,WIFI_CURR);
osMailQId wificurrmail;

typedef struct
{
uint8_t SDA_SEND_DATA[5];
} SDA_SEND;
SDA_SEND *sda_send;
osPoolDef(sdasendpool, 2, SDA_SEND);
osPoolId  sdasendpool;

typedef struct
{
uint8_t FLASH_READ_DATA[5];
} FLASH_READ;
FLASH_READ *flash_read;
osPoolDef(flashreadpool, 2, FLASH_READ);
osPoolId  flashreadpool;

typedef struct
{
uint8_t LCD_DATA[2];
} LCD_QUEUE;
LCD_QUEUE *lcd_queue;
osPoolDef(lcdpool, 7, LCD_QUEUE);
osPoolId  lcdpool;

typedef struct
{
uint8_t SDA_CONN_DATA[2];
} SDA_CONN;
SDA_CONN *sda_conn;
osPoolDef(sdaconnpool, 2, SDA_CONN);
osPoolId  sdaconnpool;

typedef struct
{
uint8_t GIVE_ME_t;
} GIVE_ME;
GIVE_ME *give_me;
osPoolDef(givepool, 5, GIVE_ME);
osPoolId  givepool;


typedef struct
{
uint8_t CURRENT_LIMIT_DATA;
int WHOLE_CURRENT_DATA;
} CURRENT_LIMIT_Q;
CURRENT_LIMIT_Q *current_limit_q;
osPoolDef(currlimitpool, 2, CURRENT_LIMIT_Q);
osPoolId  currlimitpool;



typedef struct
{
uint8_t error_data_rec[2];
} ERROR_D;
ERROR_D *error_d;
osPoolDef(errorpool, 2, ERROR_D);
osPoolId  errorpool;

typedef struct
{
uint8_t NOTIFY;
} DATA_LOG;
DATA_LOG *data_log;
osPoolDef(dlpool, 2, DATA_LOG);
osPoolId  dlpool;

typedef struct
{
uint16_t DATA_PROCESS_DATA[16];
} DATA_PROCESS;
DATA_PROCESS *data_process;
osPoolDef(datappool, 2, DATA_PROCESS);
osPoolId  datappool;

typedef struct
{
uint16_t PID_DATA_t[16];
} PID_DATA;
PID_DATA *pid_data;
osPoolDef(pidpool, 2, PID_DATA);
osPoolId  pidpool;


typedef struct
{
uint16_t CURRENT_P_DATA[40];
} CURRENT_P;
CURRENT_P *current_p;
osPoolDef(currppool, 2, CURRENT_P);
osPoolId  currppool;

typedef struct
{
uint16_t SETPOINT_t[16];
} SETPOINT;
SETPOINT *setpoint;
osPoolDef(sppool, 2, SETPOINT);
osPoolId  sppool;

typedef struct
{
uint16_t DUTY_t[16];
} DUTY_Q;
DUTY_Q *duty_q;
osPoolDef(dutypool, 2, DUTY_Q);
osPoolId  dutypool;

typedef struct
{
uint16_t FEEDBACK_t[16];
} FEEDBACK;
FEEDBACK *feedback;
osPoolDef(fbpool, 2, FEEDBACK);
osPoolId  fbpool;

typedef struct
{
uint8_t KP_KI_KD_t[4];
} KP_KI_KD;
KP_KI_KD *kp_ki_kd;
osPoolDef(kpkikdpool, 2, KP_KI_KD);
osPoolId  kpkikdpool;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */




/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;
ADC_HandleTypeDef hadc3;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

IWDG_HandleTypeDef hiwdg;

RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi3;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart3;
UART_HandleTypeDef huart6;
DMA_HandleTypeDef hdma_uart4_rx;
DMA_HandleTypeDef hdma_uart5_rx;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart6_rx;

osThreadId defaultTaskHandle;
osThreadId LORA_RECEIVEHandle;
osThreadId LORA_SENDHandle;
osThreadId MAS_SEND_SLAVEHandle;
osThreadId SLAVE_REC_MASHandle;
osThreadId SDA_SENDHandle;
osThreadId SDA_RECEIVEHandle;
osThreadId SYSTEM_PROCESSHandle;
osThreadId DATA_PROCESSHandle;
osThreadId RCU_RECEIVEHandle;
osThreadId WIFI_RECEIVEHandle;
osThreadId CURRENT_TASKHandle;
osThreadId PID_TASKHandle;
osThreadId VOLTAGE_TASKHandle;
osThreadId WHOLE_CURRENTHandle;
osThreadId ADC_TASKHandle;
osThreadId FLASH_WRITEHandle;
osThreadId POWER_ON_TESTHandle;
osThreadId LCD_TASKHandle;
osThreadId DATA_LOGGINGHandle;
osThreadId CURRENT_PROCESSHandle;
osThreadId PID_REGULATIONHandle;
osThreadId DATA_LOG_FLASHHandle;
osThreadId FLASH_READHandle;
osThreadId WIFI_SENDHandle;
osThreadId PROFILE_STARTHandle;
osMessageQId LORA_SEND_qHandle;
osMessageQId SLAVE_ID_qHandle;
osMessageQId SDA_SEND_qHandle;
osMessageQId SYSTEM_PROCESS_qHandle;
osMessageQId DATA_PROCESS_qHandle;
osMessageQId SLAVE_SEND_qHandle;
osMessageQId SETPOINT_qHandle;
osMessageQId KP_KI_KD_qHandle;
osMessageQId GIVE_ME_qHandle;
osMessageQId FEEDBACK_qHandle;
osMessageQId DATA_LOGGING_qHandle;
osMessageQId DATA_OF_SLAVE_qHandle;
osMessageQId LCD_QUEUE_qHandle;
osMessageQId DATA_ACCU_qHandle;
osMessageQId CLOSE_CURRENT_qHandle;
osMessageQId DATA_LOG_qHandle;
osMessageQId ERROR_D_qHandle;
osMessageQId DUTY_Q_qHandle;
osMessageQId CURRENT_P_qHandle;
osMessageQId PID_DATA_qHandle;
osMessageQId CURRENT_LIMIT_Q_qHandle;
osMessageQId FLASH_DATAHandle;
osMessageQId FLASH_LOGGINGHandle;
osMessageQId FLASH_READ_qHandle;
osMessageQId WIFI_TEMPHandle;
osMessageQId WIFI_CURRHandle;
osMessageQId SDA_CONN_qHandle;
osTimerId TCU_SDA_TIMERHandle;
osTimerId myTimer02Handle;
/* USER CODE BEGIN PV */
uint8_t UART_RX_DATA[50];
static uint8_t I_AM = I_AM_SLAVE;
static uint8_t LORA_RX_BUFF_SIZE	=	0x32;
static uint8_t TEMP_SIZE_UPDATE;
static uint8_t START_CONDITION =0;
static uint8_t MODE_CONDITION = 0;
static uint8_t NUMBER_OF_SYSTEMS=0;
uint8_t DATA_FROM_MAS_SLAVE[50];
char DATA_FROM_SDA[34];
uint8_t DATA_FROM_WIFI[50];
uint8_t DATA_FROM_LOCAL[50];
static uint8_t ID_S_PRESENT[3] ={0};
static int LOOP_RUNNING = 0x00;
static int WHOM_TO_GIVE_DATA = 0x00;
static int NO_DUTY = 0;
static float DUTY[16]={0};
static int log_interval =1000;
static uint8_t SDA_ERROR_DATA =1;
static uint8_t LORA_ERROR_DATA =1;
static uint8_t TCU_ERROR_DATA_FLASH =1;
static uint8_t TCU_ERROR_DATA_PWM = 1;
static int Running_Status = FIRST_TIME;
static uint8_t BIST_DATA[10]={0};
static uint8_t PROFILES[50][50]={0};
static uint8_t PROFILE_COUNT =0;
TimerHandle_t sda_tcu_timer;
static int current_limit_value = 1;
static float KP[16],KI[16],KD[16];
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART3_UART_Init(void);
static void MX_CRC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART6_UART_Init(void);
static void MX_UART4_Init(void);
static void MX_I2C1_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC2_Init(void);
static void MX_ADC1_Init(void);
static void MX_ADC3_Init(void);
static void MX_I2C2_Init(void);
static void MX_RTC_Init(void);
static void MX_SPI1_Init(void);
static void MX_SPI4_Init(void);
static void MX_SPI3_Init(void);
static void MX_UART5_Init(void);
static void MX_IWDG_Init(void);
void StartDefaultTask(void const * argument);
void Start_LORA_RECEIVE(void const * argument);
void Start_LORA_SEND(void const * argument);
void Start_MAS_SEND_SLAVE(void const * argument);
void Start_SLAVE_REC_MAS(void const * argument);
void Start_SDA_SEND(void const * argument);
void Start_SDA_RECEIVE(void const * argument);
void Start_SYSTEM_PROCESS(void const * argument);
void Start_DATA_PROCESS(void const * argument);
void Start_RCU_RECEIVE(void const * argument);
void Start_WIFI_RECEIVE(void const * argument);
void Start_CURRENT_TASK(void const * argument);
void Start_PID_TASK(void const * argument);
void Start_VOLTAGE_TASK(void const * argument);
void Start_WHOLE_CURRENT(void const * argument);
void Start_ADC_TASK(void const * argument);
void Start_FLASH_WRITE(void const * argument);
void Start_POWER_ON_TEST(void const * argument);
void Start_LCD_TASK(void const * argument);
void Start_DATA_LOGGING(void const * argument);
void Start_CURRENT_PROCESS(void const * argument);
void Start_PID_REGULATION(void const * argument);
void Start_DATA_LOG_FLASH(void const * argument);
void Start_FLASH_READ(void const * argument);
void Start_WIFI_SEND(void const * argument);
void Start_PROFILE_START(void const * argument);
void TCU_SDA_TIMER_CALL(void const * argument);
void Callback02(void const * argument);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

#ifdef __GNUC__
/* With GCC, small printf (option LD Linker->Libraries->Small printf
   set to 'Yes') calls __io_putchar() */
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif /* __GNUC__ */

PUTCHAR_PROTOTYPE
{
  /* Place your implementation of fputc here */
  /* e.g. write a character to the USART3 and Loop until the end of transmission */
  HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);

  return ch;

}

float map(float x, float in_min, float in_max, float out_min, float out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void IR_TEMPERATURE(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_4;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
{
  Error_Handler();
}
}

void AMB_TEMPERATURE(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_3;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
{
  Error_Handler();
}
}

void LHS_TEMPERATURE(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_5;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
{
  Error_Handler();
}
}

void RHS_TEMPERATURE(void)
{
	ADC_ChannelConfTypeDef sConfig = {0};
sConfig.Channel = ADC_CHANNEL_6;
sConfig.Rank = 1;
sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
{
  Error_Handler();
}
}


void SEND_ACK(void)
{
	uint8_t DATA_FOR_RCU[MAX_BUFFER_SIZE];
	uint8_t CRC_VALUE_OF_DATA_FOR_RCU;
	LORA_SEND *lorasendmsg;
	  memset(DATA_FOR_RCU,0xFF,MAX_BUFFER_SIZE);
	  DATA_FOR_RCU[0] = LORA_BIT;
	  DATA_FOR_RCU[1]=MODEM_REPLY_BIT;
	  DATA_FOR_RCU[2]=MY_TCU_ID;
	  DATA_FOR_RCU[3]=ACK_NACK_DATA;
	  DATA_FOR_RCU[4]=ACK;
	  CRC_VALUE_OF_DATA_FOR_RCU = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_RCU,(BUFFER_SIZE_ALL-1));
	  DATA_FOR_RCU[BUFFER_SIZE_ALL-1]=CRC_VALUE_OF_DATA_FOR_RCU;
	  lorasendmsg = osPoolAlloc(lorasendpool);
	  memcpy(lorasendmsg->LORA_SEND_DATA,DATA_FOR_RCU,200);
	  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
	  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
	  HAL_UART_Transmit(&huart4, DATA_FOR_RCU, 50,100);
}

void SEND_NACK(void)
{
	uint8_t DATA_FOR_RCU[MAX_BUFFER_SIZE];
	uint8_t CRC_VALUE_OF_DATA_FOR_RCU;
	LORA_SEND *lorasendmsg;
	  memset(DATA_FOR_RCU,0xFF,MAX_BUFFER_SIZE);
	  DATA_FOR_RCU[0]=MODEM_REPLY_BIT;
	  DATA_FOR_RCU[1]=MY_TCU_ID;
	  DATA_FOR_RCU[3]=ACK_NACK_DATA;
	  DATA_FOR_RCU[4]=NACK;
	  CRC_VALUE_OF_DATA_FOR_RCU = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_RCU,(BUFFER_SIZE_ALL-1));
	  DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)]=CRC_VALUE_OF_DATA_FOR_RCU;
	  lorasendmsg = osPoolAlloc(lorasendpool);
	  memcpy(lorasendmsg->LORA_SEND_DATA,DATA_FOR_RCU,200);
	  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
	  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
	  HAL_UART_Transmit(&huart4, DATA_FOR_RCU, 50,100);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if((huart->Instance == UART5))
	{
		HAL_UART_Receive_DMA(&huart5, DATA_FROM_MAS_SLAVE, 50);
		__HAL_DMA_DISABLE_IT(&hdma_uart5_rx,DMA_IT_HT);
		vTaskNotifyGiveFromISR(SLAVE_REC_MASHandle, pdFALSE);
	}
	if((huart->Instance == UART4))
	{
		HAL_UART_Receive_DMA(&huart4, DATA_FROM_WIFI, 50);
		__HAL_DMA_DISABLE_IT(&hdma_uart4_rx,DMA_IT_HT);
		vTaskNotifyGiveFromISR(WIFI_RECEIVEHandle, pdFALSE);
	}
	if((huart->Instance == USART6))
	{
		HAL_UART_Receive_DMA(&huart6, DATA_FROM_LOCAL, 50);
		__HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_HT);
		vTaskNotifyGiveFromISR(RCU_RECEIVEHandle, pdFALSE);
	}
	if((huart->Instance == USART1))
	{
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)DATA_FROM_SDA, 34);
		__HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
		vTaskNotifyGiveFromISR(SDA_RECEIVEHandle, pdFALSE);
		HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
__disable_irq();
  /* USER CODE END 1 */

  /* Enable I-Cache---------------------------------------------------------*/
  SCB_EnableICache();

  /* Enable D-Cache---------------------------------------------------------*/
  SCB_EnableDCache();

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART3_UART_Init();
  MX_CRC_Init();
  MX_USART1_UART_Init();
  MX_USART6_UART_Init();
  MX_UART4_Init();
  MX_I2C1_Init();
  MX_TIM4_Init();
  MX_ADC2_Init();
  MX_ADC1_Init();
  MX_ADC3_Init();
  MX_I2C2_Init();
  MX_RTC_Init();
  MX_SPI1_Init();
  MX_SPI4_Init();
  MX_SPI3_Init();
  MX_UART5_Init();
  MX_IWDG_Init();
  /* USER CODE BEGIN 2 */
  HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin,SET);
  /* USER CODE END 2 */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* Create the timer(s) */
  /* definition and creation of TCU_SDA_TIMER */
  osTimerDef(TCU_SDA_TIMER, TCU_SDA_TIMER_CALL);
  TCU_SDA_TIMERHandle = osTimerCreate(osTimer(TCU_SDA_TIMER), osTimerOnce, NULL);

  /* definition and creation of myTimer02 */
  osTimerDef(myTimer02, Callback02);
  myTimer02Handle = osTimerCreate(osTimer(myTimer02), osTimerOnce, NULL);

  /* USER CODE BEGIN RTOS_TIMERS */
  //sda_tcu_timer = xTimerCreate( "SDA_TIMER",  pdMS_TO_TICKS(1000), pdFALSE,	(void*)0,	vTimerCallbacksda);
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* Create the queue(s) */
  /* definition and creation of LORA_SEND_q */
  osMessageQDef(LORA_SEND_q, 2, lora_send);
  LORA_SEND_qHandle = osMessageCreate(osMessageQ(LORA_SEND_q), NULL);

  /* definition and creation of SLAVE_ID_q */
  osMessageQDef(SLAVE_ID_q, 2, slave_id);
  SLAVE_ID_qHandle = osMessageCreate(osMessageQ(SLAVE_ID_q), NULL);

  /* definition and creation of SDA_SEND_q */
  osMessageQDef(SDA_SEND_q, 2, sda_send);
  SDA_SEND_qHandle = osMessageCreate(osMessageQ(SDA_SEND_q), NULL);

  /* definition and creation of SYSTEM_PROCESS_q */
  osMessageQDef(SYSTEM_PROCESS_q, 2, system_process);
  SYSTEM_PROCESS_qHandle = osMessageCreate(osMessageQ(SYSTEM_PROCESS_q), NULL);

  /* definition and creation of DATA_PROCESS_q */
  osMessageQDef(DATA_PROCESS_q, 2, data_process);
  DATA_PROCESS_qHandle = osMessageCreate(osMessageQ(DATA_PROCESS_q), NULL);

  /* definition and creation of SLAVE_SEND_q */
  osMessageQDef(SLAVE_SEND_q, 2, slave_send);
  SLAVE_SEND_qHandle = osMessageCreate(osMessageQ(SLAVE_SEND_q), NULL);

  /* definition and creation of SETPOINT_q */
  osMessageQDef(SETPOINT_q, 2, setpoint);
  SETPOINT_qHandle = osMessageCreate(osMessageQ(SETPOINT_q), NULL);

  /* definition and creation of KP_KI_KD_q */
  osMessageQDef(KP_KI_KD_q, 2, kp_ki_kd);
  KP_KI_KD_qHandle = osMessageCreate(osMessageQ(KP_KI_KD_q), NULL);

  /* definition and creation of GIVE_ME_q */
  osMessageQDef(GIVE_ME_q, 5, give_me);
  GIVE_ME_qHandle = osMessageCreate(osMessageQ(GIVE_ME_q), NULL);

  /* definition and creation of FEEDBACK_q */
  osMessageQDef(FEEDBACK_q, 2, feedback);
  FEEDBACK_qHandle = osMessageCreate(osMessageQ(FEEDBACK_q), NULL);

  /* definition and creation of DATA_LOGGING_q */
  osMessageQDef(DATA_LOGGING_q, 4, data_logging);
  DATA_LOGGING_qHandle = osMessageCreate(osMessageQ(DATA_LOGGING_q), NULL);

  /* definition and creation of DATA_OF_SLAVE_q */
  osMessageQDef(DATA_OF_SLAVE_q, 4, data_of_slave);
  DATA_OF_SLAVE_qHandle = osMessageCreate(osMessageQ(DATA_OF_SLAVE_q), NULL);

  /* definition and creation of LCD_QUEUE_q */
  osMessageQDef(LCD_QUEUE_q, 7, lcd_queue);
  LCD_QUEUE_qHandle = osMessageCreate(osMessageQ(LCD_QUEUE_q), NULL);

  /* definition and creation of DATA_ACCU_q */
  osMessageQDef(DATA_ACCU_q, 10, data_accu);
  DATA_ACCU_qHandle = osMessageCreate(osMessageQ(DATA_ACCU_q), NULL);

  /* definition and creation of CLOSE_CURRENT_q */
  osMessageQDef(CLOSE_CURRENT_q, 2, close_curent);
  CLOSE_CURRENT_qHandle = osMessageCreate(osMessageQ(CLOSE_CURRENT_q), NULL);

  /* definition and creation of DATA_LOG_q */
  osMessageQDef(DATA_LOG_q, 2, data_log);
  DATA_LOG_qHandle = osMessageCreate(osMessageQ(DATA_LOG_q), NULL);

  /* definition and creation of ERROR_D_q */
  osMessageQDef(ERROR_D_q, 2, error_d);
  ERROR_D_qHandle = osMessageCreate(osMessageQ(ERROR_D_q), NULL);

  /* definition and creation of DUTY_Q_q */
  osMessageQDef(DUTY_Q_q, 2, duty_q);
  DUTY_Q_qHandle = osMessageCreate(osMessageQ(DUTY_Q_q), NULL);

  /* definition and creation of CURRENT_P_q */
  osMessageQDef(CURRENT_P_q, 2, current_p);
  CURRENT_P_qHandle = osMessageCreate(osMessageQ(CURRENT_P_q), NULL);

  /* definition and creation of PID_DATA_q */
  osMessageQDef(PID_DATA_q, 2, pid_data);
  PID_DATA_qHandle = osMessageCreate(osMessageQ(PID_DATA_q), NULL);

  /* definition and creation of CURRENT_LIMIT_Q_q */
  osMessageQDef(CURRENT_LIMIT_Q_q, 2, current_limit_q);
  CURRENT_LIMIT_Q_qHandle = osMessageCreate(osMessageQ(CURRENT_LIMIT_Q_q), NULL);

  /* definition and creation of FLASH_DATA */
  osMessageQDef(FLASH_DATA, 2, flash_data);
  FLASH_DATAHandle = osMessageCreate(osMessageQ(FLASH_DATA), NULL);

  /* definition and creation of FLASH_LOGGING */
  osMessageQDef(FLASH_LOGGING, 1, flash_logging);
  FLASH_LOGGINGHandle = osMessageCreate(osMessageQ(FLASH_LOGGING), NULL);

  /* definition and creation of FLASH_READ_q */
  osMessageQDef(FLASH_READ_q, 2, flash_read);
  FLASH_READ_qHandle = osMessageCreate(osMessageQ(FLASH_READ_q), NULL);

  /* definition and creation of WIFI_TEMP */
  osMessageQDef(WIFI_TEMP, 2, wifi_temp);
  WIFI_TEMPHandle = osMessageCreate(osMessageQ(WIFI_TEMP), NULL);

  /* definition and creation of WIFI_CURR */
  osMessageQDef(WIFI_CURR, 2, wifi_curr);
  WIFI_CURRHandle = osMessageCreate(osMessageQ(WIFI_CURR), NULL);

  /* definition and creation of SDA_CONN_q */
  osMessageQDef(SDA_CONN_q, 2, sda_conn);
  SDA_CONN_qHandle = osMessageCreate(osMessageQ(SDA_CONN_q), NULL);

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  slaveidpool = osPoolCreate(osPool(slaveidpool));//POOL FOR ID.
  sdasendpool = osPoolCreate(osPool(sdasendpool));
  syspropool = osPoolCreate(osPool(syspropool));
  datappool = osPoolCreate(osPool(datappool));
  slavesendpool = osPoolCreate(osPool(slavesendpool));
  sppool = osPoolCreate(osPool(sppool));
  dlpool = osPoolCreate(osPool(dlpool));
  currppool = osPoolCreate(osPool(currppool));
  flashpool = osPoolCreate(osPool(flashpool));
  flashreadpool = osPoolCreate(osPool(flashreadpool));
  flashlogpool = osPoolCreate(osPool(flashlogpool));
  currlimitpool = osPoolCreate(osPool(currlimitpool));
  closecurrmail = osMailCreate(osMailQ(closecurrmail), NULL);
  fbpool = osPoolCreate(osPool(fbpool));
  wifitempmail = osMailCreate(osMailQ(wifitempmail),NULL);
  wificurrmail = osMailCreate(osMailQ(wificurrmail),NULL);
  dutypool = osPoolCreate(osPool(dutypool));
  lcdpool = osPoolCreate(osPool(lcdpool));
  sdaconnpool = osPoolCreate(osPool(sdaconnpool));
  pidpool = osPoolCreate(osPool(pidpool));
  errorpool = osPoolCreate(osPool(errorpool));
  givepool = osPoolCreate(osPool(givepool));
  dataaccupool = osPoolCreate(osPool(dataaccupool));
  datalogpool = osPoolCreate(osPool(datalogpool));
  dataofslavepool = osPoolCreate(osPool(dataofslavepool));
  kpkikdpool = osPoolCreate(osPool(kpkikdpool));
  lorasendpool = osPoolCreate(osPool(lorasendpool));

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 2000);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of LORA_RECEIVE */
  osThreadDef(LORA_RECEIVE, Start_LORA_RECEIVE, osPriorityNormal, 0, 2000);
  LORA_RECEIVEHandle = osThreadCreate(osThread(LORA_RECEIVE), NULL);

  /* definition and creation of LORA_SEND */
  osThreadDef(LORA_SEND, Start_LORA_SEND, osPriorityNormal, 0, 1024);
  LORA_SENDHandle = osThreadCreate(osThread(LORA_SEND), NULL);

  /* definition and creation of MAS_SEND_SLAVE */
  osThreadDef(MAS_SEND_SLAVE, Start_MAS_SEND_SLAVE, osPriorityNormal, 0, 1500);
  MAS_SEND_SLAVEHandle = osThreadCreate(osThread(MAS_SEND_SLAVE), NULL);

  /* definition and creation of SLAVE_REC_MAS */
  osThreadDef(SLAVE_REC_MAS, Start_SLAVE_REC_MAS, osPriorityNormal, 0, 2000);
  SLAVE_REC_MASHandle = osThreadCreate(osThread(SLAVE_REC_MAS), NULL);

  /* definition and creation of SDA_SEND */
  osThreadDef(SDA_SEND, Start_SDA_SEND, osPriorityNormal, 0, 512);
  SDA_SENDHandle = osThreadCreate(osThread(SDA_SEND), NULL);

  /* definition and creation of SDA_RECEIVE */
  osThreadDef(SDA_RECEIVE, Start_SDA_RECEIVE, osPriorityAboveNormal, 0, 750);
  SDA_RECEIVEHandle = osThreadCreate(osThread(SDA_RECEIVE), NULL);

  /* definition and creation of SYSTEM_PROCESS */
  osThreadDef(SYSTEM_PROCESS, Start_SYSTEM_PROCESS, osPriorityNormal, 0, 1500);
  SYSTEM_PROCESSHandle = osThreadCreate(osThread(SYSTEM_PROCESS), NULL);

  /* definition and creation of DATA_PROCESS */
  osThreadDef(DATA_PROCESS, Start_DATA_PROCESS, osPriorityNormal, 0, 3000);
  DATA_PROCESSHandle = osThreadCreate(osThread(DATA_PROCESS), NULL);

  /* definition and creation of RCU_RECEIVE */
  osThreadDef(RCU_RECEIVE, Start_RCU_RECEIVE, osPriorityNormal, 0, 1500);
  RCU_RECEIVEHandle = osThreadCreate(osThread(RCU_RECEIVE), NULL);

  /* definition and creation of WIFI_RECEIVE */
  osThreadDef(WIFI_RECEIVE, Start_WIFI_RECEIVE, osPriorityNormal, 0, 1024);
  WIFI_RECEIVEHandle = osThreadCreate(osThread(WIFI_RECEIVE), NULL);

  /* definition and creation of CURRENT_TASK */
  osThreadDef(CURRENT_TASK, Start_CURRENT_TASK, osPriorityNormal, 0, 1500);
  CURRENT_TASKHandle = osThreadCreate(osThread(CURRENT_TASK), NULL);

  /* definition and creation of PID_TASK */
  osThreadDef(PID_TASK, Start_PID_TASK, osPriorityNormal, 0, 1024);
  PID_TASKHandle = osThreadCreate(osThread(PID_TASK), NULL);

  /* definition and creation of VOLTAGE_TASK */
  osThreadDef(VOLTAGE_TASK, Start_VOLTAGE_TASK, osPriorityNormal, 0, 512);
  VOLTAGE_TASKHandle = osThreadCreate(osThread(VOLTAGE_TASK), NULL);

  /* definition and creation of WHOLE_CURRENT */
  osThreadDef(WHOLE_CURRENT, Start_WHOLE_CURRENT, osPriorityNormal, 0, 4000);
  WHOLE_CURRENTHandle = osThreadCreate(osThread(WHOLE_CURRENT), NULL);

  /* definition and creation of ADC_TASK */
  osThreadDef(ADC_TASK, Start_ADC_TASK, osPriorityNormal, 0, 1024);
  ADC_TASKHandle = osThreadCreate(osThread(ADC_TASK), NULL);

  /* definition and creation of FLASH_WRITE */
  osThreadDef(FLASH_WRITE, Start_FLASH_WRITE, osPriorityNormal, 0, 1024);
  FLASH_WRITEHandle = osThreadCreate(osThread(FLASH_WRITE), NULL);

  /* definition and creation of POWER_ON_TEST */
  osThreadDef(POWER_ON_TEST, Start_POWER_ON_TEST, osPriorityAboveNormal, 0, 1024);
  POWER_ON_TESTHandle = osThreadCreate(osThread(POWER_ON_TEST), NULL);

  /* definition and creation of LCD_TASK */
  osThreadDef(LCD_TASK, Start_LCD_TASK, osPriorityNormal, 0, 128);
  LCD_TASKHandle = osThreadCreate(osThread(LCD_TASK), NULL);

  /* definition and creation of DATA_LOGGING */
  osThreadDef(DATA_LOGGING, Start_DATA_LOGGING, osPriorityNormal, 0, 3000);
  DATA_LOGGINGHandle = osThreadCreate(osThread(DATA_LOGGING), NULL);

  /* definition and creation of CURRENT_PROCESS */
  osThreadDef(CURRENT_PROCESS, Start_CURRENT_PROCESS, osPriorityNormal, 0, 7000);
  CURRENT_PROCESSHandle = osThreadCreate(osThread(CURRENT_PROCESS), NULL);

  /* definition and creation of PID_REGULATION */
  osThreadDef(PID_REGULATION, Start_PID_REGULATION, osPriorityNormal, 0, 1000);
  PID_REGULATIONHandle = osThreadCreate(osThread(PID_REGULATION), NULL);

  /* definition and creation of DATA_LOG_FLASH */
  osThreadDef(DATA_LOG_FLASH, Start_DATA_LOG_FLASH, osPriorityNormal, 0, 512);
  DATA_LOG_FLASHHandle = osThreadCreate(osThread(DATA_LOG_FLASH), NULL);

  /* definition and creation of FLASH_READ */
  osThreadDef(FLASH_READ, Start_FLASH_READ, osPriorityNormal, 0, 1024);
  FLASH_READHandle = osThreadCreate(osThread(FLASH_READ), NULL);

  /* definition and creation of WIFI_SEND */
  osThreadDef(WIFI_SEND, Start_WIFI_SEND, osPriorityNormal, 0, 512);
  WIFI_SENDHandle = osThreadCreate(osThread(WIFI_SEND), NULL);

  /* definition and creation of PROFILE_START */
  osThreadDef(PROFILE_START, Start_PROFILE_START, osPriorityNormal, 0, 2500);
  PROFILE_STARTHandle = osThreadCreate(osThread(PROFILE_START), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */

  osThreadSuspend(CURRENT_TASKHandle);
  osThreadSuspend(PROFILE_STARTHandle);

  HAL_UART_Receive_DMA(&huart4, DATA_FROM_WIFI, 50);
  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx,DMA_IT_HT);
  HAL_UART_Receive_DMA(&huart6, DATA_FROM_LOCAL, 50);
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_HT);
  HAL_UART_Receive_DMA(&huart5, DATA_FROM_MAS_SLAVE, 50);
  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx,DMA_IT_HT);
  HAL_UART_Receive_DMA(&huart1, (uint8_t*)DATA_FROM_SDA, 34);
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
  __enable_irq();
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  osKernelStart();
  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure LSE Drive Capability
  */
  HAL_PWR_EnableBkUpAccess();

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 128;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = ENABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
  */
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  //ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
  */
  hadc3.Instance = ADC3;
  hadc3.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV2;
  hadc3.Init.Resolution = ADC_RESOLUTION_12B;
  hadc3.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc3.Init.ContinuousConvMode = ENABLE;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DMAContinuousRequests = DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_4;
//  sConfig.Rank = ADC_REGULAR_RANK_1;
//  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_3;
//  sConfig.Rank = ADC_REGULAR_RANK_2;
//  sConfig.SamplingTime = ADC_SAMPLETIME_15CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_5;
//  sConfig.Rank = ADC_REGULAR_RANK_3;
//  sConfig.SamplingTime = ADC_SAMPLETIME_28CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
//
//  /** Configure for the selected ADC regular channel its corresponding rank in the sequencer and its sample time.
//  */
//  sConfig.Channel = ADC_CHANNEL_6;
//  sConfig.Rank = ADC_REGULAR_RANK_4;
//  sConfig.SamplingTime = ADC_SAMPLETIME_56CYCLES;
//  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
//  {
//    Error_Handler();
//  }
  /* USER CODE BEGIN ADC3_Init 2 */
//
  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */
//
  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */
//
  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_DISABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_DISABLE;
  hcrc.Init.GeneratingPolynomial = 203;
  hcrc.Init.CRCLength = CRC_POLYLENGTH_8B;
  hcrc.Init.InitValue = 255;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x00300F38;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x00707CBB;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief IWDG Initialization Function
  * @param None
  * @retval None
  */
static void MX_IWDG_Init(void)
{

  /* USER CODE BEGIN IWDG_Init 0 */

  /* USER CODE END IWDG_Init 0 */

  /* USER CODE BEGIN IWDG_Init 1 */

  /* USER CODE END IWDG_Init 1 */
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_32;
  hiwdg.Init.Window = 4095;
  hiwdg.Init.Reload = 3999;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN IWDG_Init 2 */

  /* USER CODE END IWDG_Init 2 */

}

/**
  * @brief RTC Initialization Function
  * @param None
  * @retval None
  */
static void MX_RTC_Init(void)
{

  /* USER CODE BEGIN RTC_Init 0 */

  /* USER CODE END RTC_Init 0 */

  RTC_TimeTypeDef sTime = {0};
  RTC_DateTypeDef sDate = {0};

  /* USER CODE BEGIN RTC_Init 1 */

  /* USER CODE END RTC_Init 1 */

  /** Initialize RTC Only
  */
  hrtc.Instance = RTC;
  hrtc.Init.HourFormat = RTC_HOURFORMAT_24;
  hrtc.Init.AsynchPrediv = 127;
  hrtc.Init.SynchPrediv = 255;
  hrtc.Init.OutPut = RTC_OUTPUT_DISABLE;
  hrtc.Init.OutPutPolarity = RTC_OUTPUT_POLARITY_HIGH;
  hrtc.Init.OutPutType = RTC_OUTPUT_TYPE_OPENDRAIN;
  if (HAL_RTC_Init(&hrtc) != HAL_OK)
  {
    Error_Handler();
  }

  /* USER CODE BEGIN Check_RTC_BKUP */

  /* USER CODE END Check_RTC_BKUP */

  /** Initialize RTC and set the Time and Date
  */
  sTime.Hours = 0x0;
  sTime.Minutes = 0x0;
  sTime.Seconds = 0x0;
  sTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sTime.StoreOperation = RTC_STOREOPERATION_RESET;
  if (HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  sDate.WeekDay = RTC_WEEKDAY_MONDAY;
  sDate.Month = RTC_MONTH_JANUARY;
  sDate.Date = 0x1;
  sDate.Year = 0x0;

  if (HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_HIGH;
  hspi4.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 7;
  hspi4.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 2047;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief UART4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 115200;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_SWAP_INIT;
  huart4.AdvancedInit.Swap = UART_ADVFEATURE_SWAP_ENABLE;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}

/**
  * @brief UART5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */

  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  huart5.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart5.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart5, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart1, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 115200;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  huart3.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart3.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

/**
  * @brief USART6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART6_UART_Init(void)
{

  /* USER CODE BEGIN USART6_Init 0 */

  /* USER CODE END USART6_Init 0 */

  /* USER CODE BEGIN USART6_Init 1 */

  /* USER CODE END USART6_Init 1 */
  huart6.Instance = USART6;
  huart6.Init.BaudRate = 115200;
  huart6.Init.WordLength = UART_WORDLENGTH_8B;
  huart6.Init.StopBits = UART_STOPBITS_1;
  huart6.Init.Parity = UART_PARITY_NONE;
  huart6.Init.Mode = UART_MODE_TX_RX;
  huart6.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart6.Init.OverSampling = UART_OVERSAMPLING_16;
  huart6.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart6.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_RS485Ex_Init(&huart6, UART_DE_POLARITY_HIGH, 0, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART6_Init 2 */

  /* USER CODE END USART6_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream0_IRQn);
  /* DMA1_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA1_Stream2_IRQn);
  /* DMA2_Stream1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream1_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream1_IRQn);
  /* DMA2_Stream2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream2_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream2_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE BEGIN MX_GPIO_Init_1 */
/* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_CS_GPIO_Port, LORA_CS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_TX_SW_GPIO_Port, LORA_TX_SW_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LORA_RESET_GPIO_Port, LORA_RESET_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(FLASH_CS_GPIO_Port, FLASH_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : CURRENT_CS_Pin LED_RED_Pin LED_YELLOW_Pin LED_GREEN_Pin */
  GPIO_InitStruct.Pin = CURRENT_CS_Pin|LED_RED_Pin|LED_YELLOW_Pin|LED_GREEN_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_CS_Pin FLASH_CS_Pin */
  GPIO_InitStruct.Pin = LORA_CS_Pin|FLASH_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_TX_SW_Pin */
  GPIO_InitStruct.Pin = LORA_TX_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_TX_SW_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LORA_BUSY_Pin DIO1_Pin */
  GPIO_InitStruct.Pin = LORA_BUSY_Pin|DIO1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOG, &GPIO_InitStruct);

  /*Configure GPIO pin : LORA_RESET_Pin */
  GPIO_InitStruct.Pin = LORA_RESET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LORA_RESET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : LED_Pin */
  GPIO_InitStruct.Pin = LED_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_GPIO_Port, &GPIO_InitStruct);

/* USER CODE BEGIN MX_GPIO_Init_2 */
/* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN 5 */
	SDA_SEND *sda_msg;
	SDA_CONN	*connmsg;
	osEvent	connevt;
	SYSTEM_PROCESS	*syspromsg;

	uint8_t DUMMY_RADIO_SEND = 0x00;
	uint8_t DUMMY_RADIO_SEND_LENGTH = 1;
	uint8_t Sperading_Factor;
    uint8_t CR;
    uint8_t DATA_FOR_PROCESSING[50];
	uint16_t Band_Width;
	uint32_t frequency_t;
	uint32_t offset_t;
    char P_DATA[50];
	char DATA_FROM_FLASH[200];
	static uint8_t CONN_CHECK_DATA = 0xFF;
	static uint8_t START_STOP_MODE_DATA = 0xFF;
	static uint8_t CONN_DATA=0;
	static int increment = 0;

	RADIO_START(LORA_DEVICE);
	osDelay(1000);
	osTimerStart(TCU_SDA_TIMERHandle, 1000);

	for(int i =0;i<50;i++)
	{
		SLAVE_Read_Data((STARTING_ADD_OF_PROFILES+increment), P_DATA, 50);
		increment = increment+50;
		for(int j =0;j<50;j++)
		{
			PROFILES[i][j] = P_DATA[j];
		}
		memset(P_DATA,0,50);
	}

	SLAVE_Read_Data(STARTING_ADD_OF_ENGG_MODE,DATA_FROM_FLASH,200);
	log_interval = (((int)DATA_FROM_FLASH[8])<<8)|(int)DATA_FROM_FLASH[9];
	current_limit_value = (int)DATA_FROM_FLASH[7];

	sda_msg = osPoolAlloc(sdasendpool);
	sda_msg->SDA_SEND_DATA[0] = SDA_CALLIBRATION_FACTOR;
	sda_msg->SDA_SEND_DATA[1] = DATA_FROM_FLASH[12];
	sda_msg->SDA_SEND_DATA[2] = DATA_FROM_FLASH[13];
	sda_msg->SDA_SEND_DATA[3] = DATA_FROM_FLASH[14];
	sda_msg->SDA_SEND_DATA[4] = DATA_FROM_FLASH[15];
	osMessagePut(SDA_SEND_qHandle, (uint32_t)sda_msg,100);
	osDelay(1000);
	frequency_t = (((uint32_t)DATA_FROM_FLASH[17])<<24)|(((uint32_t)DATA_FROM_FLASH[18])<<16)|(((uint32_t)DATA_FROM_FLASH[19])<<8)|((uint32_t)DATA_FROM_FLASH[20]);
	offset_t = (((uint32_t)DATA_FROM_FLASH[21])<<24)|(((uint32_t)DATA_FROM_FLASH[22])<<16)|(((uint32_t)DATA_FROM_FLASH[23])<<8)|((uint32_t)DATA_FROM_FLASH[24]);
	Sperading_Factor = DATA_FROM_FLASH[25];
	Band_Width = (((uint16_t)DATA_FROM_FLASH[26])<<8)|((uint16_t)DATA_FROM_FLASH[27]);
	switch(Band_Width)
	{
	case 500:
		Band_Width = 6;
		break;
	case 250:
		Band_Width = 5;
		break;
	case 125:
		Band_Width = 4;
		break;
	case 62:
		Band_Width = 3;
		break;
	case 41:
		Band_Width = 10;
		break;
	case 31:
		Band_Width = 2;
		break;
	case 20:
		Band_Width = 9;
		break;
	case 15:
		Band_Width = 1;
		break;
	case 10:
		Band_Width = 8;
		break;
	case 7:
		Band_Width = 0;
		break;
	}
	CR = DATA_FROM_FLASH[28];
	switch(CR)
	{
	case 45:
		CR =1 ;
		break;
	case 46:
		CR =2 ;
		break;
	case 47:
		CR =3 ;
		break;
	case 48:
		CR =4 ;
		break;
	}
	RADIO_SETUP_LORA(frequency_t, offset_t, Sperading_Factor, Band_Width, CR, LDRO_OFF);
	RADIO_TRANSMIT((uint8_t *)&DUMMY_RADIO_SEND,DUMMY_RADIO_SEND_LENGTH,1000,TXPower,WAIT_TX);

	int pannel =0;
	int index = 36;
	if(DATA_FROM_FLASH[index]==KP_KI_KD_DATA)
	{

		for(int i =0;i<16;i++)
		{
		pannel = (DATA_FROM_FLASH[index+1])/10;
		if((pannel >=0)&&(pannel<=15))
		{
			KP[pannel] = (DATA_FROM_FLASH[index+2])/10;
				KI[pannel] = (DATA_FROM_FLASH[index+3])/10;
				KD[pannel] = (DATA_FROM_FLASH[index+4])/10;
				index = index+4;
		}

		}
	}

	if(DATA_FROM_FLASH[192] == 1)
	{

		if(DATA_FROM_FLASH[138] == SYSTEM_STATE)
		{
		syspromsg = osPoolAlloc(syspropool);
		for(int i =0;i<50;i++)
		{
		syspromsg->DATA_TO_PROCESS[i] = DATA_FOR_PROCESSING[i+141];
		}
		osMessagePut(SYSTEM_PROCESS_qHandle, (uint32_t)syspromsg,100);
		DATA_FROM_FLASH[192] = 0;
		SLAVE_Erase_4K(STARTING_ADD_OF_ENGG_MODE);
		SLAVE_Write_Data(STARTING_ADD_OF_ENGG_MODE, (char*)DATA_FROM_FLASH,200);
		}
	}

  /* Infinite loop */
  for(;;)
  {
	  HAL_IWDG_Refresh(&hiwdg);
	  connevt = osMessageGet(SDA_CONN_qHandle, 0);
	  if(connevt.status == osEventMessage)
	  {
		  connmsg = connevt.value.p;
		  CONN_DATA = connmsg->SDA_CONN_DATA[0];
		  osPoolFree(sdaconnpool, connmsg);
	  }
	  if(CONN_DATA == 0)
	  {
	  sda_msg = osPoolAlloc(sdasendpool);
	  sda_msg->SDA_SEND_DATA[0]=CONN_CHECK_DATA;
	  sda_msg->SDA_SEND_DATA[1]=START_STOP_MODE_DATA;
	  osMessagePut(SDA_SEND_qHandle, (uint32_t)sda_msg,10);
	  }
    osDelay(1000);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_Start_LORA_RECEIVE */
/**
* @brief Function implementing the LORA_RECEIVE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LORA_RECEIVE */
void Start_LORA_RECEIVE(void const * argument)
{
  /* USER CODE BEGIN Start_LORA_RECEIVE */
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	LORA_SEND *lorasendmsg;
	KP_KI_KD *kpkikd_msg;
	SYSTEM_PROCESS	*syspromsg;
	SLAVE_SEND	*slavemsg;
	CURRENT_LIMIT_Q *currlimitmsg;
	FLASH_READ	*flashreadmsg;
	SLAVE_ID *idmsg;
	SDA_SEND *sda_msg;
	FLASH_DATA	*flashmsg;

	uint8_t ID_S[3];
	uint8_t DATA_FOR_RCU[MAX_BUFFER_SIZE];
	uint8_t CRC_VALUE_OF_DATA_FOR_RCU;
	uint8_t RXPacketL;
	uint8_t DATA_FROM_RCU[50];
	uint8_t RECEIVED_CRC;
	uint8_t CALCULATED_CRC;
    uint8_t CR;
	uint8_t DATA_FOR_PROCESSING[50];
	uint8_t DATA_OF_PROFILE[50];
	uint8_t Sperading_Factor;
	uint16_t Band_Width;
	uint32_t frequency_t;
	uint32_t offset_t;
    int pos =0;
    char DATA_FROM_FLASH[200];
    static uint32_t clear_address = STARTING_ADD_OF_ENGG_MODE;
	static int state=0;
	static int z =0;
  /* Infinite loop */
  for(;;)
  {
	  memset(DATA_FROM_RCU,0,LORA_RX_BUFF_SIZE);
	  RXPacketL = RADIO_RECEIVE(DATA_FROM_RCU, LORA_RX_BUFF_SIZE, 5000, WAIT_RX);

		if(RXPacketL !=0)
		{
		LORA_ERROR_DATA =0;
		DATA_FROM_RCU[0] = LORA_BIT;
		if(DATA_FROM_RCU[1] == MODEM_BIT)
		{
		RECEIVED_CRC = DATA_FROM_RCU[LORA_RX_BUFF_SIZE-1];
		CALCULATED_CRC = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FROM_RCU,(LORA_RX_BUFF_SIZE-1));
		if(RECEIVED_CRC == CALCULATED_CRC)
		{
		if((DATA_FROM_RCU[3] == SEARCHING_DEVICES) &&(DATA_FROM_RCU[4]!=MY_TCU_ID)&&(DATA_FROM_RCU[5]!=MY_TCU_ID)&&(DATA_FROM_RCU[6]!=MY_TCU_ID)&&(DATA_FROM_RCU[7]!=MY_TCU_ID))
		{
		WHOM_TO_GIVE_DATA = MODEM_SYSTEM;
		state = SEARCHING_DEVICES;
		sDate.Date = DATA_FROM_RCU[10];
		sDate.Month = DATA_FROM_RCU[11];
		sDate.Year = DATA_FROM_RCU[12];
		sTime.Hours = DATA_FROM_RCU[13];
		sTime.Minutes = DATA_FROM_RCU[14];
		sTime.Seconds = DATA_FROM_RCU[15];
		HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
		HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		osThreadSuspend(RCU_RECEIVEHandle);
		}
		else if(((DATA_FROM_RCU[3] == CLOSE_LOOP_START)||(DATA_FROM_RCU[3] == UPADTE_TEMP_OR_POWER)||(DATA_FROM_RCU[3] == OPEN_LOOP_START)||(DATA_FROM_RCU[3] == SLEEP)||(DATA_FROM_RCU[3]==SDA_CALLIBRATION)||(DATA_FROM_RCU[3]==OPEN_LOOP_START_PROFILE)||(DATA_FROM_RCU[3]==CLOSE_LOOP_START_PROFILE)) && (DATA_FROM_RCU[2]==MY_TCU_ID))
		{
		state = PROCESS_DATA;
		}
		else if(((DATA_FROM_RCU[3] == UPADTE_TEMP_OR_POWER)||(DATA_FROM_RCU[3]==OPEN_LOOP_START_PROFILE)||(DATA_FROM_RCU[3]==CLOSE_LOOP_START_PROFILE)) && ((DATA_FROM_RCU[2]==ID_S_PRESENT[0])||(DATA_FROM_RCU[2]==ID_S_PRESENT[1])||(DATA_FROM_RCU[2]==ID_S_PRESENT[2])))
		{
		state = SEND_DATA_TO_SLAVE;
		}

		switch(DATA_FROM_RCU[3])
		{
		case FETCH_BIST:
			state = FETCH_BIST;
			break;
		case CURRENT_LIMIT:
			SEND_ACK();
			state = CURRENT_LIMIT;
			break;
		case LOGGING_INTERVAL:
			SEND_ACK();
			state = LOGGING_INTERVAL;
			break;
		case LORA_PARAMETERS:
			SEND_ACK();
			state = LORA_PARAMETERS;
			break;
		case FETCH_DATA:
			state = FETCH_DATA;
			break;
		case CLEAR_FLASH:
			SEND_ACK();
			state = CLEAR_FLASH;
			break;
		case SDA_CALLIBRATION:
			SEND_ACK();
			state = SDA_CALLIBRATION;
			break;
		case UPDATE_PROFILE:

			state = UPDATE_PROFILE;
			break;
		case KP_KI_KD_DATA:
			SEND_ACK();
			state = KP_KI_KD_DATA;
			break;
		case MASTER_SLAVE_CONFIG:
			state = MASTER_SLAVE_CONFIG;
			break;
		case RESET_MAS_SLAVE:
			state = RESET_MAS_SLAVE;
			break;
		}
		switch(state)
		{
		case	SDA_CALLIBRATION:
			sda_msg = osPoolAlloc(sdasendpool);
			sda_msg->SDA_SEND_DATA[0] = SDA_CALLIBRATION_FACTOR;
			sda_msg->SDA_SEND_DATA[1] = DATA_FROM_RCU[4];
			sda_msg->SDA_SEND_DATA[2] = DATA_FROM_RCU[5];
			sda_msg->SDA_SEND_DATA[3] = DATA_FROM_RCU[6];
			sda_msg->SDA_SEND_DATA[4] = DATA_FROM_RCU[7];
			osMessagePut(SDA_SEND_qHandle, (uint32_t)sda_msg,100);
			flashmsg = osPoolAlloc(flashpool);
			flashmsg->FLASH_DATA_t[0] = SDA_CALLIBRATION_FACTOR;
			flashmsg->FLASH_DATA_t[1] = DATA_FROM_RCU[4];
			flashmsg->FLASH_DATA_t[2] = DATA_FROM_RCU[5];
			flashmsg->FLASH_DATA_t[3] = DATA_FROM_RCU[6];
			flashmsg->FLASH_DATA_t[4] = DATA_FROM_RCU[7];
			osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
			state = 0;
		break;

		case CURRENT_LIMIT:
			currlimitmsg = osPoolAlloc(currlimitpool);
			currlimitmsg->CURRENT_LIMIT_DATA = DATA_FROM_RCU[4];
			osMessagePut(CURRENT_LIMIT_Q_qHandle, (uint32_t)currlimitmsg, 10);
			flashmsg = osPoolAlloc(flashpool);
			flashmsg->FLASH_DATA_t[0] = CURRENT_LIMIT;
			flashmsg->FLASH_DATA_t[1] = DATA_FROM_RCU[4];
			osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
			state = 0;
		break;

		case FETCH_DATA:
			flashreadmsg = osPoolAlloc(flashreadpool);
			flashreadmsg->FLASH_READ_DATA[0] = DATA_FOR_RCU[4];
			osMessagePut(FLASH_READ_qHandle,(uint32_t) flashreadmsg,10);
			state = 0;
		break;

		case FETCH_BIST:
			flashreadmsg = osPoolAlloc(flashreadpool);
			flashreadmsg->FLASH_READ_DATA[0] = DATA_FOR_RCU[3];
			osMessagePut(FLASH_READ_qHandle,(uint32_t) flashreadmsg,10);
			state = 0;
		break;

		case CLEAR_FLASH:
			SLAVE_Erase_4K(clear_address);
			state = 0;
		break;


		case RESET_MAS_SLAVE:
			  memset(slavemsg->SLAVE_DATA,0,50);
			  slavemsg = osPoolAlloc(slavesendpool);
			  slavemsg->SLAVE_DATA[0] = MASTER_BIT;
			  slavemsg->SLAVE_DATA[1] = ID_S_PRESENT[0];
			  slavemsg->SLAVE_DATA[2] = RESET_MAS_SLAVE;
			  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
			  osDelay(100);
			  memset(slavemsg->SLAVE_DATA,0,50);
			  slavemsg = osPoolAlloc(slavesendpool);
			  slavemsg->SLAVE_DATA[0] = MASTER_BIT;
			  slavemsg->SLAVE_DATA[1] = ID_S_PRESENT[1];
			  slavemsg->SLAVE_DATA[2] = RESET_MAS_SLAVE;
			  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
			  osDelay(100);
			  memset(slavemsg->SLAVE_DATA,0,50);
			  slavemsg = osPoolAlloc(slavesendpool);
			  slavemsg->SLAVE_DATA[0] = MASTER_BIT;
			  slavemsg->SLAVE_DATA[1] = ID_S_PRESENT[2];
			  slavemsg->SLAVE_DATA[2] = RESET_MAS_SLAVE;
			  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
			  osDelay(100);
		break;

		case LOGGING_INTERVAL:
			log_interval = ((uint16_t)DATA_FROM_RCU[4]<<8)|(uint16_t)DATA_FROM_RCU[5];
			flashmsg = osPoolAlloc(flashpool);
			flashmsg->FLASH_DATA_t[0] = LOGGING_INTERVAL;
			flashmsg->FLASH_DATA_t[1] = DATA_FROM_RCU[4];
			flashmsg->FLASH_DATA_t[2] = DATA_FROM_RCU[5];
			osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,10);
			state = 0;
		break;

		case KP_KI_KD_DATA:
			 pos =0;
 	  	  	 kpkikd_msg = osPoolAlloc(kpkikdpool);
 	  	  	 for(int i =0;i<4;i++)
 	  	  	 {
 	  	  	 kpkikd_msg->KP_KI_KD_t[i] = DATA_FROM_RCU[i+4];
 	  	  	 }
 	  	  	 osMessagePut(KP_KI_KD_qHandle, (uint32_t)kpkikd_msg,100);
 	  	  	 SLAVE_Read_Data(STARTING_ADD_OF_ENGG_MODE,DATA_FROM_FLASH,200);
 	  	  	 for(int i =0;i<200;i++)
 	  	  	 {
 	  	  		 flashmsg->FLASH_DATA_t[i] = DATA_FROM_FLASH[i];
 	  	  	 }
 	  	  	 pos = (DATA_FROM_RCU[4])/10;
 	  	  	 pos = (pos*4)+37;
	  	  	 	flashmsg->FLASH_DATA_t[0] = KP_KI_KD_DATA;
 	  	  	 	flashmsg->FLASH_DATA_t[36] = KP_KI_KD_DATA;
				flashmsg->FLASH_DATA_t[pos] = DATA_FROM_RCU[4];
				flashmsg->FLASH_DATA_t[pos+1] = DATA_FROM_RCU[5];
				flashmsg->FLASH_DATA_t[pos+2] = DATA_FROM_RCU[6];
				flashmsg->FLASH_DATA_t[pos+3] = DATA_FROM_RCU[7];

 	  	  	 osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
 			state = 0;
		break;

		case LORA_PARAMETERS:
			frequency_t = (((uint32_t)DATA_FROM_RCU[4])<<24)|(((uint32_t)DATA_FROM_RCU[5])<<16)|(((uint32_t)DATA_FROM_RCU[6])<<8)|((uint32_t)DATA_FROM_RCU[7]);
			offset_t = (((uint32_t)DATA_FROM_RCU[8])<<24)|(((uint32_t)DATA_FROM_RCU[9])<<16)|(((uint32_t)DATA_FROM_RCU[10])<<8)|((uint32_t)DATA_FROM_RCU[11]);
			Sperading_Factor = DATA_FROM_RCU[12];
			Band_Width = (((uint16_t)DATA_FROM_RCU[13])<<8)|((uint16_t)DATA_FROM_RCU[14]);
			switch(Band_Width)
			{
			case 500:
				Band_Width = 6;
				break;
			case 250:
				Band_Width = 5;
				break;
			case 125:
				Band_Width = 4;
				break;
			case 62:
				Band_Width = 3;
				break;
			case 41:
				Band_Width = 10;
				break;
			case 31:
				Band_Width = 2;
				break;
			case 20:
				Band_Width = 9;
				break;
			case 15:
				Band_Width = 1;
				break;
			case 10:
				Band_Width = 8;
				break;
			case 7:
				Band_Width = 0;
				break;
			}
			CR = DATA_FROM_RCU[15];
			switch(CR)
			{
			case 45:
				CR =1 ;
				break;
			case 46:
				CR =2 ;
				break;
			case 47:
				CR =3 ;
				break;
			case 48:
				CR =4 ;
				break;
			}
			RADIO_RESET();
			RADIO_START(LORA_DEVICE);
			osDelay(1000);
			RADIO_SETUP_LORA(frequency_t, offset_t, Sperading_Factor, Band_Width, CR, LDRO_OFF);
			flashmsg = osPoolAlloc(flashpool);
			flashmsg->FLASH_DATA_t[0] = LORA_PARAMETERS;
			for(int i =1;i<13;i++)
			{
				flashmsg->FLASH_DATA_t[i] = DATA_FROM_RCU[i+3];
			}
			osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
			state = 0;
		break;

		case SEARCHING_DEVICES:
			memset(DATA_FOR_RCU,0xFF,BUFFER_SIZE_ALL);
			switch(Running_Status)
			{
			case FIRST_TIME:
				DATA_FOR_RCU[0]=LORA_BIT;
				DATA_FOR_RCU[1]=MODEM_REPLY_BIT;
				DATA_FOR_RCU[3]=SEARCH_IDs;
				DATA_FOR_RCU[4]=MY_TCU_ID;
				CRC_VALUE_OF_DATA_FOR_RCU = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_RCU,(BUFFER_SIZE_ALL-1));
				DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)]=CRC_VALUE_OF_DATA_FOR_RCU;
				lorasendmsg = osPoolAlloc(lorasendpool);
				memcpy(lorasendmsg->LORA_SEND_DATA,DATA_FOR_RCU,200);
				osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);

			break;

			case RUNNING:
				DATA_FOR_RCU[0]=LORA_BIT;
				DATA_FOR_RCU[1]=MODEM_REPLY_BIT;
				DATA_FOR_RCU[3]=RUNNING_SEARCH;
				DATA_FOR_RCU[4]=MY_TCU_ID;
				DATA_FOR_RCU[5] = ID_S_PRESENT[0];
				DATA_FOR_RCU[6] = ID_S_PRESENT[1];
				DATA_FOR_RCU[7] = ID_S_PRESENT[2];
				DATA_FOR_RCU[8] = START_STOP_STATE;
				DATA_FOR_RCU[9] = START_CONDITION;
				DATA_FOR_RCU[10] = MODE_CONDITION;
				CRC_VALUE_OF_DATA_FOR_RCU = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_RCU,(BUFFER_SIZE_ALL-1));
				DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)]=CRC_VALUE_OF_DATA_FOR_RCU;
				lorasendmsg = osPoolAlloc(lorasendpool);
				memcpy(lorasendmsg->LORA_SEND_DATA,DATA_FOR_RCU,200);
				osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
				Running_Status = RUNNING;
			break;
			}
			state = 0;
		break;

		case MASTER_SLAVE_CONFIG:
		if((DATA_FROM_RCU[4]==MY_TCU_ID)||(DATA_FROM_RCU[6]==MY_TCU_ID)||(DATA_FROM_RCU[8]==MY_TCU_ID)||(DATA_FROM_RCU[10]==MY_TCU_ID))
		{
		if(DATA_FROM_RCU[4]==MY_TCU_ID)
		{
		if(DATA_FROM_RCU[5]==I_AM_MASTER)
		{
			I_AM = I_AM_MASTER;
			ID_S[0]=DATA_FROM_RCU[6];
			ID_S[1]=DATA_FROM_RCU[8];
			ID_S[2]=DATA_FROM_RCU[10];
			idmsg = osPoolAlloc(slaveidpool);
			for(int i =0;i<3;i++)
			{
			  idmsg->SLAVE_ID[i] = ID_S[i];
			}
			osMessagePut(SLAVE_ID_qHandle, (uint32_t)idmsg,100);
		}
		else if(DATA_FROM_RCU[5]==I_AM_SLAVE)
		{
			  I_AM = I_AM_SLAVE;
		}
		}
		if(DATA_FROM_RCU[8]==MY_TCU_ID)
		{
		if(DATA_FROM_RCU[9]==I_AM_MASTER)
		{
			  I_AM = I_AM_MASTER;
			  ID_S[0]=DATA_FROM_RCU[4];
			  ID_S[1]=DATA_FROM_RCU[6];
			  ID_S[2]=DATA_FROM_RCU[10];
			  idmsg = osPoolAlloc(slaveidpool);
			  for(int i =0;i<3;i++)
			  {
				  idmsg->SLAVE_ID[i] = ID_S[i];
			  }
			  osMessagePut(SLAVE_ID_qHandle, (uint32_t)idmsg,100);
		 }
		 else if(DATA_FROM_RCU[9]==I_AM_SLAVE)
		 {
			  I_AM = I_AM_SLAVE;
		 }
		}
		if(DATA_FROM_RCU[6]==MY_TCU_ID)
		{
		if(DATA_FROM_RCU[7]==I_AM_MASTER)
		{
			  I_AM = I_AM_MASTER;
			  ID_S[0]=DATA_FROM_RCU[4];
			  ID_S[1]=DATA_FROM_RCU[8];
			  ID_S[2]=DATA_FROM_RCU[10];
			  idmsg = osPoolAlloc(slaveidpool);
			  for(int i =0;i<3;i++)
			  {
				  idmsg->SLAVE_ID[i] = ID_S[i];
			  }
			  osMessagePut(SLAVE_ID_qHandle, (uint32_t)idmsg,100);
		 }
		 else if(DATA_FROM_RCU[7]==I_AM_SLAVE)
		 {
			  I_AM = I_AM_SLAVE;
		 }
		}
		if(DATA_FROM_RCU[10]==MY_TCU_ID)
		{
		if(DATA_FROM_RCU[11]==I_AM_MASTER)
		{
			  I_AM = I_AM_MASTER;
			  ID_S[0]=DATA_FROM_RCU[6];
			  ID_S[1]=DATA_FROM_RCU[8];
			  ID_S[2]=DATA_FROM_RCU[4];
			  idmsg = osPoolAlloc(slaveidpool);
			  for(int i =0;i<3;i++)
			  {
				  idmsg->SLAVE_ID[i] = ID_S[i];
			  }
			  osMessagePut(SLAVE_ID_qHandle, (uint32_t)idmsg,100);
		 }
		 else if(DATA_FROM_RCU[11]==I_AM_SLAVE)
		 {
			  I_AM = I_AM_SLAVE;
		 }
		}
		if(I_AM == I_AM_MASTER)
		{
		  WHOM_TO_GIVE_DATA = MODEM_SYSTEM;
		}
		if(I_AM == I_AM_SLAVE)
		{
		  WHOM_TO_GIVE_DATA = MAS_SLAVE_SYSTEM;
		}
		}
		state = 0;
		break;

		case PROCESS_DATA:
		for(int i =0;i<49;i++)
		{
		DATA_FOR_PROCESSING[i] = DATA_FROM_RCU[i+1];
		}
		DATA_FOR_PROCESSING[0] = MASTER_BIT;
		DATA_FOR_PROCESSING[49] = 0x00;
		syspromsg = osPoolAlloc(syspropool);
		for(int i =0;i<50;i++)
		{
		syspromsg->DATA_TO_PROCESS[i] = DATA_FOR_PROCESSING[i];
		}
		osMessagePut(SYSTEM_PROCESS_qHandle, (uint32_t)syspromsg,100);
		if((DATA_FROM_RCU[3] == CLOSE_LOOP_START)||(DATA_FROM_RCU[3] == OPEN_LOOP_START)||(DATA_FROM_RCU[3] == SLEEP)||(DATA_FROM_RCU[3]==OPEN_LOOP_START_PROFILE)||(DATA_FROM_RCU[3]==CLOSE_LOOP_START_PROFILE))
		{
		  for(int i =0;i<49;i++)
		  {
			  DATA_FOR_PROCESSING[i] = DATA_FROM_RCU[i+1];
		  }
		  DATA_FOR_PROCESSING[0] = MASTER_BIT;
		  DATA_FOR_PROCESSING[49] = 0x00;
		  slavemsg = osPoolAlloc(slavesendpool);
		  for(int i =0;i<50;i++)
		  {
			  slavemsg->SLAVE_DATA[i] = DATA_FOR_PROCESSING[i];
		  }
		  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
		}
		state = 0;
		break;

		case UPDATE_PROFILE:
		for(int i =0;i<49;i++)
		{
		DATA_OF_PROFILE[i] = DATA_FROM_RCU[i+1];
		}

		if((I_AM==I_AM_MASTER)&&(DATA_FROM_RCU[2]==MY_TCU_ID))
		{
			for(int i=0;i<50;i++)
			{
				PROFILES[z][i] = DATA_OF_PROFILE[i];
			}
			PROFILE_COUNT=z;
			z = z+1;
			if(z==50)
			{
				z=0;
			}
		}
		if((I_AM==I_AM_MASTER)&&(DATA_FROM_RCU[2]!=MY_TCU_ID))
		{
			DATA_OF_PROFILE[0] = MASTER_BIT;
			  slavemsg = osPoolAlloc(slavesendpool);
			  for(int i =0;i<50;i++)
			  {
				  slavemsg->SLAVE_DATA[i] = DATA_OF_PROFILE[i];
			  }
			  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
		}
		state = 0;
		SEND_ACK();
		break;


		case SEND_DATA_TO_SLAVE:
		for(int i =0;i<49;i++)
		{
		DATA_FOR_PROCESSING[i] = DATA_FROM_RCU[i+1];
		}
		DATA_FOR_PROCESSING[0] = MASTER_BIT;
		DATA_FOR_PROCESSING[49] = 0x00;
		slavemsg = osPoolAlloc(slavesendpool);
		for(int i =0;i<50;i++)
		{
		slavemsg->SLAVE_DATA[i] = DATA_FOR_PROCESSING[i];
		}
		osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
		if((DATA_FROM_RCU[3] == CLOSE_LOOP_START)||(DATA_FROM_RCU[3] == OPEN_LOOP_START)||(DATA_FROM_RCU[3] == SLEEP)||(DATA_FROM_RCU[3]==OPEN_LOOP_START_PROFILE)||(DATA_FROM_RCU[3]==CLOSE_LOOP_START_PROFILE))
		{
		  for(int i =0;i<49;i++)
		  {
			  DATA_FOR_PROCESSING[i] = DATA_FROM_RCU[i+1];
		  }
		  DATA_FOR_PROCESSING[0] = MASTER_BIT;
		  DATA_FOR_PROCESSING[49] = 0x00;
		  syspromsg = osPoolAlloc(syspropool);
		  for(int i =0;i<50;i++)
		  {
			  syspromsg->DATA_TO_PROCESS[i] = DATA_FOR_PROCESSING[i];
		  }
		  osMessagePut(SYSTEM_PROCESS_qHandle, (uint32_t)syspromsg,100);
		}
		state = 0;
		break;
		}


		if(DATA_FROM_RCU[40]==ACK_NACK_DATA)
		{
			  if(DATA_FROM_RCU[41]==ACK)
			  {
			  xTaskNotifyGive(DATA_LOGGINGHandle);
			  }
		}
		}
		}
		}


    osDelay(1);
  }
  /* USER CODE END Start_LORA_RECEIVE */
}

/* USER CODE BEGIN Header_Start_LORA_SEND */
/**
* @brief Function implementing the LORA_SEND thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LORA_SEND */
void Start_LORA_SEND(void const * argument)
{
  /* USER CODE BEGIN Start_LORA_SEND */
	LORA_SEND *lorasendmsg;
	osEvent	lorasendevt;

	uint8_t DATA_FOR_RCU[MAX_BUFFER_SIZE];
	uint8_t TXPacketL;
  /* Infinite loop */
  for(;;)
  {
	  lorasendevt = osMessageGet(LORA_SEND_qHandle, osWaitForever);
	  if(lorasendevt.status == osEventMessage)
	  {
		  lorasendmsg = lorasendevt.value.p;
		  memcpy(DATA_FOR_RCU,lorasendmsg->LORA_SEND_DATA,200);
		  osPoolFree(lorasendpool, lorasendmsg);
		  TXPacketL = 200;
		  RADIO_CHECK_BUSY();
		  RADIO_TRANSMIT(DATA_FOR_RCU,TXPacketL,5000,TXPower,WAIT_TX);
		  HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
	  }
    osDelay(5);
  }
  /* USER CODE END Start_LORA_SEND */
}

/* USER CODE BEGIN Header_Start_MAS_SEND_SLAVE */
/**
* @brief Function implementing the MAS_SEND_SLAVE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_MAS_SEND_SLAVE */
void Start_MAS_SEND_SLAVE(void const * argument)
{
  /* USER CODE BEGIN Start_MAS_SEND_SLAVE */
	SLAVE_ID *idmsg;
	LORA_SEND *lorasendmsg;
	SLAVE_SEND	*slavemsg;

	osEvent idevt,slaveevt;

	uint8_t SLAVE_ID_RX[3];
	uint8_t DATA_FOR_SLAVE[50];
	uint8_t DATA_FOR_RCU[MAX_BUFFER_SIZE];
	uint8_t DUMMY =0;
	uint8_t CRC_VALUE_OF_SLAVE_DATA;
	uint8_t CRC_VALUE_FOR_LORA;
	int notification_value;
  /* Infinite loop */
  for(;;)
  {
	  slaveevt = osMessageGet(SLAVE_SEND_qHandle, 0);
	  if(slaveevt.status == osEventMessage)
	  {
		  memset(DATA_FOR_SLAVE,0xFF,50);
		  slavemsg = slaveevt.value.p;
		  memcpy(DATA_FOR_SLAVE,slavemsg->SLAVE_DATA,50);
		  osPoolFree(slavesendpool, slavemsg);
		  CRC_VALUE_OF_SLAVE_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_SLAVE,49);
		  DATA_FOR_SLAVE[49] = CRC_VALUE_OF_SLAVE_DATA;
		  HAL_UART_Transmit(&huart5, DATA_FOR_SLAVE, 50,100);
	  }

	  idevt = osMessageGet(SLAVE_ID_qHandle, 0);
	  if(idevt.status == osEventMessage)
	  {
		  idmsg = idevt.value.p;
		  for(int i =0;i<3;i++)
		  {
			  SLAVE_ID_RX[i] = idmsg->SLAVE_ID[i];
		  }
		  osPoolFree(slaveidpool, idmsg);
		  memset(DATA_FOR_SLAVE,0xFF,50);
		  DATA_FOR_SLAVE[0] = MASTER_BIT;
		  DATA_FOR_SLAVE[1] = SLAVE_ID_RX[0];
		  DATA_FOR_SLAVE[2] = ALL_OK;
		  CRC_VALUE_OF_SLAVE_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_SLAVE,49);
		  DATA_FOR_SLAVE[49] = CRC_VALUE_OF_SLAVE_DATA;
		  HAL_UART_Transmit(&huart5, DATA_FOR_SLAVE, 50,100);

		  notification_value = ulTaskNotifyTake(0,5000);
		  if(notification_value >0)
		  {
			  notification_value=0;
			  ID_S_PRESENT[0] = SLAVE_ID_RX[0];
			  memset(DATA_FOR_SLAVE,0xFF,50);
			  DATA_FOR_SLAVE[0] = MASTER_BIT;
			  DATA_FOR_SLAVE[1] = SLAVE_ID_RX[1];
			  DATA_FOR_SLAVE[2] = ALL_OK;
			  CRC_VALUE_OF_SLAVE_DATA = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_SLAVE,49);
			  DATA_FOR_SLAVE[49] = CRC_VALUE_OF_SLAVE_DATA;
			  HAL_UART_Transmit(&huart5, DATA_FOR_SLAVE, 50,100);

			  notification_value = ulTaskNotifyTake(0,5000);
			  if(notification_value >0)
			  {
				  notification_value=0;
				  ID_S_PRESENT[1] = SLAVE_ID_RX[1];
				  memset(DATA_FOR_SLAVE,0xFF,50);
				  DATA_FOR_SLAVE[0] = MASTER_BIT;
				  DATA_FOR_SLAVE[1] = SLAVE_ID_RX[2];
				  DATA_FOR_SLAVE[2] = ALL_OK;
				  CRC_VALUE_OF_SLAVE_DATA = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_SLAVE,49);
				  DATA_FOR_SLAVE[49] = CRC_VALUE_OF_SLAVE_DATA;
				  HAL_UART_Transmit(&huart5, DATA_FOR_SLAVE, 50,100);

				  notification_value = ulTaskNotifyTake(0,5000);
				  if(notification_value >0)
				  {
					  notification_value=0;
					  ID_S_PRESENT[2] = SLAVE_ID_RX[2];
					  memset(DATA_FOR_RCU,0xFF,BUFFER_SIZE_ALL);
					  NUMBER_OF_SYSTEMS = DATA_SIZE_FOR_4SYS;
					  DATA_FOR_RCU[0] = LORA_BIT;
					  DATA_FOR_RCU[1] = MODEM_REPLY_BIT;
					  DATA_FOR_RCU[2] = SEARCH_IDs;
					  DATA_FOR_RCU[3] = MY_TCU_ID;
					  DATA_FOR_RCU[4] = ID_S_PRESENT[0];
					  DATA_FOR_RCU[5] = ID_S_PRESENT[1];
					  DATA_FOR_RCU[6] = ID_S_PRESENT[2];
					  DATA_FOR_RCU[7] = DATA_SIZE_FOR_4SYS;
					  CRC_VALUE_FOR_LORA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_RCU, (BUFFER_SIZE_ALL-1));
					  DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_LORA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_FOR_RCU[i];
						  }
						  osMessagePut(LORA_SEND_qHandle, (uint32_t)lorasendmsg,100);
						  break;
					  case LOCAL_SYSTEM:
						  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
						  break;
					  }

				  }
				  else
				  {
					  memset(DATA_FOR_RCU,0xFF,BUFFER_SIZE_ALL);
					  NUMBER_OF_SYSTEMS = DATA_SIZE_FOR_3SYS;
					  DATA_FOR_RCU[0] = LORA_BIT;
					  DATA_FOR_RCU[1] = MODEM_REPLY_BIT;
					  DATA_FOR_RCU[2] = SEARCH_IDs;
					  DATA_FOR_RCU[3] = MY_TCU_ID;
					  DATA_FOR_RCU[4] = ID_S_PRESENT[0];
					  DATA_FOR_RCU[5] = ID_S_PRESENT[1];
					  DATA_FOR_RCU[6] = DUMMY;
					  DATA_FOR_RCU[7] = DATA_SIZE_FOR_3SYS;
					  CRC_VALUE_FOR_LORA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_RCU, (BUFFER_SIZE_ALL-1));
					  DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_LORA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_FOR_RCU[i];
						  }
						  osMessagePut(LORA_SEND_qHandle, (uint32_t)lorasendmsg,100);
						  break;
					  case LOCAL_SYSTEM:
						  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
						  break;
					  }
				  }
			  }
			  else
			  {
				  memset(DATA_FOR_SLAVE,0xFF,50);
				  DATA_FOR_SLAVE[0] = MASTER_BIT;
				  DATA_FOR_SLAVE[1] = SLAVE_ID_RX[2];
				  DATA_FOR_SLAVE[2] = ALL_OK;
				  CRC_VALUE_OF_SLAVE_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_SLAVE,49);
				  DATA_FOR_SLAVE[49] = CRC_VALUE_OF_SLAVE_DATA;
				  HAL_UART_Transmit(&huart5, DATA_FOR_SLAVE, 50,100);

				  notification_value = ulTaskNotifyTake(0,5000);
				  if(notification_value >0)
				  {
					  notification_value=0;
					  ID_S_PRESENT[1] = SLAVE_ID_RX[2];
					  memset(DATA_FOR_RCU,0xFF,BUFFER_SIZE_ALL);
					  NUMBER_OF_SYSTEMS = DATA_SIZE_FOR_3SYS;
					  DATA_FOR_RCU[0] = LORA_BIT;
					  DATA_FOR_RCU[1] = MODEM_REPLY_BIT;
					  DATA_FOR_RCU[2] = SEARCH_IDs;
					  DATA_FOR_RCU[3] = MY_TCU_ID;
					  DATA_FOR_RCU[4] = ID_S_PRESENT[0];
					  DATA_FOR_RCU[5] = ID_S_PRESENT[1];
					  DATA_FOR_RCU[6] = DUMMY;
					  DATA_FOR_RCU[7] = DATA_SIZE_FOR_3SYS;
					  CRC_VALUE_FOR_LORA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_RCU, (BUFFER_SIZE_ALL-1));
					  DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_LORA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_FOR_RCU[i];
						  }
						  osMessagePut(LORA_SEND_qHandle, (uint32_t)lorasendmsg,100);
						  break;
					  case LOCAL_SYSTEM:
						  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
						  break;
					  }
				  }
				  else
				  {
					  memset(DATA_FOR_RCU,0xFF,BUFFER_SIZE_ALL);
					  NUMBER_OF_SYSTEMS = DATA_SIZE_FOR_2SYS;
					  DATA_FOR_RCU[0] = LORA_BIT;
					  DATA_FOR_RCU[1] = MODEM_REPLY_BIT;
					  DATA_FOR_RCU[2] = SEARCH_IDs;
					  DATA_FOR_RCU[3] = MY_TCU_ID;
					  DATA_FOR_RCU[4] = ID_S_PRESENT[0];
					  DATA_FOR_RCU[5] = DUMMY;
					  DATA_FOR_RCU[6] = DUMMY;
					  DATA_FOR_RCU[7] = DATA_SIZE_FOR_2SYS;
					  CRC_VALUE_FOR_LORA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_RCU, (BUFFER_SIZE_ALL-1));
					  DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_LORA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_FOR_RCU[i];
						  }
						  osMessagePut(LORA_SEND_qHandle, (uint32_t)lorasendmsg,100);
						  break;
					  case LOCAL_SYSTEM:
						  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
						  break;
					  }
				  }
			  }
		  }
		  else
		  {
			  memset(DATA_FOR_SLAVE,0xFF,50);
			  DATA_FOR_SLAVE[0] = MASTER_BIT;
			  DATA_FOR_SLAVE[1] = SLAVE_ID_RX[1];
			  DATA_FOR_SLAVE[2] = ALL_OK;
			  CRC_VALUE_OF_SLAVE_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_SLAVE,49);
			  DATA_FOR_SLAVE[49] = CRC_VALUE_OF_SLAVE_DATA;
			  HAL_UART_Transmit(&huart5, DATA_FOR_SLAVE, 50,100);

			  notification_value = ulTaskNotifyTake(0,5000);
			  if(notification_value >0)
			  {
				  notification_value=0;
				  ID_S_PRESENT[0] = SLAVE_ID_RX[1];
				  memset(DATA_FOR_SLAVE,0xFF,50);
				  DATA_FOR_SLAVE[0] = MASTER_BIT;
				  DATA_FOR_SLAVE[1] = SLAVE_ID_RX[2];
				  DATA_FOR_SLAVE[2] = ALL_OK;
				  CRC_VALUE_OF_SLAVE_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_SLAVE,49);
				  DATA_FOR_SLAVE[49] = CRC_VALUE_OF_SLAVE_DATA;
				  HAL_UART_Transmit(&huart5, DATA_FOR_SLAVE, 50,100);
				  notification_value = ulTaskNotifyTake(0,5000);
				  if(notification_value >0)
				  {
					  notification_value=0;
					  ID_S_PRESENT[1] = SLAVE_ID_RX[2];
					  memset(DATA_FOR_RCU,0xFF,BUFFER_SIZE_ALL);
					  NUMBER_OF_SYSTEMS = DATA_SIZE_FOR_3SYS;
					  DATA_FOR_RCU[0] = LORA_BIT;
					  DATA_FOR_RCU[1] = MODEM_REPLY_BIT;
					  DATA_FOR_RCU[2] = SEARCH_IDs;
					  DATA_FOR_RCU[3] = MY_TCU_ID;
					  DATA_FOR_RCU[4] = ID_S_PRESENT[0];
					  DATA_FOR_RCU[5] = ID_S_PRESENT[1];
					  DATA_FOR_RCU[6] = DUMMY;
					  DATA_FOR_RCU[7] = DATA_SIZE_FOR_3SYS;
					  CRC_VALUE_FOR_LORA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_RCU, (BUFFER_SIZE_ALL-1));
					  DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_LORA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_FOR_RCU[i];
						  }
						  osMessagePut(LORA_SEND_qHandle, (uint32_t)lorasendmsg,100);
						  break;
					  case LOCAL_SYSTEM:
						  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
						  break;
					  }
				  }
				  else
				  {
					  memset(DATA_FOR_RCU,0xFF,BUFFER_SIZE_ALL);
					  NUMBER_OF_SYSTEMS = DATA_SIZE_FOR_2SYS;
					  DATA_FOR_RCU[0] = LORA_BIT;
					  DATA_FOR_RCU[1] = MODEM_REPLY_BIT;
					  DATA_FOR_RCU[2] = SEARCH_IDs;
					  DATA_FOR_RCU[3] = MY_TCU_ID;
					  DATA_FOR_RCU[4] = ID_S_PRESENT[0];
					  DATA_FOR_RCU[5] = DUMMY;
					  DATA_FOR_RCU[6] = DUMMY;
					  DATA_FOR_RCU[7] = DATA_SIZE_FOR_2SYS;
					  CRC_VALUE_FOR_LORA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_RCU, (BUFFER_SIZE_ALL-1));
					  DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_LORA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_FOR_RCU[i];
						  }
						  osMessagePut(LORA_SEND_qHandle, (uint32_t)lorasendmsg,100);
						  break;
					  case LOCAL_SYSTEM:
						  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
						  break;
					  }
				  }
			  }
			  else
			  {
				  memset(DATA_FOR_SLAVE,0xFF,50);
				  DATA_FOR_SLAVE[0] = MASTER_BIT;
				  DATA_FOR_SLAVE[1] = SLAVE_ID_RX[2];
				  DATA_FOR_SLAVE[2] = ALL_OK;
				  CRC_VALUE_OF_SLAVE_DATA = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_SLAVE,49);
				  DATA_FOR_SLAVE[49] = CRC_VALUE_OF_SLAVE_DATA;
				  HAL_UART_Transmit(&huart5, DATA_FOR_SLAVE, 50,100);

				  notification_value = ulTaskNotifyTake(0,5000);
				  if(notification_value >0)
				  {
					  notification_value=0;
					  ID_S_PRESENT[0] = SLAVE_ID_RX[2];
					  memset(DATA_FOR_RCU,0xFF,BUFFER_SIZE_ALL);
					  NUMBER_OF_SYSTEMS = DATA_SIZE_FOR_2SYS;
					  DATA_FOR_RCU[0] = LORA_BIT;
					  DATA_FOR_RCU[1] = MODEM_REPLY_BIT;
					  DATA_FOR_RCU[2] = SEARCH_IDs;
					  DATA_FOR_RCU[3] = MY_TCU_ID;
					  DATA_FOR_RCU[4] = ID_S_PRESENT[0];
					  DATA_FOR_RCU[5] = DUMMY;
					  DATA_FOR_RCU[6] = DUMMY;
					  DATA_FOR_RCU[7] = DATA_SIZE_FOR_2SYS;
					  CRC_VALUE_FOR_LORA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_RCU, (BUFFER_SIZE_ALL-1));
					  DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_LORA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_FOR_RCU[i];
						  }
						  osMessagePut(LORA_SEND_qHandle, (uint32_t)lorasendmsg,100);
						  break;
					  case LOCAL_SYSTEM:
						  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
						  break;
					  }
				  }
				  else
				  {
					  memset(DATA_FOR_RCU,0xFF,BUFFER_SIZE_ALL);
					  NUMBER_OF_SYSTEMS = DATA_SIZE_FOR_1SYS;
					  DATA_FOR_RCU[0] = LORA_BIT;
					  DATA_FOR_RCU[1] = MODEM_REPLY_BIT;
					  DATA_FOR_RCU[2] = SEARCH_IDs;
					  DATA_FOR_RCU[3] = MY_TCU_ID;
					  DATA_FOR_RCU[4] = 0x00;
					  DATA_FOR_RCU[5] = 0x00;
					  DATA_FOR_RCU[6] = 0x00;
					  DATA_FOR_RCU[7] = DATA_SIZE_FOR_1SYS;
					  CRC_VALUE_FOR_LORA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_RCU, (BUFFER_SIZE_ALL-1));
					  DATA_FOR_RCU[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_LORA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_FOR_RCU[i];
						  }
						  osMessagePut(LORA_SEND_qHandle, (uint32_t)lorasendmsg,100);
						  break;
					  case LOCAL_SYSTEM:
						  HAL_UART_Transmit(&huart6, DATA_FOR_RCU, 200,100);
						  break;
					  }
				  }
			  }
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END Start_MAS_SEND_SLAVE */
}

/* USER CODE BEGIN Header_Start_SLAVE_REC_MAS */
/**
* @brief Function implementing the SLAVE_REC_MAS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_SLAVE_REC_MAS */
void Start_SLAVE_REC_MAS(void const * argument)
{
  /* USER CODE BEGIN Start_SLAVE_REC_MAS */
	SYSTEM_PROCESS	*syspromsg;
	GIVE_ME	*givememsg;
	DATA_OF_SLAVE	*dataofslavemsg;
	DATA_LOG	*dlmsg;


	uint8_t DATA_OF_PROFILE[50];
	uint8_t DATA_FROM_MASTER[50];
	uint8_t DATA_FOR_MASTER[50];
	uint8_t DATA_FROM_SLAVE[50];
	uint8_t RECEIVED_CRC;
	uint8_t CALCULATED_CRC;
	uint8_t CRC_VALUE_FOR_MAS_DATA;
	static int notification =0;
	static int z =0;
  /* Infinite loop */
  for(;;)
  {
	  notification = ulTaskNotifyTake(0,portMAX_DELAY);
	  if( (notification>=0))
	  {
	  notification =0;
	  if(I_AM == I_AM_SLAVE)
	  {
	  memcpy(DATA_FROM_MASTER,DATA_FROM_MAS_SLAVE,sizeof(DATA_FROM_MASTER));
	  RECEIVED_CRC = DATA_FROM_MASTER[49];
	  CALCULATED_CRC = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FROM_MASTER,49);
	  if(RECEIVED_CRC == CALCULATED_CRC)
	  {
	  if(DATA_FROM_MASTER[0] == MASTER_BIT && DATA_FROM_MASTER[1] == MY_TCU_ID)
	  {
		  switch(DATA_FROM_MASTER[2])
		  {
		  case ALL_OK:
			  osThreadSuspend(LORA_RECEIVEHandle);
			  memset(DATA_FOR_MASTER,0xFF,50);
			  DATA_FOR_MASTER[0] = SLAVE_BIT;
			  DATA_FOR_MASTER[1] = MY_TCU_ID;
			  DATA_FOR_MASTER[2] = YES_OK;
			  CRC_VALUE_FOR_MAS_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_MASTER,49);
			  DATA_FOR_MASTER[49] = CRC_VALUE_FOR_MAS_DATA;
			  HAL_UART_Transmit(&huart5, DATA_FOR_MASTER,50,100);
			  break;
		  case GIVE_DATA:
			  givememsg = osPoolAlloc(givepool);
			  givememsg->GIVE_ME_t = I_AM_FOR_MASTER;
			  osMessagePut(GIVE_ME_qHandle, (uint32_t)givememsg,100);
			  break;
		  case GIVE_CURRENT:
			  dlmsg = osPoolAlloc(datalogpool);
			  dlmsg->NOTIFY = 0x01;
			  osMessagePut(DATA_LOG_qHandle,(uint32_t)dlmsg,100);
			  break;
		  case RESET_MAS_SLAVE:
			  osThreadResume(LORA_RECEIVEHandle);
			  break;
		  case FETCH_BIST:
			  memset(DATA_FOR_MASTER,0xFF,50);
			  DATA_FOR_MASTER[0] = SLAVE_BIT;
			  DATA_FOR_MASTER[1] = MY_TCU_ID;
			  DATA_FOR_MASTER[2] = FETCH_BIST;
			  DATA_FOR_MASTER[3] = RADIO_ERROR_DATA_t;
			  DATA_FOR_MASTER[4] = LORA_ERROR_DATA;
			  DATA_FOR_MASTER[5] = TCU_ERROR_DATA_t;
			  DATA_FOR_MASTER[6] = TCU_ERROR_DATA_FLASH;
			  DATA_FOR_MASTER[7] = TCU_ERROR_DATA_PWM;
			  DATA_FOR_MASTER[8] = SDA_ERROR_DATA_t;
			  DATA_FOR_MASTER[9] = SDA_ERROR_DATA;

			  CRC_VALUE_FOR_MAS_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FOR_MASTER,49);
			  DATA_FOR_MASTER[49] = CRC_VALUE_FOR_MAS_DATA;
			  HAL_UART_Transmit(&huart5, DATA_FOR_MASTER,50,100);
			  break;
		  case UPDATE_PROFILE:
			  	memcpy(DATA_OF_PROFILE,DATA_FROM_MASTER,50);
				for(int i=0;i<50;i++)
				{
					PROFILES[z][i] = DATA_OF_PROFILE[i];
				}
				z = z+1;
				if(z==50)
				{
					z=0;
				}
			  break;
		  }
		  if(((DATA_FROM_MASTER[2] == CLOSE_LOOP_START)||(DATA_FROM_MASTER[2] == UPADTE_TEMP_OR_POWER)||(DATA_FROM_MASTER[2] == OPEN_LOOP_START)||(DATA_FROM_MASTER[2] == SLEEP)||(DATA_FROM_MASTER[2] == KP_KI_KD_DATA)||(DATA_FROM_MASTER[3]==OPEN_LOOP_START_PROFILE)||(DATA_FROM_MASTER[3]==CLOSE_LOOP_START_PROFILE)))
		  {

			  syspromsg = osPoolAlloc(syspropool);
			  for(int i =0;i<50;i++)
			  {
				  syspromsg->DATA_TO_PROCESS[i] = DATA_FROM_MASTER[i];
			  }
			  osMessagePut(SYSTEM_PROCESS_qHandle, (uint32_t)syspromsg,100);
		  }
	  }
	  if((DATA_FROM_MASTER[0] == MASTER_BIT) && ((DATA_FROM_MASTER[2] == CLOSE_LOOP_START)||(DATA_FROM_MASTER[2] == OPEN_LOOP_START)||(DATA_FROM_MASTER[2] == SLEEP)||(DATA_FROM_MASTER[3]==OPEN_LOOP_START_PROFILE)||(DATA_FROM_MASTER[3]==CLOSE_LOOP_START_PROFILE)))
	  {

		  syspromsg = osPoolAlloc(syspropool);
		  for(int i =0;i<50;i++)
		  {
			  syspromsg->DATA_TO_PROCESS[i] = DATA_FROM_MASTER[i];
		  }
		  osMessagePut(SYSTEM_PROCESS_qHandle, (uint32_t)syspromsg,100);
	  }
	  }
	  }
	  if(I_AM == I_AM_MASTER)
	  {
		  memcpy(DATA_FROM_SLAVE,DATA_FROM_MAS_SLAVE,50);
		  RECEIVED_CRC = DATA_FROM_SLAVE[49];
		  CALCULATED_CRC = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_FROM_SLAVE,49);
		  if(RECEIVED_CRC == CALCULATED_CRC)
		  {
		  if(DATA_FROM_SLAVE[0] == SLAVE_BIT)
		  {
			  switch(DATA_FROM_SLAVE[2])
			  {
			  case YES_OK:
				  xTaskNotifyGive(MAS_SEND_SLAVEHandle);
				  break;
			  case FETCH_BIST:
				  BIST_DATA[0] = DATA_FROM_SLAVE[1];
				  BIST_DATA[1] = DATA_FROM_SLAVE[2];
				  BIST_DATA[2] = DATA_FROM_SLAVE[3];
				  BIST_DATA[3] = DATA_FROM_SLAVE[4];
				  BIST_DATA[4] = DATA_FROM_SLAVE[5];
				  BIST_DATA[5] = DATA_FROM_SLAVE[6];
				  BIST_DATA[6] = DATA_FROM_SLAVE[7];
				  BIST_DATA[7] = DATA_FROM_SLAVE[8];
				  BIST_DATA[8] = DATA_FROM_SLAVE[9];
				  break;
			  }
			  if((DATA_FROM_SLAVE[2] == DATA_TYPE_CURRENT)&& ((LOOP_RUNNING == OPEN_LOOP)||(LOOP_RUNNING == CLOSE_LOOP)))
			  {
				  dataofslavemsg = osPoolAlloc(dataofslavepool);
				  for(int i =0;i<50;i++)
				  {
					  dataofslavemsg->DATA_FROM_SLAVE_t[i] = DATA_FROM_SLAVE[i];
				  }
				  osMessagePut(DATA_OF_SLAVE_qHandle, (uint32_t)dataofslavemsg,100);
			  }
			  if((DATA_FROM_SLAVE[2] == SDA_TEMPERATURE)&& (LOOP_RUNNING == CLOSE_LOOP))
			  {
				  dataofslavemsg = osPoolAlloc(dataofslavepool);
				  for(int i =0;i<50;i++)
				  {
					  dataofslavemsg->DATA_FROM_SLAVE_t[i] = DATA_FROM_SLAVE[i];
				  }
				  osMessagePut(DATA_OF_SLAVE_qHandle, (uint32_t)dataofslavemsg,100);
			  }
		  }
		  }
	  }
	  HAL_UART_Receive_DMA(&huart5, DATA_FROM_MAS_SLAVE, 50);
	  __HAL_DMA_DISABLE_IT(&hdma_uart5_rx,DMA_IT_HT);
	  }

    osDelay(1);
  }
  /* USER CODE END Start_SLAVE_REC_MAS */
}

/* USER CODE BEGIN Header_Start_SDA_SEND */
/**
* @brief Function implementing the SDA_SEND thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_SDA_SEND */
void Start_SDA_SEND(void const * argument)
{
  /* USER CODE BEGIN Start_SDA_SEND */
	SDA_SEND *sdasend_msg;
	uint8_t SDA_DATA[5] ={0};
	osEvent evt;
  /* Infinite loop */
  for(;;)
  {
	  evt = osMessageGet(SDA_SEND_qHandle, osWaitForever);
	  if(evt.status == osEventMessage)
	  {
		  sdasend_msg = evt.value.p;
		  for(int i =0;i<5;i++)
		  {
			  SDA_DATA[i] = sdasend_msg->SDA_SEND_DATA[i];
		  }
		  osPoolFree(sdasendpool, sdasend_msg);
		  HAL_UART_Transmit(&huart1, SDA_DATA,5,10);
	  }
    osDelay(1);
  }
  /* USER CODE END Start_SDA_SEND */
}

/* USER CODE BEGIN Header_Start_SDA_RECEIVE */
/**
* @brief Function implementing the SDA_RECEIVE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_SDA_RECEIVE */
void Start_SDA_RECEIVE(void const * argument)
{
  /* USER CODE BEGIN Start_SDA_RECEIVE */
	DATA_PROCESS *datap_msg;

	uint8_t RECEIVED_CRC;
	uint8_t CALCULATED_CRC;
	uint8_t DATA_FROM_SDA_t[34];
	uint16_t SDA_TEMP[16];
	static int j =0;
	static int notification =0;
  /* Infinite loop */
  for(;;)
  {
	  notification = ulTaskNotifyTake(0,portMAX_DELAY);
	  if( (notification>=0))
	  {
	  notification =0;
	  SDA_ERROR_DATA=0;
	  xTimerReset(TCU_SDA_TIMERHandle,1000);
	  memcpy(DATA_FROM_SDA_t,DATA_FROM_SDA,34);
	  RECEIVED_CRC = DATA_FROM_SDA_t[33];
	  CALCULATED_CRC = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FROM_SDA_t,33);
	  if((DATA_FROM_SDA_t[32] == 0xDD)&&(RECEIVED_CRC==CALCULATED_CRC))
	  {
	  if(DATA_FROM_SDA_t[0]==0xFF)
	  {
		  osTimerStop(TCU_SDA_TIMERHandle);
	  }
	  j=0;
	  for(int i =0;i<16;i++)
		  {
			  SDA_TEMP[i] = (((((uint16_t)DATA_FROM_SDA_t[j])<<8)&0xFF00)|(((uint16_t)DATA_FROM_SDA_t[j+1])&0x00FF));
			  j =j+2;
			  if(j ==32)
			  {
				  j =0;
			  }
		  }
	  if(DATA_FROM_SDA_t[0]!=0xFF && LOOP_RUNNING == CLOSE_LOOP)
	  {
		  datap_msg = osPoolAlloc(datappool);
		  for(int i =0;i<16;i++)
		  {
			  datap_msg->DATA_PROCESS_DATA[i]=SDA_TEMP[i];
		  }
		  osMessagePut(DATA_PROCESS_qHandle, (uint32_t)datap_msg,100);
	  }
	  }
	  HAL_UART_Receive_DMA(&huart1, (uint8_t*)DATA_FROM_SDA, 34);
	  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx,DMA_IT_HT);
	  }

    osDelay(1);
  }
  /* USER CODE END Start_SDA_RECEIVE */
}

/* USER CODE BEGIN Header_Start_SYSTEM_PROCESS */
/**
* @brief Function implementing the SYSTEM_PROCESS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_SYSTEM_PROCESS */
void Start_SYSTEM_PROCESS(void const * argument)
{
  /* USER CODE BEGIN Start_SYSTEM_PROCESS */
	SYSTEM_PROCESS	*syspromsg;
	SDA_SEND *sda_msg;
	SDA_CONN	*connmsg;
	SETPOINT *sp_msg;
	KP_KI_KD *kpkikd_msg;
	PID_DATA	*pidmsg;
	FLASH_DATA	*flashmsg;

	osEvent sysproevt;
	uint8_t DATA_FOR_PROCESSING_t[50];
	uint16_t SETPOINT_t[16];
	static int j =4;
  /* Infinite loop */
  for(;;)
  {
	  sysproevt = osMessageGet(SYSTEM_PROCESS_qHandle, 0);
	  if(sysproevt.status == osEventMessage)
	  {

		  syspromsg = sysproevt.value.p;
		  memcpy(DATA_FOR_PROCESSING_t,syspromsg->DATA_TO_PROCESS,50);
		  osPoolFree(syspropool, syspromsg);
		  switch(DATA_FOR_PROCESSING_t[2])
		  {
		  case CLOSE_LOOP_START:
			  Running_Status = RUNNING;
			  LOOP_RUNNING = CLOSE_LOOP;
			  connmsg = osPoolAlloc(sdaconnpool);
			  connmsg->SDA_CONN_DATA[0] = 0x01;
			  osMessagePut(SDA_CONN_qHandle,(uint32_t) connmsg,100);
			  osThreadResume(VOLTAGE_TASKHandle);
			  osThreadResume(WHOLE_CURRENTHandle);
			  osThreadResume(CURRENT_TASKHandle);
			  osThreadResume(PID_TASKHandle);
			  osThreadResume(CURRENT_PROCESSHandle);
			  osThreadResume(DATA_LOGGINGHandle);
			  osThreadResume(WIFI_SENDHandle);
			  sda_msg = osPoolAlloc(sdasendpool);
			  sda_msg->SDA_SEND_DATA[0]=0xFF;
			  sda_msg->SDA_SEND_DATA[1]=DATA_FOR_PROCESSING_t[3];
			  osMessagePut(SDA_SEND_qHandle, (uint32_t)sda_msg,10);
			  osDelay(1);
			  START_CONDITION = DATA_FOR_PROCESSING_t[2];
			  MODE_CONDITION = DATA_FOR_PROCESSING_t[3];
			  j=4;
				  for(int i =0;i<16;i++)
				  {
					  SETPOINT_t[i] = (((((uint16_t)DATA_FOR_PROCESSING_t[j])<<8)&0xFF00)|((((uint16_t)DATA_FOR_PROCESSING_t[j+1]))&0x00FF));
					  j =j+2;
					  if(j ==36)
					  {
						  j =4;
					  }
				  }
				  sp_msg = osPoolAlloc(sppool);
				  for(int i =0;i<16;i++)
				  {
					  sp_msg->SETPOINT_t[i] = SETPOINT_t[i];
				  }
				  osMessagePut(SETPOINT_qHandle, (uint32_t)sp_msg,100);


			  break;
		  case UPADTE_TEMP_OR_POWER:
			  Running_Status = RUNNING;
			  j=4;
				  for(int i =0;i<16;i++)
				  {
					  SETPOINT_t[i] = (((((uint16_t)DATA_FOR_PROCESSING_t[j])<<8)&0xFF00)|((((uint16_t)DATA_FOR_PROCESSING_t[j+1]))&0x00FF));
					  j =j+2;
					  if(j ==36)
					  {
						  j =4;
					  }
				  }
				  sp_msg = osPoolAlloc(sppool);
				  for(int i =0;i<16;i++)
				  {
					  sp_msg->SETPOINT_t[i] = SETPOINT_t[i];
				  }
				  osMessagePut(SETPOINT_qHandle, (uint32_t)sp_msg,100);




		  	  break;
		  case OPEN_LOOP_START:
			  Running_Status = RUNNING;
			  LOOP_RUNNING = OPEN_LOOP;
			  connmsg = osPoolAlloc(sdaconnpool);
			  connmsg->SDA_CONN_DATA[0] = 0x01;
			  osMessagePut(SDA_CONN_qHandle,(uint32_t) connmsg,100);
			  osThreadResume(PID_TASKHandle);
			  osThreadResume(CURRENT_TASKHandle);
			  osThreadResume(CURRENT_PROCESSHandle);
			  osThreadResume(DATA_LOGGINGHandle);
			  START_CONDITION = DATA_FOR_PROCESSING_t[2];
			  MODE_CONDITION = DATA_FOR_PROCESSING_t[3];
			  j=4;
				  for(int i =0;i<16;i++)
				  {
					  SETPOINT_t[i] = (((((uint16_t)DATA_FOR_PROCESSING_t[j])<<8)&0xFF00)|((((uint16_t)DATA_FOR_PROCESSING_t[j+1]))&0x00FF));
					  j =j+2;
					  if(j ==36)
					  {
						  j =4;
					  }
				  }
				  sp_msg = osPoolAlloc(sppool);
				  for(int i =0;i<16;i++)
				  {
					  sp_msg->SETPOINT_t[i] = SETPOINT_t[i];
				  }
				  osMessagePut(SETPOINT_qHandle, (uint32_t)sp_msg,100);
			  break;

		  case KP_KI_KD_DATA:
			  	  	  	 kpkikd_msg = osPoolAlloc(kpkikdpool);
			  	  	  	 for(int i =0;i<4;i++)
			  	  	  	 {
			  	  	  	 kpkikd_msg->KP_KI_KD_t[i] = DATA_FOR_PROCESSING_t[i+3];
			  	  	  	 }
			  	  	  	 osMessagePut(KP_KI_KD_qHandle, (uint32_t)kpkikd_msg,100);
			  break;

		  case CLOSE_LOOP_START_PROFILE:
			  Running_Status = RUNNING;
				flashmsg = osPoolAlloc(flashpool);
				flashmsg->FLASH_DATA_t[0] = UPDATE_PROFILE;
				osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
				osDelay(3000);
				LOOP_RUNNING = CLOSE_LOOP;
				  connmsg = osPoolAlloc(sdaconnpool);
				  connmsg->SDA_CONN_DATA[0] = 0x01;
				  osMessagePut(SDA_CONN_qHandle,(uint32_t) connmsg,100);
				  osThreadResume(VOLTAGE_TASKHandle);
				  osThreadResume(WHOLE_CURRENTHandle);
				  osThreadResume(CURRENT_TASKHandle);
				  osThreadResume(PID_TASKHandle);
				  osThreadResume(CURRENT_PROCESSHandle);
				  osThreadResume(DATA_LOGGINGHandle);
				  osThreadResume(WIFI_SENDHandle);
				  sda_msg = osPoolAlloc(sdasendpool);
				  sda_msg->SDA_SEND_DATA[0]=0xFF;
				  sda_msg->SDA_SEND_DATA[1]=DATA_FOR_PROCESSING_t[3];
				  osMessagePut(SDA_SEND_qHandle, (uint32_t)sda_msg,10);
				  osDelay(1);
				  START_CONDITION = DATA_FOR_PROCESSING_t[2];
				  MODE_CONDITION = DATA_FOR_PROCESSING_t[3];
			  break;
		  case OPEN_LOOP_START_PROFILE:
			  Running_Status = RUNNING;
				flashmsg = osPoolAlloc(flashpool);
				flashmsg->FLASH_DATA_t[0] = UPDATE_PROFILE;
				osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
				osDelay(3000);
				  LOOP_RUNNING = OPEN_LOOP;
				  connmsg = osPoolAlloc(sdaconnpool);
				  connmsg->SDA_CONN_DATA[0] = 0x01;
				  osMessagePut(SDA_CONN_qHandle,(uint32_t) connmsg,100);
				  osThreadResume(PID_TASKHandle);
				  osThreadResume(CURRENT_TASKHandle);
				  osThreadResume(CURRENT_PROCESSHandle);
				  osThreadResume(DATA_LOGGINGHandle);
				  START_CONDITION = DATA_FOR_PROCESSING_t[2];
				  MODE_CONDITION = DATA_FOR_PROCESSING_t[3];
			  break;
		  case SLEEP:
			  Running_Status = RUNNING;
			  	  	osThreadSuspend(PID_TASKHandle);
			  	    osThreadSuspend(CURRENT_TASKHandle);
			  	    osThreadSuspend(CURRENT_PROCESSHandle);
			  	    osThreadSuspend(DATA_LOGGINGHandle);
			  	    osThreadSuspend(WIFI_SENDHandle);
			  	    pidmsg = osPoolAlloc(pidpool);
			  	    for(int i =0;i<16;i++)
			  	    {
			  	    pidmsg->PID_DATA_t[i] = 0;
			  	    }
			  	    osMessagePut(PID_DATA_qHandle, (uint32_t)pidmsg, 100);

			  	  	PCA9685_SleepMode(1);
			  	  	SEND_ACK();
			  break;
		  }
			flashmsg = osPoolAlloc(flashpool);
			flashmsg->FLASH_DATA_t[0] = SYSTEM_STATE;
			flashmsg->FLASH_DATA_t[1] = START_CONDITION;
			flashmsg->FLASH_DATA_t[2] = MODE_CONDITION;
			  for(int i =0;i<50;i++)
			  {
				  flashmsg->FLASH_DATA_t[i+3]=DATA_FOR_PROCESSING_t[i];
			  }
			osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
	  }
    osDelay(1);
  }
  /* USER CODE END Start_SYSTEM_PROCESS */
}

/* USER CODE BEGIN Header_Start_DATA_PROCESS */
/**
* @brief Function implementing the DATA_PROCESS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_DATA_PROCESS */
void Start_DATA_PROCESS(void const * argument)
{
  /* USER CODE BEGIN Start_DATA_PROCESS */
	DATA_PROCESS *datap_msg;
	GIVE_ME	*givememsg;
	FEEDBACK	*feedbackmsg;
	DATA_LOGGING	*datalogmsg;
	DATA_ACCU	*dataaccumsg;
	WIFI_TEMP *wifitempmsg;
	WIFI_CURR	*wificurrmsg;

	osEvent dataprocessevt,givemeevt,dataaccuevt;

	uint8_t DATA_FOR_MASTER[50];
	uint8_t CRC_VALUE_FOR_MASTER;
	uint8_t DATA_IS_FOR;
	uint8_t SDA_TEMP_8BIT[32];
	uint8_t CURRENT_8BIT[32];
	uint16_t DATA_TO_PROCESS[16] = {0};
	static int k =0;
	static uint8_t DATA_FOR_DATA_LOGG[50];
	static int state = 0,mstate =0;
	static uint8_t SYSTEM_VOLTAGE_tt;
	static uint8_t SYSTEM_CURRENT_tt;
	static uint8_t AMBIENT_TEMPERATURE_tt;
	static uint8_t IR_TEMPERATURE_tt;
	static uint8_t LEFT_HS_TEMPERATURE_tt;
	static uint8_t RIGHT_HS_TEMPERATURE_tt;
	static uint8_t RSSI_tt;
	static uint8_t ERROR_tt;
  /* Infinite loop */
  for(;;)
  {
	  dataprocessevt = osMessageGet(DATA_PROCESS_qHandle, 0);
	  if(dataprocessevt.status == osEventMessage)
	  {
		  datap_msg = dataprocessevt.value.p;
		  for(int i =0;i<16;i++)
		  {
			  DATA_TO_PROCESS[i] = datap_msg->DATA_PROCESS_DATA[i];
		  }
		  osPoolFree(datappool, datap_msg);
	  }

	  dataaccuevt = osMessageGet(DATA_ACCU_qHandle, 0);
	  if(dataaccuevt.status == osEventMessage)
	  {
		  dataaccumsg = dataaccuevt.value.p;
		  SYSTEM_VOLTAGE_tt = dataaccumsg->SYSTEM_VOLTAGE_t;
		  SYSTEM_CURRENT_tt = dataaccumsg->SYSTEM_CURRENT_t;
		  AMBIENT_TEMPERATURE_tt = dataaccumsg->AMBIENT_TEMPERATURE_t;
		  IR_TEMPERATURE_tt = dataaccumsg->IR_TEMPERATURE_t;
		  LEFT_HS_TEMPERATURE_tt = dataaccumsg->LEFT_HS_TEMPERATURE_t;
		  RIGHT_HS_TEMPERATURE_tt = dataaccumsg->RIGHT_HS_TEMPERATURE_t;
		  RSSI_tt = dataaccumsg->RSSI_t;
		  ERROR_tt = dataaccumsg->ERROR_t;
		  osPoolFree(dataaccupool, dataaccumsg);
	  }

	  givemeevt = osMessageGet(GIVE_ME_qHandle, 0);
	  if(givemeevt.status == osEventMessage)
	  {
		  givememsg = givemeevt.value.p;
		  DATA_IS_FOR = givememsg->GIVE_ME_t;
		  osPoolFree(givepool, givememsg);
		  if(LOOP_RUNNING == OPEN_LOOP)
		  {
		  switch(DATA_IS_FOR)
		  {
		  	  case	I_AM_WIFI:
			  	  	k=0;
					for(int i =0;i<16;i++)
					{
					CURRENT_8BIT[k] = (uint8_t)((DATA_TO_PROCESS[i]>>8)&0x00FF);
					CURRENT_8BIT[k+1] = (uint8_t)(DATA_TO_PROCESS[i]&0x00FF);
					k = k+2;
					if(k==32)
					{
					k=0;
					}
					}
					switch(state)
					{
					case 0:
						state =1;
						DATA_FOR_DATA_LOGG[0] = MY_TCU_ID;
						DATA_FOR_DATA_LOGG[1] = DATA_TYPE_CURRENT;
						memcpy(&DATA_FOR_DATA_LOGG[2],CURRENT_8BIT,32);
						DATA_FOR_DATA_LOGG[34] = SYS_VOLTAGE;
						DATA_FOR_DATA_LOGG[35] = SYSTEM_VOLTAGE_tt;
						DATA_FOR_DATA_LOGG[36] = WHOLE_CURRENT;
						DATA_FOR_DATA_LOGG[37] = SYSTEM_CURRENT_tt;
						DATA_FOR_DATA_LOGG[38] = RSSI;
						DATA_FOR_DATA_LOGG[39] = RSSI_tt;
						DATA_FOR_DATA_LOGG[40] = ERRORS;
						DATA_FOR_DATA_LOGG[41] = ERROR_tt;
						DATA_FOR_DATA_LOGG[42] = START_STOP_STATE;
						DATA_FOR_DATA_LOGG[43] = START_CONDITION;
						DATA_FOR_DATA_LOGG[44] = MODE_CONDITION;

						wificurrmsg = osMailAlloc(wificurrmail,100);
						memcpy(wificurrmsg->CURR_DATA_FOR_WIFI,DATA_FOR_DATA_LOGG,50);
						osMailPut(wificurrmail, wificurrmsg);
						break;
					case 1:
						state =0;
						DATA_FOR_DATA_LOGG[0] = MY_TCU_ID;
						DATA_FOR_DATA_LOGG[1] = DATA_TYPE_CURRENT;
						memcpy(&DATA_FOR_DATA_LOGG[2],CURRENT_8BIT,32);
						DATA_FOR_DATA_LOGG[34] = AMB_TEMP_t;
						DATA_FOR_DATA_LOGG[35] = AMBIENT_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[36] = IR_TEMP_t;
						DATA_FOR_DATA_LOGG[37] = IR_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[38] = HS_LEFT_t;
						DATA_FOR_DATA_LOGG[39] = LEFT_HS_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[40] = HS_RIGHT_t;
						DATA_FOR_DATA_LOGG[41] = RIGHT_HS_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[42] = START_STOP_STATE;
						DATA_FOR_DATA_LOGG[43] = START_CONDITION;
						wificurrmsg = osMailAlloc(wificurrmail,100);
						memcpy(wificurrmsg->CURR_DATA_FOR_WIFI,DATA_FOR_DATA_LOGG,50);
						osMailPut(wificurrmail, wificurrmsg);
						break;
					}
			  break;
		  	  case	I_AM_DATA_LOGG:
			  	  	k=0;
					for(int i =0;i<16;i++)
					{
					CURRENT_8BIT[k] = (uint8_t)((DATA_TO_PROCESS[i]>>8)&0x00FF);
					CURRENT_8BIT[k+1] = (uint8_t)(DATA_TO_PROCESS[i]&0x00FF);
					k = k+2;
					if(k==32)
					{
					k=0;
					}
					}
					switch(state)
					{
					case 0:
						state =1;
						DATA_FOR_DATA_LOGG[0] = MY_TCU_ID;
						DATA_FOR_DATA_LOGG[1] = DATA_TYPE_CURRENT;
//						for(int i =0;i<32;i++)
//						{
//							DATA_FOR_DATA_LOGG[i+2] = CURRENT_8BIT[i];
//						}
						memcpy(&DATA_FOR_DATA_LOGG[2],CURRENT_8BIT,32);
						DATA_FOR_DATA_LOGG[34] = SYS_VOLTAGE;
						DATA_FOR_DATA_LOGG[35] = SYSTEM_VOLTAGE_tt;
						DATA_FOR_DATA_LOGG[36] = WHOLE_CURRENT;
						DATA_FOR_DATA_LOGG[37] = SYSTEM_CURRENT_tt;
						DATA_FOR_DATA_LOGG[38] = RSSI;
						DATA_FOR_DATA_LOGG[39] = RSSI_tt;
						DATA_FOR_DATA_LOGG[40] = ERRORS;
						DATA_FOR_DATA_LOGG[41] = ERROR_tt;
						DATA_FOR_DATA_LOGG[42] = START_STOP_STATE;
						DATA_FOR_DATA_LOGG[43] = START_CONDITION;
						DATA_FOR_DATA_LOGG[44] = MODE_CONDITION;
						datalogmsg = osPoolAlloc(datalogpool);

//						for(int i=0;i<50;i++)
//						{
//							datalogmsg->DATA_LOGGING_DATA[i] = DATA_FOR_DATA_LOGG[i];
//
//						}
						memcpy(datalogmsg->DATA_LOGGING_DATA,DATA_FOR_DATA_LOGG,50);
						osMessagePut(DATA_LOGGING_qHandle, (uint32_t)datalogmsg, 100);


						break;
					case 1:
						state =0;
						DATA_FOR_DATA_LOGG[0] = MY_TCU_ID;
						DATA_FOR_DATA_LOGG[1] = DATA_TYPE_CURRENT;
//						for(int i =0;i<32;i++)
//						{
//							DATA_FOR_DATA_LOGG[i+2] = CURRENT_8BIT[i];
//						}
						memcpy(&DATA_FOR_DATA_LOGG[2],CURRENT_8BIT,32);
						DATA_FOR_DATA_LOGG[34] = AMB_TEMP_t;
						DATA_FOR_DATA_LOGG[35] = AMBIENT_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[36] = IR_TEMP_t;
						DATA_FOR_DATA_LOGG[37] = IR_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[38] = HS_LEFT_t;
						DATA_FOR_DATA_LOGG[39] = LEFT_HS_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[40] = HS_RIGHT_t;
						DATA_FOR_DATA_LOGG[41] = RIGHT_HS_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[42] = START_STOP_STATE;
						DATA_FOR_DATA_LOGG[43] = START_CONDITION;
						datalogmsg = osPoolAlloc(datalogpool);

//						for(int i=0;i<50;i++)
//						{
//							datalogmsg->DATA_LOGGING_DATA[i] = DATA_FOR_DATA_LOGG[i];
//
//						}
						memcpy(datalogmsg->DATA_LOGGING_DATA,DATA_FOR_DATA_LOGG,50);
						osMessagePut(DATA_LOGGING_qHandle, (uint32_t)datalogmsg, 100);

						break;
					}
		  	      break;
		  	  case I_AM_FOR_MASTER:
			  	  	k=0;
					for(int i =0;i<16;i++)
					{
					CURRENT_8BIT[k] = (uint8_t)((DATA_TO_PROCESS[i]>>8)&0x00FF);
					CURRENT_8BIT[k+1] = (uint8_t)(DATA_TO_PROCESS[i]&0x00FF);
					k = k+2;
					if(k==32)
					{
					k=0;
					}
					}
					switch(mstate)
					{
					case 0:
						mstate =1;
						  DATA_FOR_MASTER[0] = SLAVE_BIT;
						  DATA_FOR_MASTER[1] = MY_TCU_ID;
						  DATA_FOR_MASTER[2] = DATA_TYPE_CURRENT;
							for(int i =0;i<32;i++)
							{
								DATA_FOR_MASTER[i+3] = CURRENT_8BIT[i];
							}
							DATA_FOR_MASTER[35] = AMB_TEMP_t;
							DATA_FOR_MASTER[36] = AMBIENT_TEMPERATURE_tt;
							DATA_FOR_MASTER[37] = IR_TEMP_t;
							DATA_FOR_MASTER[38] = IR_TEMPERATURE_tt;
							DATA_FOR_MASTER[39] = HS_LEFT_t;
							DATA_FOR_MASTER[40] = LEFT_HS_TEMPERATURE_tt;
							DATA_FOR_MASTER[41] = HS_RIGHT_t;
							DATA_FOR_MASTER[42] = RIGHT_HS_TEMPERATURE_tt;
						  CRC_VALUE_FOR_MASTER = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_MASTER,49);
						  DATA_FOR_MASTER[49] = CRC_VALUE_FOR_MASTER;
						  HAL_UART_Transmit(&huart5, DATA_FOR_MASTER, 50,100);

						break;
					case 1:
						mstate =0;
						  DATA_FOR_MASTER[0] = SLAVE_BIT;
						  DATA_FOR_MASTER[1] = MY_TCU_ID;
						  DATA_FOR_MASTER[2] = DATA_TYPE_CURRENT;
							for(int i =0;i<32;i++)
							{
								DATA_FOR_MASTER[i+3] = CURRENT_8BIT[i];
							}
							DATA_FOR_DATA_LOGG[35] = SYS_VOLTAGE;
							DATA_FOR_DATA_LOGG[36] = SYSTEM_VOLTAGE_tt;
							DATA_FOR_DATA_LOGG[37] = WHOLE_CURRENT;
							DATA_FOR_DATA_LOGG[38] = SYSTEM_CURRENT_tt;
							DATA_FOR_DATA_LOGG[39] = RSSI;
							DATA_FOR_DATA_LOGG[40] = RSSI_tt;
							DATA_FOR_DATA_LOGG[41] = ERRORS;
							DATA_FOR_DATA_LOGG[42] = ERROR_tt;
						  CRC_VALUE_FOR_MASTER = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_MASTER,49);
						  DATA_FOR_MASTER[49] = CRC_VALUE_FOR_MASTER;
						  HAL_UART_Transmit(&huart5, DATA_FOR_MASTER, 50,100);

						break;
					}

		  		  break;
		  }
		  }
		  if(LOOP_RUNNING == CLOSE_LOOP)
		  {
		  switch(DATA_IS_FOR)
		  {
		  	  case	I_AM_PID:
		  		  feedbackmsg = osPoolAlloc(fbpool);
		  		  for(int i =0;i<16;i++)
		  		  {
		  			  feedbackmsg->FEEDBACK_t[i] = DATA_TO_PROCESS[i];
		  		  }
		  		  osMessagePut(FEEDBACK_qHandle,(uint32_t) feedbackmsg,100);
			  break;
		  	  case	I_AM_WIFI:
		  		k=0;
		  						for(int i =0;i<16;i++)
		  						{
		  						SDA_TEMP_8BIT[k] = (uint8_t)((DATA_TO_PROCESS[i]>>8)&0x00FF);
		  						SDA_TEMP_8BIT[k+1] = (uint8_t)(DATA_TO_PROCESS[i]&0x00FF);
		  						k = k+2;
		  						if(k==32)
		  						{
		  						k=0;
		  						}
		  						}
		  						switch(state)
		  						{
		  						case 0:
		  							state =1;
		  							DATA_FOR_DATA_LOGG[0] = MY_TCU_ID;
		  							DATA_FOR_DATA_LOGG[1] = SDA_TEMPERATURE;
		  							for(int i =0;i<32;i++)
		  							{
		  								DATA_FOR_DATA_LOGG[i+2] = SDA_TEMP_8BIT[i];
		  							}
		  							DATA_FOR_DATA_LOGG[34] = SYS_VOLTAGE;
		  							DATA_FOR_DATA_LOGG[35] = SYSTEM_VOLTAGE_tt;
		  							DATA_FOR_DATA_LOGG[36] = WHOLE_CURRENT;
		  							DATA_FOR_DATA_LOGG[37] = SYSTEM_CURRENT_tt;
		  							DATA_FOR_DATA_LOGG[38] = RSSI;
		  							DATA_FOR_DATA_LOGG[39] = RSSI_tt;
		  							DATA_FOR_DATA_LOGG[40] = ERRORS;
		  							DATA_FOR_DATA_LOGG[41] = ERROR_tt;
		  							DATA_FOR_DATA_LOGG[42] = START_STOP_STATE;
		  							DATA_FOR_DATA_LOGG[43] = START_CONDITION;
		  							DATA_FOR_DATA_LOGG[44] = MODE_CONDITION;

		  							wifitempmsg = osMailAlloc(wifitempmail,100);
		  							for(int i=0;i<50;i++)
		  							{

		  								wifitempmsg->TEMP_DATA_FOR_WIFI[i] = DATA_FOR_DATA_LOGG[i];
		  							}

		  							osMailPut(wifitempmail, wifitempmsg);

		  							break;
		  						case 1:
		  							state =0;
		  							DATA_FOR_DATA_LOGG[0] = MY_TCU_ID;
		  							DATA_FOR_DATA_LOGG[1] = SDA_TEMPERATURE;
		  							for(int i =0;i<32;i++)
		  							{
		  								DATA_FOR_DATA_LOGG[i+2] = SDA_TEMP_8BIT[i];
		  							}
		  							DATA_FOR_DATA_LOGG[34] = AMB_TEMP_t;
		  							DATA_FOR_DATA_LOGG[35] = AMBIENT_TEMPERATURE_tt;
		  							DATA_FOR_DATA_LOGG[36] = IR_TEMP_t;
		  							DATA_FOR_DATA_LOGG[37] = IR_TEMPERATURE_tt;
		  							DATA_FOR_DATA_LOGG[38] = HS_LEFT_t;
		  							DATA_FOR_DATA_LOGG[39] = LEFT_HS_TEMPERATURE_tt;
		  							DATA_FOR_DATA_LOGG[40] = HS_RIGHT_t;
		  							DATA_FOR_DATA_LOGG[41] = RIGHT_HS_TEMPERATURE_tt;
		  							DATA_FOR_DATA_LOGG[42] = START_STOP_STATE;
		  							DATA_FOR_DATA_LOGG[43] = START_CONDITION;
		  							DATA_FOR_DATA_LOGG[44] = MODE_CONDITION;


		  							wifitempmsg = osMailAlloc(wifitempmail,100);
		  							for(int i=0;i<50;i++)
		  							{

		  								wifitempmsg->TEMP_DATA_FOR_WIFI[i] = DATA_FOR_DATA_LOGG[i];
		  							}

		  							osMailPut(wifitempmail, wifitempmsg);

		  							break;
		  						}
		  		  break;
		  	  case	I_AM_DATA_LOGG:
			  	  	k=0;
					for(int i =0;i<16;i++)
					{
					SDA_TEMP_8BIT[k] = (uint8_t)((DATA_TO_PROCESS[i]>>8)&0x00FF);
					SDA_TEMP_8BIT[k+1] = (uint8_t)(DATA_TO_PROCESS[i]&0x00FF);
					k = k+2;
					if(k==32)
					{
					k=0;
					}
					}
					switch(state)
					{
					case 0:
						state =1;
						DATA_FOR_DATA_LOGG[0] = MY_TCU_ID;
						DATA_FOR_DATA_LOGG[1] = SDA_TEMPERATURE;
						for(int i =0;i<32;i++)
						{
							DATA_FOR_DATA_LOGG[i+2] = SDA_TEMP_8BIT[i];
						}
						DATA_FOR_DATA_LOGG[34] = SYS_VOLTAGE;
						DATA_FOR_DATA_LOGG[35] = SYSTEM_VOLTAGE_tt;
						DATA_FOR_DATA_LOGG[36] = WHOLE_CURRENT;
						DATA_FOR_DATA_LOGG[37] = SYSTEM_CURRENT_tt;
						DATA_FOR_DATA_LOGG[38] = RSSI;
						DATA_FOR_DATA_LOGG[39] = RSSI_tt;
						DATA_FOR_DATA_LOGG[40] = ERRORS;
						DATA_FOR_DATA_LOGG[41] = ERROR_tt;
						DATA_FOR_DATA_LOGG[42] = START_STOP_STATE;
						DATA_FOR_DATA_LOGG[43] = START_CONDITION;
						DATA_FOR_DATA_LOGG[44] = MODE_CONDITION;
						datalogmsg = osPoolAlloc(datalogpool);

						for(int i=0;i<50;i++)
						{
							datalogmsg->DATA_LOGGING_DATA[i] = DATA_FOR_DATA_LOGG[i];

						}
						osMessagePut(DATA_LOGGING_qHandle, (uint32_t)datalogmsg, 100);


						break;
					case 1:
						state =0;
						DATA_FOR_DATA_LOGG[0] = MY_TCU_ID;
						DATA_FOR_DATA_LOGG[1] = SDA_TEMPERATURE;
						for(int i =0;i<32;i++)
						{
							DATA_FOR_DATA_LOGG[i+2] = SDA_TEMP_8BIT[i];
						}
						DATA_FOR_DATA_LOGG[34] = AMB_TEMP_t;
						DATA_FOR_DATA_LOGG[35] = AMBIENT_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[36] = IR_TEMP_t;
						DATA_FOR_DATA_LOGG[37] = IR_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[38] = HS_LEFT_t;
						DATA_FOR_DATA_LOGG[39] = LEFT_HS_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[40] = HS_RIGHT_t;
						DATA_FOR_DATA_LOGG[41] = RIGHT_HS_TEMPERATURE_tt;
						DATA_FOR_DATA_LOGG[42] = START_STOP_STATE;
						DATA_FOR_DATA_LOGG[43] = START_CONDITION;
						DATA_FOR_DATA_LOGG[44] = MODE_CONDITION;
						datalogmsg = osPoolAlloc(datalogpool);
						datalogmsg = osPoolAlloc(datalogpool);

						for(int i=0;i<50;i++)
						{
							datalogmsg->DATA_LOGGING_DATA[i] = DATA_FOR_DATA_LOGG[i];

						}
						osMessagePut(DATA_LOGGING_qHandle, (uint32_t)datalogmsg, 100);


						break;
					}
		  	  break;
		  	  case I_AM_FOR_MASTER:
			  	  	k=0;
					for(int i =0;i<16;i++)
					{
					SDA_TEMP_8BIT[k] = (uint8_t)((DATA_TO_PROCESS[i]>>8)&0x00FF);
					SDA_TEMP_8BIT[k+1] = (uint8_t)(DATA_TO_PROCESS[i]&0x00FF);
					k = k+2;
					if(k==32)
					{
					k=0;
					}
					}
					switch(mstate)
					{
					case 0:
						mstate =1;
						  DATA_FOR_MASTER[0] = SLAVE_BIT;
						  DATA_FOR_MASTER[1] = MY_TCU_ID;
						  DATA_FOR_MASTER[2] = SDA_TEMPERATURE;
							for(int i =0;i<32;i++)
							{
								DATA_FOR_MASTER[i+3] = SDA_TEMP_8BIT[i];
							}
							DATA_FOR_MASTER[35] = AMB_TEMP_t;
							DATA_FOR_MASTER[36] = AMBIENT_TEMPERATURE_tt;
							DATA_FOR_MASTER[37] = IR_TEMP_t;
							DATA_FOR_MASTER[38] = IR_TEMPERATURE_tt;
							DATA_FOR_MASTER[39] = HS_LEFT_t;
							DATA_FOR_MASTER[40] = LEFT_HS_TEMPERATURE_tt;
							DATA_FOR_MASTER[41] = HS_RIGHT_t;
							DATA_FOR_MASTER[42] = RIGHT_HS_TEMPERATURE_tt;
						  CRC_VALUE_FOR_MASTER = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_MASTER,49);
						  DATA_FOR_MASTER[49] = CRC_VALUE_FOR_MASTER;
						  HAL_UART_Transmit(&huart5, DATA_FOR_MASTER, 50,100);

						break;
					case 1:
						mstate =0;
						  DATA_FOR_MASTER[0] = SLAVE_BIT;
						  DATA_FOR_MASTER[1] = MY_TCU_ID;
						  DATA_FOR_MASTER[2] = SDA_TEMPERATURE;
							for(int i =0;i<32;i++)
							{
								DATA_FOR_MASTER[i+3] = SDA_TEMP_8BIT[i];
							}
							DATA_FOR_MASTER[35] = SYS_VOLTAGE;
							DATA_FOR_MASTER[36] = SYSTEM_VOLTAGE_tt;
							DATA_FOR_MASTER[37] = WHOLE_CURRENT;
							DATA_FOR_MASTER[38] = SYSTEM_CURRENT_tt;
							DATA_FOR_MASTER[39] = RSSI;
							DATA_FOR_MASTER[40] = RSSI_tt;
							DATA_FOR_MASTER[41] = ERRORS;
							DATA_FOR_MASTER[42] = ERROR_tt;
						  CRC_VALUE_FOR_MASTER = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_MASTER,49);
						  DATA_FOR_MASTER[49] = CRC_VALUE_FOR_MASTER;
						  HAL_UART_Transmit(&huart5, DATA_FOR_MASTER, 50,100);

						break;
					}
		  		 // HAL_UART_Transmit(&huart5, DATA_FOR_MASTER, 50,100);
		  		  break;
		  }
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END Start_DATA_PROCESS */
}

/* USER CODE BEGIN Header_Start_RCU_RECEIVE */
/**
* @brief Function implementing the RCU_RECEIVE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_RCU_RECEIVE */
void Start_RCU_RECEIVE(void const * argument)
{
  /* USER CODE BEGIN Start_RCU_RECEIVE */
	RTC_TimeTypeDef sTime;
	RTC_DateTypeDef sDate;

	SYSTEM_PROCESS	*syspromsg;
	SLAVE_SEND	*slavemsg;
	SLAVE_ID *idmsg;
	KP_KI_KD *kpkikd_msg;
	CURRENT_LIMIT_Q *currlimitmsg;
	FLASH_READ	*flashreadmsg;
	SDA_SEND *sda_msg;
	FLASH_DATA	*flashmsg;

	uint8_t DATA_FROM_LOCAL_t[50];
	uint8_t DATA_SENDING_TO_RCU_t[MAX_BUFFER_SIZE];
	uint8_t CRC_VALUE_OF_DATA_FOR_RCU_t;
	uint8_t RECEIVED_CRC;
	uint8_t CALCULATED_CRC;
	uint8_t DATA_FOR_PROCESSING[50];
	uint8_t ID_S[3];
	uint8_t DATA_OF_PROFILE[50];
	uint8_t Sperading_Factor;
    uint8_t CR;
	uint16_t Band_Width;
	uint32_t frequency_t;
	uint32_t offset_t;
    int pos =0;
	static int state=0;
    char DATA_FROM_FLASH[200];
    static uint32_t clear_address = STARTING_ADD_OF_ENGG_MODE;
    static int notification =0;
    static int z=0;
  /* Infinite loop */
  for(;;)
  {
	  notification = ulTaskNotifyTake(0,portMAX_DELAY);
	  if( (notification>=0))
	  {
	  notification =0;
	  HAL_GPIO_TogglePin(LED_YELLOW_GPIO_Port, LED_YELLOW_Pin);
	  memcpy(DATA_FROM_LOCAL_t,DATA_FROM_LOCAL,50);
	  if(DATA_FROM_LOCAL_t[1] == MODEM_BIT)
	  {
		  RECEIVED_CRC = DATA_FROM_LOCAL_t[LORA_RX_BUFF_SIZE-1];
		  CALCULATED_CRC = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FROM_LOCAL_t,(LORA_RX_BUFF_SIZE-1));
		  if(RECEIVED_CRC == CALCULATED_CRC)
		  {
			  if((DATA_FROM_LOCAL_t[3] == SEARCHING_DEVICES) &&(DATA_FROM_LOCAL_t[4]!=MY_TCU_ID)&&(DATA_FROM_LOCAL_t[5]!=MY_TCU_ID)&&(DATA_FROM_LOCAL_t[6]!=MY_TCU_ID)&&(DATA_FROM_LOCAL_t[7]!=MY_TCU_ID))
			  {
				  WHOM_TO_GIVE_DATA = LOCAL_SYSTEM;
				  state = SEARCHING_DEVICES;
				  sDate.Date = DATA_FROM_LOCAL_t[10];
				  sDate.Month = DATA_FROM_LOCAL_t[11];
				  sDate.Year = DATA_FROM_LOCAL_t[12];
				  sTime.Hours = DATA_FROM_LOCAL_t[13];
				  sTime.Minutes = DATA_FROM_LOCAL_t[14];
				  sTime.Seconds = DATA_FROM_LOCAL_t[15];
				  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
				  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
				  osThreadSuspend(LORA_RECEIVEHandle);
			  }
				else if(((DATA_FROM_LOCAL_t[3] == CLOSE_LOOP_START)||(DATA_FROM_LOCAL_t[3] == UPADTE_TEMP_OR_POWER)||(DATA_FROM_LOCAL_t[3] == OPEN_LOOP_START)||(DATA_FROM_LOCAL_t[3] == SLEEP)||(DATA_FROM_LOCAL_t[3]==SDA_CALLIBRATION)||(DATA_FROM_LOCAL_t[3]==OPEN_LOOP_START_PROFILE)||(DATA_FROM_LOCAL_t[3]==CLOSE_LOOP_START_PROFILE)) && (DATA_FROM_LOCAL_t[2]==MY_TCU_ID))
				{
				state = PROCESS_DATA;
				}
				else if(((DATA_FROM_LOCAL_t[3] == UPADTE_TEMP_OR_POWER)||(DATA_FROM_LOCAL_t[3]==OPEN_LOOP_START_PROFILE)||(DATA_FROM_LOCAL_t[3]==CLOSE_LOOP_START_PROFILE)) && ((DATA_FROM_LOCAL_t[2]==ID_S_PRESENT[0])||(DATA_FROM_LOCAL_t[2]==ID_S_PRESENT[1])||(DATA_FROM_LOCAL_t[2]==ID_S_PRESENT[2])))
				{
				state = SEND_DATA_TO_SLAVE;
				}
				switch(DATA_FROM_LOCAL_t[3])
				{
				case FETCH_BIST:
					state = FETCH_BIST;
					break;
				case CURRENT_LIMIT:
					SEND_ACK();
					state = CURRENT_LIMIT;
					break;
				case LOGGING_INTERVAL:
					SEND_ACK();
					state = LOGGING_INTERVAL;
					break;
				case LORA_PARAMETERS:
					SEND_ACK();
					state = LORA_PARAMETERS;
					break;
				case FETCH_DATA:
					state = FETCH_DATA;
					break;
				case CLEAR_FLASH:
					SEND_ACK();
					state = CLEAR_FLASH;
					break;
				case SDA_CALLIBRATION:
					SEND_ACK();
					state = SDA_CALLIBRATION;
					break;
				case KP_KI_KD_DATA:
					SEND_ACK();
					state = KP_KI_KD_DATA;
					break;
				case UPDATE_PROFILE:

					state = UPDATE_PROFILE;
					break;
				case MASTER_SLAVE_CONFIG:
					state = MASTER_SLAVE_CONFIG;
					break;
				case RESET_MAS_SLAVE:
					state = RESET_MAS_SLAVE;
					break;
				}
				switch(state)
				{
				case	SDA_CALLIBRATION:
					sda_msg = osPoolAlloc(sdasendpool);
					sda_msg->SDA_SEND_DATA[0] = SDA_CALLIBRATION_FACTOR;
					sda_msg->SDA_SEND_DATA[1] = DATA_FROM_LOCAL_t[4];
					sda_msg->SDA_SEND_DATA[2] = DATA_FROM_LOCAL_t[5];
					sda_msg->SDA_SEND_DATA[3] = DATA_FROM_LOCAL_t[6];
					sda_msg->SDA_SEND_DATA[4] = DATA_FROM_LOCAL_t[7];
					osMessagePut(SDA_SEND_qHandle, (uint32_t)sda_msg,100);
					flashmsg = osPoolAlloc(flashpool);
					flashmsg->FLASH_DATA_t[0] = SDA_CALLIBRATION_FACTOR;
					flashmsg->FLASH_DATA_t[1] = DATA_FROM_LOCAL_t[4];
					flashmsg->FLASH_DATA_t[2] = DATA_FROM_LOCAL_t[5];
					flashmsg->FLASH_DATA_t[3] = DATA_FROM_LOCAL_t[6];
					flashmsg->FLASH_DATA_t[4] = DATA_FROM_LOCAL_t[7];
					osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
					state = 0;
				break;

				case CURRENT_LIMIT:
					currlimitmsg = osPoolAlloc(currlimitpool);
					currlimitmsg->CURRENT_LIMIT_DATA = DATA_FROM_LOCAL_t[4];
					osMessagePut(CURRENT_LIMIT_Q_qHandle, (uint32_t)currlimitmsg, 10);
					flashmsg = osPoolAlloc(flashpool);
					flashmsg->FLASH_DATA_t[0] = CURRENT_LIMIT;
					flashmsg->FLASH_DATA_t[1] = DATA_FROM_LOCAL_t[4];
					osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
					state = 0;
				break;

				case FETCH_DATA:
					flashreadmsg = osPoolAlloc(flashreadpool);
					flashreadmsg->FLASH_READ_DATA[0] = DATA_FROM_LOCAL_t[4];
					osMessagePut(FLASH_READ_qHandle,(uint32_t) flashreadmsg,10);
					state = 0;
				break;

				case FETCH_BIST:
					flashreadmsg = osPoolAlloc(flashreadpool);
					flashreadmsg->FLASH_READ_DATA[0] = DATA_FROM_LOCAL_t[3];
					osMessagePut(FLASH_READ_qHandle,(uint32_t) flashreadmsg,10);
					state = 0;
				break;

				case CLEAR_FLASH:
					SLAVE_Erase_4K(clear_address);
					state = 0;
				break;

				case RESET_MAS_SLAVE:
					  memset(slavemsg->SLAVE_DATA,0,50);
					  slavemsg = osPoolAlloc(slavesendpool);
					  slavemsg->SLAVE_DATA[0] = MASTER_BIT;
					  slavemsg->SLAVE_DATA[1] = ID_S_PRESENT[0];
					  slavemsg->SLAVE_DATA[2] = RESET_MAS_SLAVE;
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  osDelay(100);
					  memset(slavemsg->SLAVE_DATA,0,50);
					  slavemsg = osPoolAlloc(slavesendpool);
					  slavemsg->SLAVE_DATA[0] = MASTER_BIT;
					  slavemsg->SLAVE_DATA[1] = ID_S_PRESENT[1];
					  slavemsg->SLAVE_DATA[2] = RESET_MAS_SLAVE;
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  osDelay(100);
					  memset(slavemsg->SLAVE_DATA,0,50);
					  slavemsg = osPoolAlloc(slavesendpool);
					  slavemsg->SLAVE_DATA[0] = MASTER_BIT;
					  slavemsg->SLAVE_DATA[1] = ID_S_PRESENT[2];
					  slavemsg->SLAVE_DATA[2] = RESET_MAS_SLAVE;
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  osDelay(100);
				break;

				case LOGGING_INTERVAL:
					log_interval = ((uint16_t)DATA_FROM_LOCAL_t[4]<<8)|(uint16_t)DATA_FROM_LOCAL_t[5];
					flashmsg = osPoolAlloc(flashpool);
					flashmsg->FLASH_DATA_t[0] = LOGGING_INTERVAL;
					flashmsg->FLASH_DATA_t[1] = DATA_FROM_LOCAL_t[4];
					flashmsg->FLASH_DATA_t[2] = DATA_FROM_LOCAL_t[5];
					osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,10);
					state = 0;
				break;

				case KP_KI_KD_DATA:
					 pos =0;
		 	  	  	 kpkikd_msg = osPoolAlloc(kpkikdpool);
		 	  	  	 for(int i =0;i<4;i++)
		 	  	  	 {
		 	  	  	 kpkikd_msg->KP_KI_KD_t[i] = DATA_FROM_LOCAL_t[i+4];
		 	  	  	 }
		 	  	  	 osMessagePut(KP_KI_KD_qHandle, (uint32_t)kpkikd_msg,100);
		 	  	  	 SLAVE_Read_Data(STARTING_ADD_OF_ENGG_MODE,DATA_FROM_FLASH,200);
		 	  	  	 for(int i =0;i<200;i++)
		 	  	  	 {
		 	  	  		 flashmsg->FLASH_DATA_t[i] = DATA_FROM_FLASH[i];
		 	  	  	 }
		 	  	  	 pos = (DATA_FROM_LOCAL_t[4])/10;
		 	  	  	 pos = (pos*4)+37;
			  	  	 	flashmsg->FLASH_DATA_t[0] = KP_KI_KD_DATA;
		 	  	  	 	flashmsg->FLASH_DATA_t[36] = KP_KI_KD_DATA;
						flashmsg->FLASH_DATA_t[pos] = DATA_FROM_LOCAL_t[4];
						flashmsg->FLASH_DATA_t[pos+1] = DATA_FROM_LOCAL_t[5];
						flashmsg->FLASH_DATA_t[pos+2] = DATA_FROM_LOCAL_t[6];
						flashmsg->FLASH_DATA_t[pos+3] = DATA_FROM_LOCAL_t[7];

		 	  	  	 osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
		 			state = 0;
				break;

				case LORA_PARAMETERS:
					frequency_t = (((uint32_t)DATA_FROM_LOCAL_t[4])<<24)|(((uint32_t)DATA_FROM_LOCAL_t[5])<<16)|(((uint32_t)DATA_FROM_LOCAL_t[6])<<8)|((uint32_t)DATA_FROM_LOCAL_t[7]);
					offset_t = (((uint32_t)DATA_FROM_LOCAL_t[8])<<24)|(((uint32_t)DATA_FROM_LOCAL_t[9])<<16)|(((uint32_t)DATA_FROM_LOCAL_t[10])<<8)|((uint32_t)DATA_FROM_LOCAL_t[11]);
					Sperading_Factor = DATA_FROM_LOCAL_t[12];
					Band_Width = (((uint16_t)DATA_FROM_LOCAL_t[13])<<8)|((uint16_t)DATA_FROM_LOCAL_t[14]);
					switch(Band_Width)
					{
					case 500:
						Band_Width = 6;
						break;
					case 250:
						Band_Width = 5;
						break;
					case 125:
						Band_Width = 4;
						break;
					case 62:
						Band_Width = 3;
						break;
					case 41:
						Band_Width = 10;
						break;
					case 31:
						Band_Width = 2;
						break;
					case 20:
						Band_Width = 9;
						break;
					case 15:
						Band_Width = 1;
						break;
					case 10:
						Band_Width = 8;
						break;
					case 7:
						Band_Width = 0;
						break;
					}
					CR = DATA_FROM_LOCAL_t[15];
					switch(CR)
					{
					case 45:
						CR =1 ;
						break;
					case 46:
						CR =2 ;
						break;
					case 47:
						CR =3 ;
						break;
					case 48:
						CR =4 ;
						break;
					}
					RADIO_RESET();
					RADIO_START(LORA_DEVICE);
					osDelay(1000);
					RADIO_SETUP_LORA(frequency_t, offset_t, Sperading_Factor, Band_Width, CR, LDRO_OFF);
					flashmsg = osPoolAlloc(flashpool);
					flashmsg->FLASH_DATA_t[0] = LORA_PARAMETERS;
					for(int i =1;i<13;i++)
					{
						flashmsg->FLASH_DATA_t[i] = DATA_FROM_LOCAL_t[i+3];
					}
					osMessagePut(FLASH_DATAHandle, (uint32_t)flashmsg,100);
					state = 0;
				break;

				case SEARCHING_DEVICES:
					memset(DATA_FROM_LOCAL_t,0xFF,BUFFER_SIZE_ALL);
					switch(Running_Status)
					{
					case FIRST_TIME:
						DATA_SENDING_TO_RCU_t[0]=LORA_BIT;
						DATA_SENDING_TO_RCU_t[1]=MODEM_REPLY_BIT;
						DATA_SENDING_TO_RCU_t[3]=SEARCH_IDs;
						DATA_SENDING_TO_RCU_t[4]=MY_TCU_ID;
						CRC_VALUE_OF_DATA_FOR_RCU_t  = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_SENDING_TO_RCU_t,(BUFFER_SIZE_ALL-1));
						DATA_SENDING_TO_RCU_t[(BUFFER_SIZE_ALL-1)]=CRC_VALUE_OF_DATA_FOR_RCU_t ;
						HAL_UART_Transmit(&huart6, DATA_SENDING_TO_RCU_t, 200,1000);

					break;

					case RUNNING:
						DATA_SENDING_TO_RCU_t[0]=LORA_BIT;
						DATA_SENDING_TO_RCU_t[1]=MODEM_REPLY_BIT;
						DATA_SENDING_TO_RCU_t[3]=RUNNING_SEARCH;
						DATA_SENDING_TO_RCU_t[4]=MY_TCU_ID;
						DATA_SENDING_TO_RCU_t[5] = ID_S_PRESENT[0];
						DATA_SENDING_TO_RCU_t[6] = ID_S_PRESENT[1];
						DATA_SENDING_TO_RCU_t[7] = ID_S_PRESENT[2];
						DATA_SENDING_TO_RCU_t[8] = START_STOP_STATE;
						DATA_SENDING_TO_RCU_t[9] = START_CONDITION;
						DATA_SENDING_TO_RCU_t[10] = MODE_CONDITION;
						CRC_VALUE_OF_DATA_FOR_RCU_t  = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_SENDING_TO_RCU_t,(BUFFER_SIZE_ALL-1));
						DATA_SENDING_TO_RCU_t[(BUFFER_SIZE_ALL-1)]=CRC_VALUE_OF_DATA_FOR_RCU_t ;
						HAL_UART_Transmit(&huart6, DATA_SENDING_TO_RCU_t, 200,1000);
						Running_Status = RUNNING;
					break;
					}
					state = 0;
				break;
				case MASTER_SLAVE_CONFIG:
				if((DATA_FROM_LOCAL_t[4]==MY_TCU_ID)||(DATA_FROM_LOCAL_t[6]==MY_TCU_ID)||(DATA_FROM_LOCAL_t[8]==MY_TCU_ID)||(DATA_FROM_LOCAL_t[10]==MY_TCU_ID))
				{
				if(DATA_FROM_LOCAL_t[4]==MY_TCU_ID)
				{
				if(DATA_FROM_LOCAL_t[5]==I_AM_MASTER)
				{
					I_AM = I_AM_MASTER;
					ID_S[0]=DATA_FROM_LOCAL_t[6];
					ID_S[1]=DATA_FROM_LOCAL_t[8];
					ID_S[2]=DATA_FROM_LOCAL_t[10];
					idmsg = osPoolAlloc(slaveidpool);
					for(int i =0;i<3;i++)
					{
					  idmsg->SLAVE_ID[i] = ID_S[i];
					}
					osMessagePut(SLAVE_ID_qHandle, (uint32_t)idmsg,100);
				}
				else if(DATA_FROM_LOCAL_t[5]==I_AM_SLAVE)
				{
					  I_AM = I_AM_SLAVE;
				}
				}
				if(DATA_FROM_LOCAL_t[8]==MY_TCU_ID)
				{
				if(DATA_FROM_LOCAL_t[9]==I_AM_MASTER)
				{
					  I_AM = I_AM_MASTER;
					  ID_S[0]=DATA_FROM_LOCAL_t[4];
					  ID_S[1]=DATA_FROM_LOCAL_t[6];
					  ID_S[2]=DATA_FROM_LOCAL_t[10];
					  idmsg = osPoolAlloc(slaveidpool);
					  for(int i =0;i<3;i++)
					  {
						  idmsg->SLAVE_ID[i] = ID_S[i];
					  }
					  osMessagePut(SLAVE_ID_qHandle, (uint32_t)idmsg,100);
				 }
				 else if(DATA_FROM_LOCAL_t[9]==I_AM_SLAVE)
				 {
					  I_AM = I_AM_SLAVE;
				 }
				}
				if(DATA_FROM_LOCAL_t[6]==MY_TCU_ID)
				{
				if(DATA_FROM_LOCAL_t[7]==I_AM_MASTER)
				{
					  I_AM = I_AM_MASTER;
					  ID_S[0]=DATA_FROM_LOCAL_t[4];
					  ID_S[1]=DATA_FROM_LOCAL_t[8];
					  ID_S[2]=DATA_FROM_LOCAL_t[10];
					  idmsg = osPoolAlloc(slaveidpool);
					  for(int i =0;i<3;i++)
					  {
						  idmsg->SLAVE_ID[i] = ID_S[i];
					  }
					  osMessagePut(SLAVE_ID_qHandle, (uint32_t)idmsg,100);
				 }
				 else if(DATA_FROM_LOCAL_t[7]==I_AM_SLAVE)
				 {
					  I_AM = I_AM_SLAVE;
				 }
				}
				if(DATA_FROM_LOCAL_t[10]==MY_TCU_ID)
				{
				if(DATA_FROM_LOCAL_t[11]==I_AM_MASTER)
				{
					  I_AM = I_AM_MASTER;
					  ID_S[0]=DATA_FROM_LOCAL_t[6];
					  ID_S[1]=DATA_FROM_LOCAL_t[8];
					  ID_S[2]=DATA_FROM_LOCAL_t[4];
					  idmsg = osPoolAlloc(slaveidpool);
					  for(int i =0;i<3;i++)
					  {
						  idmsg->SLAVE_ID[i] = ID_S[i];
					  }
					  osMessagePut(SLAVE_ID_qHandle, (uint32_t)idmsg,100);
				 }
				 else if(DATA_FROM_LOCAL_t[11]==I_AM_SLAVE)
				 {
					  I_AM = I_AM_SLAVE;
				 }
				}
				if(I_AM == I_AM_MASTER)
				{
				  WHOM_TO_GIVE_DATA = LOCAL_SYSTEM;
				}
				if(I_AM == I_AM_SLAVE)
				{
				  WHOM_TO_GIVE_DATA = MAS_SLAVE_SYSTEM;
				}
				}
				state = 0;
				break;

				case PROCESS_DATA:
				for(int i =0;i<49;i++)
				{
				DATA_FOR_PROCESSING[i] = DATA_FROM_LOCAL_t[i+1];
				}
				DATA_FOR_PROCESSING[0] = MASTER_BIT;
				DATA_FOR_PROCESSING[49] = 0x00;
				syspromsg = osPoolAlloc(syspropool);
				for(int i =0;i<50;i++)
				{
				syspromsg->DATA_TO_PROCESS[i] = DATA_FOR_PROCESSING[i];
				}
				osMessagePut(SYSTEM_PROCESS_qHandle, (uint32_t)syspromsg,100);
				if((DATA_FROM_LOCAL_t[3] == CLOSE_LOOP_START)||(DATA_FROM_LOCAL_t[3] == OPEN_LOOP_START)||(DATA_FROM_LOCAL_t[3] == SLEEP))
				{
				  for(int i =0;i<49;i++)
				  {
					  DATA_FOR_PROCESSING[i] = DATA_FROM_LOCAL_t[i+1];
				  }
				  DATA_FOR_PROCESSING[0] = MASTER_BIT;
				  DATA_FOR_PROCESSING[49] = 0x00;
				  slavemsg = osPoolAlloc(slavesendpool);
				  for(int i =0;i<50;i++)
				  {
					  slavemsg->SLAVE_DATA[i] = DATA_FOR_PROCESSING[i];
				  }
				  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
				}
				state = 0;
				break;

				case UPDATE_PROFILE:
				for(int i =0;i<49;i++)
				{
				DATA_OF_PROFILE[i] = DATA_FROM_LOCAL_t[i+1];
				}

				if((I_AM==I_AM_MASTER)&&(DATA_FROM_LOCAL_t[2]==MY_TCU_ID))
				{
					for(int i=0;i<50;i++)
					{
						PROFILES[z][i] = DATA_OF_PROFILE[i];
					}
					PROFILE_COUNT=z;
					z = z+1;
					if(z==50)
					{
						z=0;
					}
				}
				if((I_AM==I_AM_MASTER)&&(DATA_FROM_LOCAL_t[2]!=MY_TCU_ID))
				{
					DATA_OF_PROFILE[0] = MASTER_BIT;
					  slavemsg = osPoolAlloc(slavesendpool);
					  for(int i =0;i<50;i++)
					  {
						  slavemsg->SLAVE_DATA[i] = DATA_OF_PROFILE[i];
					  }
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
				}
				state = 0;
				SEND_ACK();
				break;


				case SEND_DATA_TO_SLAVE:
				for(int i =0;i<49;i++)
				{
				DATA_FOR_PROCESSING[i] = DATA_FROM_LOCAL_t[i+1];
				}
				DATA_FOR_PROCESSING[0] = MASTER_BIT;
				DATA_FOR_PROCESSING[49] = 0x00;
				slavemsg = osPoolAlloc(slavesendpool);
				for(int i =0;i<50;i++)
				{
				slavemsg->SLAVE_DATA[i] = DATA_FOR_PROCESSING[i];
				}
				osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
				if((DATA_FROM_LOCAL_t[3] == CLOSE_LOOP_START)||(DATA_FROM_LOCAL_t[3] == OPEN_LOOP_START)||(DATA_FROM_LOCAL_t[3] == SLEEP))
				{
				  for(int i =0;i<49;i++)
				  {
					  DATA_FOR_PROCESSING[i] = DATA_FROM_LOCAL_t[i+1];
				  }
				  DATA_FOR_PROCESSING[0] = MASTER_BIT;
				  DATA_FOR_PROCESSING[49] = 0x00;
				  syspromsg = osPoolAlloc(syspropool);
				  for(int i =0;i<50;i++)
				  {
					  syspromsg->DATA_TO_PROCESS[i] = DATA_FOR_PROCESSING[i];
				  }
				  osMessagePut(SYSTEM_PROCESS_qHandle, (uint32_t)syspromsg,100);
				}
				state = 0;
				break;
				}


				if(DATA_FROM_LOCAL_t[40]==ACK_NACK_DATA)
				{
					  if(DATA_FROM_LOCAL_t[41]==ACK)
					  {
					  xTaskNotifyGive(DATA_LOGGINGHandle);
					  }
				}

      }

	  }
	  HAL_UART_Receive_DMA(&huart6, DATA_FROM_LOCAL, 50);
	  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx,DMA_IT_HT);
	  }
    osDelay(1);
  }
  /* USER CODE END Start_RCU_RECEIVE */
}

/* USER CODE BEGIN Header_Start_WIFI_RECEIVE */
/**
* @brief Function implementing the WIFI_RECEIVE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_WIFI_RECEIVE */
void Start_WIFI_RECEIVE(void const * argument)
{
  /* USER CODE BEGIN Start_WIFI_RECEIVE */
	SYSTEM_PROCESS	*syspromsg;

	uint8_t DATA_FROM_WIFI_t[50];
	uint8_t DATA_FOR_PROCESSING[50];

	static int notification =0;
  /* Infinite loop */
  for(;;)
  {
	  notification = ulTaskNotifyTake(0,portMAX_DELAY);
	  if( (notification>=0))
	  {
	  notification =0;
	  memcpy(DATA_FROM_WIFI_t,DATA_FROM_WIFI,50);
	  if(DATA_FROM_WIFI_t[1] == MODEM_BIT)
	  {
			  if(((DATA_FROM_WIFI_t[3] == CLOSE_LOOP_START)||(DATA_FROM_WIFI_t[3] == UPADTE_TEMP_OR_POWER)||(DATA_FROM_WIFI_t[3] == OPEN_LOOP_START)||(DATA_FROM_WIFI_t[3] == SLEEP)||(DATA_FROM_WIFI_t[3] == KP_KI_KD_DATA)) )
			  {
					osThreadResume(DATA_LOGGINGHandle);
					for(int i =0;i<49;i++)
					{
					DATA_FOR_PROCESSING[i] = DATA_FROM_WIFI_t[i+1];
					}
					DATA_FOR_PROCESSING[0] = MASTER_BIT;
					DATA_FOR_PROCESSING[49] = 0x00;
					syspromsg = osPoolAlloc(syspropool);
					for(int i =0;i<50;i++)
					{
					syspromsg->DATA_TO_PROCESS[i] = DATA_FOR_PROCESSING[i];
					}
					osMessagePut(SYSTEM_PROCESS_qHandle, (uint32_t)syspromsg,100);
			  }
	  }
	  HAL_UART_Receive_DMA(&huart4, DATA_FROM_WIFI, 50);
	  __HAL_DMA_DISABLE_IT(&hdma_uart4_rx,DMA_IT_HT);
	  }
    osDelay(1);
  }
  /* USER CODE END Start_WIFI_RECEIVE */
}

/* USER CODE BEGIN Header_Start_CURRENT_TASK */
/**
* @brief Function implementing the CURRENT_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CURRENT_TASK */
void Start_CURRENT_TASK(void const * argument)
{
  /* USER CODE BEGIN Start_CURRENT_TASK */
	CURRENT_P *currmsgp;

	uint8_t ADC_Receive[4];
	uint8_t ADC_MODE_CTRL[2];
	uint8_t b[40];
	uint8_t ADC_CONFIG[2] = {0x82,0x04};
	uint8_t ADC_RST[] = {0b00001000,0b11000000};
	uint8_t ADC_RSTBUFF[] = {0b00001000,0b10100000};
	uint8_t ADC_CNFG[] = {0b10000010,0b00000100};
	uint8_t ADC_MODE[] = {0b00011111,0b10000110};
	uint8_t ADC_getdata[] = {0b00011111,0b10000100};
	uint16_t ADC_MODE_t;

	HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
	HAL_SPI_TransmitReceive(&hspi4, ADC_RST, b, 2, 100);
	HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
	HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
	HAL_SPI_TransmitReceive(&hspi4, ADC_CNFG, b, 2, 100);
	HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
  /* Infinite loop */
  for(;;)
  {
	  HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
	  HAL_SPI_TransmitReceive(&hspi4, ADC_CNFG, b, 2, 100);
      HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
	  HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
	  HAL_SPI_TransmitReceive(&hspi4, ADC_RSTBUFF, b, 2, 100);
      HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
	  HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
	  HAL_SPI_TransmitReceive(&hspi4, ADC_MODE, b, 3, 100);
      HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
	  osDelay(1);

	  for (uint8_t i=0; i<15;)
	  {
		    HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
			HAL_StatusTypeDef spists =   HAL_SPI_TransmitReceive(&hspi4, ADC_getdata, &b[2*i], 2, 100);
			HAL_GPIO_TogglePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin);
			if(spists == HAL_OK)
			{
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			i++;
			b[0] = ADC_12_CH_DATA;
			}
			if(spists != HAL_OK)
	        {
			HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
			b[0] = 0;
	        }
      }

		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, RESET);
		HAL_SPI_Transmit(&hspi4, ADC_CONFIG, sizeof(ADC_CONFIG), 500);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, SET);

		ADC_MODE_t = 0x16A2;
		ADC_MODE_CTRL[0] = (ADC_MODE_t&(0XFF00))>>8;
		ADC_MODE_CTRL[1] = (ADC_MODE_t&(0X00FF));
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, RESET);
		HAL_SPI_Transmit(&hspi4, ADC_MODE_CTRL, sizeof(ADC_MODE_CTRL), 500);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, SET);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, RESET);
		HAL_StatusTypeDef spists13 =HAL_SPI_Receive(&hspi4, &b[30], 2,0xFFFF);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, SET);
		if(spists13!=HAL_OK)
		{
		b[36] = 0;
		}
		if(spists13 == HAL_OK)
		{
		b[36] = ADC_13_CH_DATA;
		}
		ADC_MODE_t =0;
		memset(ADC_MODE_CTRL,0,sizeof(ADC_MODE_CTRL));
		memset(ADC_Receive,0,sizeof(ADC_Receive));

		ADC_MODE_t = 0x1722;
		ADC_MODE_CTRL[0] = (ADC_MODE_t&(0XFF00))>>8;
		ADC_MODE_CTRL[1] = (ADC_MODE_t&(0X00FF));
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, RESET);
		HAL_SPI_Transmit(&hspi4, ADC_MODE_CTRL, sizeof(ADC_MODE_CTRL), 500);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, SET);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, RESET);
		HAL_StatusTypeDef spists14 = HAL_SPI_Receive(&hspi4, &b[32], 2,0xFFFF);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, SET);
		if(spists14!=HAL_OK)
		{
		b[37] = 0;
		}
		if(spists14 == HAL_OK)
		{
		b[37] = ADC_14_CH_DATA;
		}
		ADC_MODE_t =0;
		memset(ADC_MODE_CTRL,0,sizeof(ADC_MODE_CTRL));
		memset(ADC_Receive,0,sizeof(ADC_Receive));


		ADC_MODE_t = 0x17A2;
		ADC_MODE_CTRL[0] = (ADC_MODE_t&(0XFF00))>>8;
		ADC_MODE_CTRL[1] = (ADC_MODE_t&(0X00FF));
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, RESET);
		HAL_SPI_Transmit(&hspi4, ADC_MODE_CTRL, sizeof(ADC_MODE_CTRL), 500);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, SET);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, RESET);
		HAL_StatusTypeDef spists15 = HAL_SPI_Receive(&hspi4, &b[34], 2,0xFFFF);
		HAL_GPIO_WritePin(CURRENT_CS_GPIO_Port, CURRENT_CS_Pin, SET);
		if(spists15!=HAL_OK)
		{
		b[38] = 0;
		}
		if(spists15 == HAL_OK)
		{
		b[38] = ADC_15_CH_DATA;
		}
		ADC_MODE_t =0;
		memset(ADC_MODE_CTRL,0,sizeof(ADC_MODE_CTRL));
		memset(ADC_Receive,0,sizeof(ADC_Receive));

		currmsgp = osPoolAlloc(currppool);
		for(int i =0;i<40;i++)
		{
		currmsgp->CURRENT_P_DATA[i] = b[i];
		}
		osMessagePut(CURRENT_P_qHandle, (uint32_t)currmsgp,100);
		memset(b,0,sizeof(b));
  }
  /* USER CODE END Start_CURRENT_TASK */
}

/* USER CODE BEGIN Header_Start_PID_TASK */
/**
* @brief Function implementing the PID_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_PID_TASK */
void Start_PID_TASK(void const * argument)
{
  /* USER CODE BEGIN Start_PID_TASK */
	GIVE_ME	*givememsg;
	FEEDBACK	*feedbackmsg;
	SETPOINT	*setpointmsg;
	DUTY_Q *dutymsg;
	KP_KI_KD *kpkikdmsg;
	PID_DATA	*pidmsg;

	osEvent	spevt,fbevt,kpkikdevt;

	uint16_t FEEDBACK[16];

	static float PID_OUTPUT[16] ={0};
	float CURRENT_COUNT,COUNT_DIFFERENCE,PREVIOUS_COUNT;
	float PROPORTIONAL[16],CURRENT_ERROR[16],INTEGRAL[16],PREVIOUS_ERROR[16],DERIVATIVE[16] ={0};
	static int fb =0;
	static int sp =0;
	static uint16_t SETPOINT[16]={0};
	float setpoint,feedback;
	float KPKIKD_DATA_t[4];
	int pos;


  /* Infinite loop */
  for(;;)
  {
	  givememsg = osPoolAlloc(givepool);
	  givememsg->GIVE_ME_t = I_AM_PID;
	  osMessagePut(GIVE_ME_qHandle, (uint32_t)givememsg, 100);

	  kpkikdevt = osMessageGet(KP_KI_KD_qHandle, 0);
	  if(kpkikdevt.status == osEventMessage)
	  {
		  kpkikdmsg = kpkikdevt.value.p;
		  for(int i =0;i<4;i++)
		  {
			  KPKIKD_DATA_t[i]=kpkikdmsg->KP_KI_KD_t[i];
			  KPKIKD_DATA_t[i] = KPKIKD_DATA_t[i]/10;
		  }
		  osPoolFree(kpkikdpool, kpkikdmsg);
		  pos = (int)KPKIKD_DATA_t[0];
		  KP[pos] = KPKIKD_DATA_t[1];
		  KI[pos] = KPKIKD_DATA_t[2];
		  KD[pos] = KPKIKD_DATA_t[3];
	  }

	  spevt = osMessageGet(SETPOINT_qHandle, 0);
	  if(spevt.status == osEventMessage)
	  {
		  setpointmsg = spevt.value.p;
		  for(int i =0;i<16;i++)
		  {
			  SETPOINT[i] = setpointmsg->SETPOINT_t[i];
		  }
		  osPoolFree(sppool, setpointmsg);
		  sp=1;
	  }

	  fbevt = osMessageGet(FEEDBACK_qHandle, 0);
	  if(fbevt.status == osEventMessage)
	  {
		  feedbackmsg = fbevt.value.p;
		  for(int i =0;i<16;i++)
		  {
			  FEEDBACK[i] = feedbackmsg->FEEDBACK_t[i];
			  FEEDBACK[i] = (FEEDBACK[i]&0x0FFF);
		  }
		  osPoolFree(fbpool, feedbackmsg);
		  fb=1;
	  }

	  if(LOOP_RUNNING == OPEN_LOOP)
	  {
		  pidmsg = osPoolAlloc(pidpool);
		  dutymsg = osPoolAlloc(dutypool);
		  for(int i =0;i<16;i++)
		  {
	  			setpoint = (((float)SETPOINT[i])/10000);
	  			DUTY[i]=setpoint;
         		pidmsg->PID_DATA_t[i] = (int)(DUTY[i]*1000);
	  		 	dutymsg->DUTY_t[i] = SETPOINT[i];
		  }
		  osMessagePut(PID_DATA_qHandle, (uint32_t)pidmsg,100);
		  osMessagePut(DUTY_Q_qHandle, (uint32_t)dutymsg,100);
	  }
	  if(LOOP_RUNNING == CLOSE_LOOP)
	  {
	  if(NO_DUTY == 1)
	  {
		  for(int i =0;i<16;i++)
		  {
			  DUTY[i]=0;
			  PCA9685_SetDuty(i, DUTY[i]);
		  }

	  }
	  if(sp==1 && fb ==1 )
	  {
		  fb=0;
  		  HAL_TIM_Base_Stop(&htim4);
		  CURRENT_COUNT =(__HAL_TIM_GET_COUNTER(&htim4))/(float)30000;
		  COUNT_DIFFERENCE = (CURRENT_COUNT-PREVIOUS_COUNT);
		  if(COUNT_DIFFERENCE == 0)
		  {
			  COUNT_DIFFERENCE =1;
		  }
			for(int i =0;i<16;i++)
			{
			setpoint =0;
		    feedback=0;
			setpoint = (((float)SETPOINT[i])/100);
			feedback = (((float)FEEDBACK[i])/(float)10);
			CURRENT_ERROR[i] = setpoint - (feedback);
			PROPORTIONAL[i] = CURRENT_ERROR[i];
			INTEGRAL[i] =  (KI[i]/KP[i])*((CURRENT_ERROR[i])-(PREVIOUS_ERROR[i])/2)*COUNT_DIFFERENCE;
			if(INTEGRAL[i]<0)
			{
			INTEGRAL[i] =0;
			}
			DERIVATIVE[i] = (KD[i]/KP[i])*((CURRENT_ERROR[i])-(PREVIOUS_ERROR[i]));
			PID_OUTPUT[i] = KP[i]*(PROPORTIONAL[i]+INTEGRAL[i]+DERIVATIVE[i]);
			if( PID_OUTPUT[i]>0)
			{
			DUTY[i] = map(PID_OUTPUT[i],0,setpoint,0,1);
			PREVIOUS_ERROR[i] = CURRENT_ERROR[i];
			}
			if(PID_OUTPUT[i]<0)
			{
			DUTY[i] =0;
			}
			}

			pidmsg = osPoolAlloc(pidpool);
			for(int i =0;i<16;i++)
			{
			pidmsg->PID_DATA_t[i] = (int)(DUTY[i]*1000);
			}
			osMessagePut(PID_DATA_qHandle, (uint32_t)pidmsg,100);

			PREVIOUS_COUNT = CURRENT_COUNT;
			CURRENT_COUNT =0;
			__HAL_TIM_SET_COUNTER(&htim4,0);
			HAL_TIM_Base_Start(&htim4);
	  }
	  }
    osDelay(50);
  }
  /* USER CODE END Start_PID_TASK */
}

/* USER CODE BEGIN Header_Start_VOLTAGE_TASK */
/**
* @brief Function implementing the VOLTAGE_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_VOLTAGE_TASK */
void Start_VOLTAGE_TASK(void const * argument)
{
  /* USER CODE BEGIN Start_VOLTAGE_TASK */
	LCD_QUEUE *lcdmsg;
	DATA_ACCU	*dataaccumsg;

	uint16_t ADC_VALUE_OF_VOLTAGE;
	float REAL_VOLTAGE_ADC_RANGE;
	int VOLTAGE;
	float buffer[10];
	float MAVG;
	static int k =0;
  /* Infinite loop */
  for(;;)
  {
		HAL_ADC_Start(&hadc2);
		HAL_ADC_PollForConversion(&hadc2, 1);
		ADC_VALUE_OF_VOLTAGE = HAL_ADC_GetValue(&hadc2);
		REAL_VOLTAGE_ADC_RANGE = (ADC_VALUE_OF_VOLTAGE*(3.3/4096))*1000;
		VOLTAGE = map(REAL_VOLTAGE_ADC_RANGE, 1755,193,60, 80);
		HAL_ADC_Stop(&hadc2);
		buffer[k] = VOLTAGE;
		k++;
		if(k>=10)
		{
		k=0;
		}
		for(int j =0;j<10;j++)
		{
		MAVG = MAVG+buffer[j];
		}
		MAVG = MAVG/10;
		lcdmsg = osPoolAlloc(lcdpool);
		dataaccumsg = osPoolAlloc(dataaccupool);
		lcdmsg->LCD_DATA[0]=SYS_VOLTAGE;
		lcdmsg->LCD_DATA[1] = (int)MAVG;
		dataaccumsg->SYSTEM_VOLTAGE_t = (int)MAVG;
		osMessagePut(DATA_ACCU_qHandle, (uint32_t)dataaccumsg,100);
		osMessagePut(LCD_QUEUE_qHandle, (uint32_t)lcdmsg,100);
		MAVG = 0;
    osDelay(1);
  }
  /* USER CODE END Start_VOLTAGE_TASK */
}

/* USER CODE BEGIN Header_Start_WHOLE_CURRENT */
/**
* @brief Function implementing the WHOLE_CURRENT thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_WHOLE_CURRENT */
void Start_WHOLE_CURRENT(void const * argument)
{
  /* USER CODE BEGIN Start_WHOLE_CURRENT */
	DATA_ACCU *dataaccumsg;
	LCD_QUEUE *lcdmsg;
	CURRENT_LIMIT_Q *currlmsg;

	uint16_t ADC_READINg_CURR_SENSOR;
	float VOLT_READING_CURR_SENSOR;
	float CURR_READING;
	float buffer[200];
	float MAVG;
	static int k =0;
  /* Infinite loop */
  for(;;)
  {
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, 10);
		ADC_READINg_CURR_SENSOR = HAL_ADC_GetValue(&hadc1);
		VOLT_READING_CURR_SENSOR = (float)ADC_READINg_CURR_SENSOR*(3.3/4096);
		CURR_READING = (float)(2.495-VOLT_READING_CURR_SENSOR)/(float)(0.003125);
		buffer[k] = CURR_READING;
		k++;
		if(k>=200)
		{
		k=0;
		}
		for(int j =0;j<200;j++)
		{
		MAVG = MAVG+buffer[j];
		}
		MAVG = MAVG/200;
		lcdmsg = osPoolAlloc(lcdpool);
		dataaccumsg = osPoolAlloc(dataaccupool);
		lcdmsg->LCD_DATA[0]=WHOLE_CURRENT;
		lcdmsg->LCD_DATA[1] = MAVG;
		dataaccumsg->SYSTEM_CURRENT_t = MAVG;
		osMessagePut(DATA_ACCU_qHandle,(uint32_t)dataaccumsg,100);
		osMessagePut(LCD_QUEUE_qHandle, (uint32_t)lcdmsg,100);
		if(MAVG>=1)
		{
		currlmsg = osPoolAlloc(currlimitpool);
		currlmsg->WHOLE_CURRENT_DATA = (int)MAVG;
		osMessagePut(CURRENT_LIMIT_Q_qHandle, (uint32_t)currlmsg,10);
		}
		MAVG =0;
    osDelay(1);
  }
  /* USER CODE END Start_WHOLE_CURRENT */
}

/* USER CODE BEGIN Header_Start_ADC_TASK */
/**
* @brief Function implementing the ADC_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_ADC_TASK */
void Start_ADC_TASK(void const * argument)
{
  /* USER CODE BEGIN Start_ADC_TASK */
	LCD_QUEUE	*lcdmsg;
	DATA_ACCU	*dataaccumsg;

	uint16_t IR_TEMPERATURE_t;
	uint16_t AMB_TEMPERATURE_t;
	uint16_t LHS_TEMPERATURE_t;
	uint16_t RHS_TEMPERATURE_t;
	float VOLTS_OF_IR_READING;
	float FINAL_IR;
	float VOLTS_OF_AMB_READING;
	float VOLTS_OF_LHS_READING;
	float VOLTS_OF_RHS_READING;
	static int pos =0;
	int AMB_RES,HSL_RES,HSR_RES;
	static float IR_A = 0.0007408923526;
	static float IR_B = 0.0002122157150;
	static float IR_C = 0.0000001113364198;
	static float HS_A =0.001103301412;
	static float HS_B=0.0002371732258;
	static float HS_C=0.00000008481694645;
	float IR_Inv_T,HSL_Inv_T,HSR_Inv_T;
	float IR_AMB_T,HSL_T,HSR_T;
	float IR_logr,HSL_logr,HSR_logr;
	float irbuffer[10];
	float irMAVG;
	static int irk =0;
	float ambbuffer[10];
	float ambMAVG;
	static int ambk =0;
	float lsbuffer[10];
	float lsMAVG;
	static int lsk =0;
	float rsbuffer[10];
	float rsMAVG;
	static int rsk =0;
	static int connected =0;
  /* Infinite loop */
  for(;;)
  {
	  switch(pos)
	  {
	  case 0:
		  IR_TEMPERATURE();
		  HAL_ADC_Start(&hadc3);
		  HAL_ADC_PollForConversion(&hadc3, 1000);
		  IR_TEMPERATURE_t = HAL_ADC_GetValue(&hadc3);
		  HAL_ADC_Stop(&hadc3);
		  VOLTS_OF_IR_READING = (((float)IR_TEMPERATURE_t*3.3)/4096);
		  FINAL_IR = map(VOLTS_OF_IR_READING,2.9,0.2,0,100);
		  if((VOLTS_OF_IR_READING<0.2)||(VOLTS_OF_IR_READING>2.9))
		  {
			  FINAL_IR =0;
			  connected =0;
		  }
		  else
		  {
			  connected =1;
		  }
		  irbuffer[irk] = FINAL_IR;
		  irk++;
		  if(irk>=10)
		  {
			  irk = 0;
		  }
		  for(int j =0;j<10;j++)
		  {
			  irMAVG = irMAVG+irbuffer[j];
		  }
		  irMAVG = irMAVG/10;
		  lcdmsg=osPoolAlloc(lcdpool);
		  dataaccumsg = osPoolAlloc(dataaccupool);
		  lcdmsg->LCD_DATA[0] = IR_TEMP_t;
		  lcdmsg->LCD_DATA[1] = (int)irMAVG;
		  dataaccumsg->IR_TEMPERATURE_t = (int)irMAVG;
		  osMessagePut(DATA_ACCU_qHandle, (uint32_t)dataaccumsg,100);
		  osMessagePut(LCD_QUEUE_qHandle, (uint32_t)lcdmsg, 100);
		  pos =1;
		  break;

	  case 1:
		  AMB_TEMPERATURE();
		  HAL_ADC_Start(&hadc3);
		  HAL_ADC_PollForConversion(&hadc3, 1000);
		  AMB_TEMPERATURE_t = HAL_ADC_GetValue(&hadc3);
		  HAL_ADC_Stop(&hadc3);
		  VOLTS_OF_AMB_READING = ((AMB_TEMPERATURE_t*3.3)/4096);
		  AMB_RES = (VOLTS_OF_AMB_READING/(9-VOLTS_OF_AMB_READING))*1000000;
		  IR_logr = logf(AMB_RES);
		  IR_Inv_T = IR_A+(IR_B*IR_logr)+(IR_C*(IR_logr*IR_logr*IR_logr));
		  IR_AMB_T = (1/IR_Inv_T)-273.15;
		  if(connected ==0)
		  {
			  IR_AMB_T=0;
		  }
		  ambbuffer[ambk] = IR_AMB_T;
		  ambk++;
		  if(ambk>=10)
		  {
			  ambk=0;
		  }
		  for(int j =0;j<10;j++)
		  {
			  ambMAVG = ambMAVG+ambbuffer[j];
		  }
		  ambMAVG = ambMAVG/10;
		  lcdmsg=osPoolAlloc(lcdpool);
		  lcdmsg->LCD_DATA[0] = AMB_TEMP_t;
		  lcdmsg->LCD_DATA[1] = (int)ambMAVG;
		  dataaccumsg = osPoolAlloc(dataaccupool);
		  dataaccumsg->AMBIENT_TEMPERATURE_t = (int)ambMAVG;
		  osMessagePut(DATA_ACCU_qHandle, (uint32_t)dataaccumsg,100);
		  osMessagePut(LCD_QUEUE_qHandle, (uint32_t)lcdmsg, 100);
		  pos =2;
		  break;

	  case 2:
		  LHS_TEMPERATURE();
		  HAL_ADC_Start(&hadc3);
		  HAL_ADC_PollForConversion(&hadc3, 1000);
		  LHS_TEMPERATURE_t = HAL_ADC_GetValue(&hadc3);
		  HAL_ADC_Stop(&hadc3);
		  VOLTS_OF_LHS_READING = ((LHS_TEMPERATURE_t*3.3)/4096);
		  HSL_RES = (VOLTS_OF_LHS_READING/(5-VOLTS_OF_LHS_READING))*27000;
		  HSL_logr = logf(HSL_RES);
		  HSL_Inv_T = HS_A+(HS_B*HSL_logr)+(HS_C*(HSL_logr*HSL_logr*HSL_logr));
		  HSL_T = (1/HSL_Inv_T)-273.15;
		  lsbuffer[lsk] = HSL_T;
		  lsk++;
		  if(lsk>=10)
		  {
			  lsk=0;
		  }
		  for(int j =0;j<10;j++)
		  {
			  lsMAVG = lsMAVG+lsbuffer[j];
		  }
		  lsMAVG = lsMAVG/10;
		  lcdmsg=osPoolAlloc(lcdpool);
		  lcdmsg->LCD_DATA[0] = HS_LEFT_t;
		  lcdmsg->LCD_DATA[1] = (int)lsMAVG;
		  dataaccumsg = osPoolAlloc(dataaccupool);
		  dataaccumsg->LEFT_HS_TEMPERATURE_t = (int)lsMAVG;
		  osMessagePut(DATA_ACCU_qHandle, (uint32_t)dataaccumsg,100);
		  osMessagePut(LCD_QUEUE_qHandle, (uint32_t)lcdmsg, 100);
		  pos=3;
		  break;

	  case 3:
		  RHS_TEMPERATURE();
		  HAL_ADC_Start(&hadc3);
		  HAL_ADC_PollForConversion(&hadc3, 1000);
		  RHS_TEMPERATURE_t = HAL_ADC_GetValue(&hadc3);
		  HAL_ADC_Stop(&hadc3);
		  VOLTS_OF_RHS_READING = ((RHS_TEMPERATURE_t*3.3)/4096);
		  HSR_RES = (VOLTS_OF_RHS_READING/(5-VOLTS_OF_RHS_READING))*27000;
		  HSR_logr = logf(HSR_RES);
		  HSR_Inv_T = HS_A+(HS_B*HSR_logr)+(HS_C*(HSR_logr*HSR_logr*HSR_logr));
		  HSR_T = (1/HSR_Inv_T)-273.15;
		  rsbuffer[rsk] = HSR_T;
		  rsk++;
		  if(rsk>=10)
		  {
			  rsk =0;
		  }
		  for(int j =0;j<10;j++)
		  {
			  rsMAVG = rsMAVG+rsbuffer[j];
		  }
		  rsMAVG = rsMAVG/10;
		  lcdmsg=osPoolAlloc(lcdpool);
		  lcdmsg->LCD_DATA[0] = HS_RIGHT_t;
		  lcdmsg->LCD_DATA[1] = (int)rsMAVG;
		  dataaccumsg = osPoolAlloc(dataaccupool);
		  dataaccumsg->RIGHT_HS_TEMPERATURE_t = (int)rsMAVG;
		  osMessagePut(DATA_ACCU_qHandle, (uint32_t)dataaccumsg,100);
		  osMessagePut(LCD_QUEUE_qHandle, (uint32_t)lcdmsg, 100);
		  pos=0;
		  break;
	  }
	osDelay(1);
  }
  /* USER CODE END Start_ADC_TASK */
}

/* USER CODE BEGIN Header_Start_FLASH_WRITE */
/**
* @brief Function implementing the FLASH_WRITE thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_FLASH_WRITE */
void Start_FLASH_WRITE(void const * argument)
{
  /* USER CODE BEGIN Start_FLASH_WRITE */
	FLASH_DATA *flashmsg;
	osEvent flashevt;
	uint8_t DATA_FOR_FLASH[200];
	uint32_t address_to_write;
	uint32_t address_to_read;
	uint8_t DATA_FROM_FLASH[200];
	static int increment = 0x32;
  /* Infinite loop */
  for(;;)
  {
	  flashevt = osMessageGet(FLASH_DATAHandle, 0);
	  if(flashevt.status == osEventMessage)
	  {
		  flashmsg = flashevt.value.p;
		  memcpy(DATA_FOR_FLASH,flashmsg->FLASH_DATA_t,200);
		  osPoolFree(flashpool, flashmsg);
		  switch(DATA_FOR_FLASH[0])
		  {
		  case CURRENT_LIMIT:
			  address_to_write = STARTING_ADD_OF_ENGG_MODE;
			  address_to_read = STARTING_ADD_OF_ENGG_MODE;
			  SLAVE_Read_Data(address_to_read, (char*)DATA_FROM_FLASH,200);
			  SLAVE_Erase_4K(address_to_write);
			  DATA_FROM_FLASH[0] = 0;
			  DATA_FROM_FLASH[1] = 0;
			  DATA_FROM_FLASH[2] = 0;
			  DATA_FROM_FLASH[3] = ENGG_MODE_DATA;
			  DATA_FROM_FLASH[4] = SIZE_UPDATE;
			  DATA_FROM_FLASH[5] = TEMP_SIZE_UPDATE;
			  DATA_FROM_FLASH[6] = CURRENT_LIMIT;
			  DATA_FROM_FLASH[7] = DATA_FOR_FLASH[1];
			  SLAVE_Write_Data(address_to_write, (char*)DATA_FROM_FLASH,200);
			  break;
		  case LOGGING_INTERVAL:
			  address_to_write = STARTING_ADD_OF_ENGG_MODE;
			  address_to_read = STARTING_ADD_OF_ENGG_MODE;
			  SLAVE_Read_Data(address_to_read, (char*)DATA_FROM_FLASH,200);
			  SLAVE_Erase_4K(address_to_write);
			  DATA_FROM_FLASH[0] = 0;
			  DATA_FROM_FLASH[1] = 0;
			  DATA_FROM_FLASH[2] = 0;
			  DATA_FROM_FLASH[3] = ENGG_MODE_DATA;
			  DATA_FROM_FLASH[4] = SIZE_UPDATE;
			  DATA_FROM_FLASH[5] = TEMP_SIZE_UPDATE;
			  DATA_FROM_FLASH[8] = LOGGING_INTERVAL;
			  DATA_FROM_FLASH[9] = DATA_FOR_FLASH[1];
			  DATA_FROM_FLASH[10] = DATA_FOR_FLASH[2];
			  SLAVE_Write_Data(address_to_write, (char*)DATA_FROM_FLASH,200);
			  break;
		  case SDA_CALLIBRATION_FACTOR:
			  address_to_write = STARTING_ADD_OF_ENGG_MODE;
			  address_to_read = STARTING_ADD_OF_ENGG_MODE;
			  SLAVE_Read_Data(address_to_read, (char*)DATA_FROM_FLASH,200);
			  SLAVE_Erase_4K(address_to_write);
			  DATA_FROM_FLASH[0] = 0;
			  DATA_FROM_FLASH[1] = 0;
			  DATA_FROM_FLASH[2] = 0;
			  DATA_FROM_FLASH[3] = ENGG_MODE_DATA;
			  DATA_FROM_FLASH[4] = SIZE_UPDATE;
			  DATA_FROM_FLASH[5] = TEMP_SIZE_UPDATE;
			  DATA_FROM_FLASH[11] = SDA_CALLIBRATION_FACTOR;
			  DATA_FROM_FLASH[12] = DATA_FOR_FLASH[1];
			  DATA_FROM_FLASH[13] = DATA_FOR_FLASH[2];
			  DATA_FROM_FLASH[14] = DATA_FOR_FLASH[3];
			  DATA_FROM_FLASH[15] = DATA_FOR_FLASH[4];
			  SLAVE_Write_Data(address_to_write, (char*)DATA_FROM_FLASH,200);
			  break;
		  case LORA_PARAMETERS:
			  address_to_write = STARTING_ADD_OF_ENGG_MODE;
			  address_to_read = STARTING_ADD_OF_ENGG_MODE;
			  SLAVE_Read_Data(address_to_read, (char*)DATA_FROM_FLASH,200);
			  SLAVE_Erase_4K(address_to_write);
			  DATA_FROM_FLASH[0] = 0;
			  DATA_FROM_FLASH[1] = 0;
			  DATA_FROM_FLASH[2] = 0;
			  DATA_FROM_FLASH[3] = ENGG_MODE_DATA;
			  DATA_FROM_FLASH[4] = SIZE_UPDATE;
			  DATA_FROM_FLASH[5] = TEMP_SIZE_UPDATE;
			  DATA_FROM_FLASH[16] = LORA_PARAMETERS;
			  for(int i =17;i<29;i++)
			  {
				  DATA_FROM_FLASH[i] = DATA_FOR_FLASH[i-16];
			  }
			  SLAVE_Write_Data(address_to_write, (char*)DATA_FROM_FLASH,200);
			  break;
		  case KP_KI_KD_DATA:
			  address_to_write = STARTING_ADD_OF_ENGG_MODE;
			  address_to_read = STARTING_ADD_OF_ENGG_MODE;
			  SLAVE_Read_Data(address_to_read, (char*)DATA_FROM_FLASH,200);
			  SLAVE_Erase_4K(address_to_write);
			  DATA_FROM_FLASH[0] = 0;
			  DATA_FROM_FLASH[1] = 0;
			  DATA_FROM_FLASH[2] = 0;
			  DATA_FROM_FLASH[3] = ENGG_MODE_DATA;
			  DATA_FROM_FLASH[4] = SIZE_UPDATE;
			  DATA_FROM_FLASH[5] = TEMP_SIZE_UPDATE;
			  DATA_FROM_FLASH[36] = KP_KI_KD_DATA;
			  for(int i =37;i<101;i++)
			  {
				  DATA_FROM_FLASH[i] = DATA_FOR_FLASH[i];
			  }
			  SLAVE_Write_Data(address_to_write, (char*)DATA_FROM_FLASH,200);
			  break;

		  case UPDATE_PROFILE:
			  address_to_write = STARTING_ADD_OF_PROFILES;
			  SLAVE_Erase_4K(address_to_write);
			  for(int j =0;j<50;j++)
			  {
			  for(int i =0;i<50;i++)
			  {
				  DATA_FROM_FLASH[i] = PROFILES[j][i];
			  }
			  SLAVE_Write_Data(address_to_write, (char*)DATA_FROM_FLASH,50);
			  address_to_write = address_to_write+increment;
			  }
			  osThreadResume(PROFILE_STARTHandle);
			  break;
		  case SYSTEM_STATE:
			  address_to_write = STARTING_ADD_OF_ENGG_MODE;
			  address_to_read = STARTING_ADD_OF_ENGG_MODE;
			  SLAVE_Read_Data(address_to_read, (char*)DATA_FROM_FLASH,200);
			  SLAVE_Erase_4K(address_to_write);
			  for(int i =138;i<191;i++)
			  {
				  DATA_FROM_FLASH[i]=DATA_FOR_FLASH[i-138];
			  }
			  SLAVE_Write_Data(address_to_write, (char*)DATA_FROM_FLASH,200);
			  break;
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END Start_FLASH_WRITE */
}

/* USER CODE BEGIN Header_Start_POWER_ON_TEST */
/**
* @brief Function implementing the POWER_ON_TEST thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_POWER_ON_TEST */
void Start_POWER_ON_TEST(void const * argument)
{
  /* USER CODE BEGIN Start_POWER_ON_TEST */
	PCA9685_STATUS pwmstatus;

	Lcd_init();
	LCD_setCursor_xy(0, 3);
	Lcd_send_string("*** MCTC ***");
	LCD_setCursor_xy(1,0);
	Lcd_send_string("  ARIHANT LTD.  ");
	char Data[] = "ab";
	char RXData[2]={0};
  /* Infinite loop */
  for(;;)
  {
	SLAVE_Read_ID();
	SLAVE_Read_StatusReg1();
    SLAVE_Read_StatusReg2();
	SLAVE_Clear_StatusReg();
	SLAVE_Read_StatusReg1();
	SLAVE_Read_StatusReg2();
	SLAVE_Write_Data(0x00, Data,2);
	SLAVE_Read_Data(0x00,RXData,2);
	if(RXData[0]=='a' && RXData[1]=='b')
	{
		TCU_ERROR_DATA_FLASH =0;
	}
	else TCU_ERROR_DATA_FLASH =1;

	bool START_STOP = RADIO_START(LORA_DEVICE);
	if(START_STOP == true)
	{
		LORA_ERROR_DATA=0;
	}
	else LORA_ERROR_DATA=1;
	pwmstatus  = PCA9685_SleepMode(1);
	if(pwmstatus == PCA9685_OK)
	{
		TCU_ERROR_DATA_PWM=0;
	}
	else TCU_ERROR_DATA_PWM =1;
	//ESP Verification will be done by UART.

	  osThreadTerminate(POWER_ON_TESTHandle);
    osDelay(1);
  }
  /* USER CODE END Start_POWER_ON_TEST */
}

/* USER CODE BEGIN Header_Start_LCD_TASK */
/**
* @brief Function implementing the LCD_TASK thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_LCD_TASK */
void Start_LCD_TASK(void const * argument)
{
  /* USER CODE BEGIN Start_LCD_TASK */
	LCD_QUEUE *lcdmsg;
	ERROR_D	*errmsg;

	osEvent lcdevt,errevt;

	uint8_t VOLTAGE =0;
	uint8_t CURRENT=0;;
	uint8_t AMB_TEMP=0;
	uint8_t IR_TEMP=0;
	uint8_t HS_LEFT=0;
	uint8_t HS_RIGHT=0;
	uint8_t ERROR_DATA=0;
	uint8_t LCD_DATA_t[2];
	char line1_buffer[16];
	char line2_buffer[16];
	char error_string[2];
	static int page=1;

	TickType_t last_tick = xTaskGetTickCount();
  /* Infinite loop */
  for(;;)
  {
	  errevt = osMessageGet(ERROR_D_qHandle, 0);
	  if(errevt.status == osEventMessage)
	  {
		  errmsg = errevt.value.p;
		  ERROR_DATA = errmsg->error_data_rec[1];
		  osPoolFree(errorpool, errmsg);
	  }
	  lcdevt = osMessageGet(LCD_QUEUE_qHandle, 0);
	  if(lcdevt.status == osEventMessage)
	  {
		  lcdmsg = lcdevt.value.p;
		  for(int i =0;i<2;i++)
		  {
			  LCD_DATA_t[i] = lcdmsg->LCD_DATA[i];
		  }
		  osPoolFree(lcdpool, lcdmsg);
		  switch(LCD_DATA_t[0])
		  {
		  case SYS_VOLTAGE:
			  VOLTAGE = LCD_DATA_t[1];
			  break;
		  case WHOLE_CURRENT:
			  CURRENT = LCD_DATA_t[1];
			  break;
		  case IR_TEMP_t:
			  IR_TEMP = LCD_DATA_t[1];
		  	  break;
		  case HS_LEFT_t:
			  HS_LEFT = LCD_DATA_t[1];
			  break;
		  case HS_RIGHT_t:
			  HS_RIGHT = LCD_DATA_t[1];
			  break;
		  case AMB_TEMP_t:
			  AMB_TEMP = LCD_DATA_t[1];
			  break;
		  }
	  }


	  if((xTaskGetTickCount()-last_tick)>pdMS_TO_TICKS( 4000 ))
	  {
		  last_tick=xTaskGetTickCount();
	  switch(page)
	  {
	  case 1:

		  sprintf(line1_buffer,"SYS VOLTAGE: %dV",VOLTAGE);
		  sprintf(line2_buffer,"SYS CURRENT: %dA",CURRENT);
		  page =2;
		  break;
	  case 2:
		  sprintf(line1_buffer,"AMBIENT: %d C",AMB_TEMP);
		  sprintf(line2_buffer,"IR TEMP: %d C",IR_TEMP);
		  page =3;
		  break;
	  case 3:
		  sprintf(line1_buffer,"L SINK T : %d C",HS_LEFT);
		  sprintf(line2_buffer,"R SINK T : %d C",HS_RIGHT);
		  page =4;
		  break;
	  case 4:
		  sprintf(line1_buffer,"MY TCU ID : %d",MY_TCU_ID);
		  if(ERROR_DATA == ERROR_NO_SDA)
		  {
			  error_string[0] = 'E';
			  error_string[1] = '2';
		  }
		  else
		  {
			  error_string[0] = 'N';
			  error_string[1] = 'O';
		  }
		  sprintf(line2_buffer,"ERROR : %c%c",error_string[0],error_string[1]);
		  page =1;
		  break;
	  }

	  LCD_Clear();
	  LCD_setCursor_xy(0,0);
	  Lcd_send_string(line1_buffer);
	  LCD_setCursor_xy(1,0);
	  Lcd_send_string(line2_buffer);
	  }
    osDelay(10);
  }
  /* USER CODE END Start_LCD_TASK */
}

/* USER CODE BEGIN Header_Start_DATA_LOGGING */
/**
* @brief Function implementing the DATA_LOGGING thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_DATA_LOGGING */
void Start_DATA_LOGGING(void const * argument)
{
  /* USER CODE BEGIN Start_DATA_LOGGING */
	GIVE_ME	*givememsg;
	DATA_LOGGING	*datalogmsg;
	SLAVE_SEND	*slavemsg;
	DATA_OF_SLAVE	*dataofslavemsg;
	LORA_SEND *lorasendmsg;
	CLOSE_CURRENT *clcurrmsg;
//	DATA_LOG	*dlmsg;

	osEvent		datalogevt,dataofslaveevt,/*dlevt,*/clcurrevt;

	uint8_t CLOSE_CURRENT_DATA_t[50];
	uint8_t DATA_TO_SEND[MAX_BUFFER_SIZE];
	uint8_t DATA_FOR_SLAVE[50];
//	uint8_t DATA_FOR_MASTER[50];
	uint8_t CRC_VALUE_FOR_SENDING_DATA;
//	uint8_t CRC_VALUE_FOR_MASTER;
	uint8_t DATA_RECEIVED_FROM_SLAVE[50];
	static uint8_t DATA_RECEIVED[50];
	static int state =0;
	static int notification = 0;
//	static uint8_t NOTIFY_t = 0x00;
  /* Infinite loop */
  for(;;)
  {
	  switch(state)
	  {
	  case 0:
	  clcurrevt = osMailGet(closecurrmail, 0);
	  if(clcurrevt.status == osEventMail)
	  {
		  clcurrmsg = clcurrevt.value.p;
		  for(int i =0;i<50;i++)
		  {
			  CLOSE_CURRENT_DATA_t[i] = clcurrmsg->CLOSE_CURRENT_DATA[i];
		  }
		  osMailFree(closecurrmail, clcurrmsg);
			  if(I_AM == I_AM_MASTER)
			  {

				  if(NUMBER_OF_SYSTEMS == DATA_SIZE_FOR_1SYS)
				  {
		  			  memset(DATA_TO_SEND,0x00,BUFFER_SIZE_ALL);
		  			  DATA_TO_SEND[0] = MODEM_BIT;
		  			  DATA_TO_SEND[1] = MODEM_REPLY_BIT;
		  			  memcpy( &DATA_TO_SEND[2],CLOSE_CURRENT_DATA_t,48);
		  			  CRC_VALUE_FOR_SENDING_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_TO_SEND,(BUFFER_SIZE_ALL-1));
		  			  DATA_TO_SEND[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_SENDING_DATA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
			  			  notification = ulTaskNotifyTake(0,5000);
			  			  if( (notification>=0))
			  			  {
			  			  notification =0;
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_TO_SEND[i];
						  }
						  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
			  			  }
						  break;
					  case LOCAL_SYSTEM:
			  			  if(CLOSE_CURRENT_DATA_t[0]==MY_TCU_ID)
			  			  {
			  				  notification = ulTaskNotifyTake(0,5000);
			  				  if( (notification>=0))
			  				  {
			  				  notification =0;
			  			      HAL_UART_Transmit(&huart6, DATA_TO_SEND, 200, 100);
			  				  }
			  			  }
						  break;
					  }
				  }

				  if(NUMBER_OF_SYSTEMS == DATA_SIZE_FOR_2SYS)
				  {
					  memset(DATA_RECEIVED_FROM_SLAVE,0x00,50);
					  memset(DATA_FOR_SLAVE,0xFF,50);
					  DATA_FOR_SLAVE[0] = MASTER_BIT;
					  DATA_FOR_SLAVE[1] = ID_S_PRESENT[0];
					  DATA_FOR_SLAVE[2] = GIVE_CURRENT;
					  slavemsg = osPoolAlloc(slavesendpool);
					  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
					  if(dataofslaveevt.status == osEventMessage)
					  {
						  dataofslavemsg = dataofslaveevt.value.p;
						  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
						  osPoolFree(dataofslavepool, dataofslavemsg);
					  }
					  memset(DATA_TO_SEND,0x00,BUFFER_SIZE_ALL);
					  DATA_TO_SEND[0] = MODEM_BIT;
					  DATA_TO_SEND[1] = MODEM_REPLY_BIT;
		  			  for(int i=0;i<48;i++)
		  			  {
		  				  DATA_TO_SEND[i+2] = CLOSE_CURRENT_DATA_t[i];
		  			  }
		  			  for(int i =50;i<92;i++)
		  			  {
		  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-49];
		  			  }
					  CRC_VALUE_FOR_SENDING_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_TO_SEND,(BUFFER_SIZE_ALL-1));
					  DATA_TO_SEND[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_SENDING_DATA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  notification = ulTaskNotifyTake(0,5000);
						  if( (notification>=0))
			  			  {
			  			  notification =0;
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_TO_SEND[i];
						  }
						  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
			  			  }
						  break;

					  case LOCAL_SYSTEM:
			  			  if(CLOSE_CURRENT_DATA_t[0]==MY_TCU_ID)
			  			  {
			  				  notification = ulTaskNotifyTake(0,5000);
			  				  if( (notification>=0))
			  				  {
			  				  notification =0;
			  			  HAL_UART_Transmit(&huart6, DATA_TO_SEND, 200, 100);
			  				  }
			  			  }
						  break;
					  }
				  }

				  if(NUMBER_OF_SYSTEMS == DATA_SIZE_FOR_3SYS)
				  {
					  memset(DATA_FOR_SLAVE,0xFF,50);
					  DATA_FOR_SLAVE[0] = MASTER_BIT;
					  DATA_FOR_SLAVE[1] = ID_S_PRESENT[0];
					  DATA_FOR_SLAVE[2] = GIVE_CURRENT;
					  slavemsg = osPoolAlloc(slavesendpool);
					  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
					  if(dataofslaveevt.status == osEventMessage)
					  {
						  dataofslavemsg = dataofslaveevt.value.p;
						  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
						  osPoolFree(dataofslavepool, dataofslavemsg);
					  }
					  memset(DATA_TO_SEND,0x00,BUFFER_SIZE_ALL);
					  DATA_TO_SEND[0] = MODEM_BIT;
					  DATA_TO_SEND[1] = MODEM_REPLY_BIT;
		  			  for(int i=0;i<48;i++)
		  			  {
		  				  DATA_TO_SEND[i+2] = CLOSE_CURRENT_DATA_t[i];
		  			  }
		  			  for(int i =50;i<92;i++)
		  			  {
		  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-49];
		  			  }
					  memset(DATA_RECEIVED_FROM_SLAVE,0x00,50);
					  memset(DATA_FOR_SLAVE,0xFF,50);
					  DATA_FOR_SLAVE[0] = MASTER_BIT;
					  DATA_FOR_SLAVE[1] = ID_S_PRESENT[1];
					  DATA_FOR_SLAVE[2] = GIVE_CURRENT;
					  slavemsg = osPoolAlloc(slavesendpool);
					  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
					  if(dataofslaveevt.status == osEventMessage)
					  {
						  dataofslavemsg = dataofslaveevt.value.p;
						  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
						  osPoolFree(dataofslavepool, dataofslavemsg);
					  }
		  			  for(int i =92;i<134;i++)
		  			  {
		  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-91];
		  			  }
					  CRC_VALUE_FOR_SENDING_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_TO_SEND,(BUFFER_SIZE_ALL-1));
					  DATA_TO_SEND[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_SENDING_DATA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  notification = ulTaskNotifyTake(0,5000);
						  if( (notification>=0))
			  			  {
			  			  notification =0;
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_TO_SEND[i];
						  }
						  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
			  			  }

						  break;

					  case LOCAL_SYSTEM:
			  			  if(CLOSE_CURRENT_DATA_t[0]==MY_TCU_ID)
			  			  {
			  				  notification = ulTaskNotifyTake(0,5000);
			  				  if( (notification>=0))
			  				  {
			  				  notification =0;
			  				  HAL_UART_Transmit(&huart6, DATA_TO_SEND, 200, 100);
			  				  }
			  			  }
						  break;
					  }
				  }

				  if(NUMBER_OF_SYSTEMS == DATA_SIZE_FOR_4SYS)
				  {
					  memset(DATA_FOR_SLAVE,0xFF,50);
					  DATA_FOR_SLAVE[0] = MASTER_BIT;
					  DATA_FOR_SLAVE[1] = ID_S_PRESENT[0];
					  DATA_FOR_SLAVE[2] = GIVE_CURRENT;
					  slavemsg = osPoolAlloc(slavesendpool);
					  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
					  if(dataofslaveevt.status == osEventMessage)
					  {
						  dataofslavemsg = dataofslaveevt.value.p;
						  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
						  osPoolFree(dataofslavepool, dataofslavemsg);
					  }
					  memset(DATA_TO_SEND,0x00,BUFFER_SIZE_ALL);
					  DATA_TO_SEND[0] = MODEM_BIT;
					  DATA_TO_SEND[1] = MODEM_REPLY_BIT;
		  			  for(int i=0;i<48;i++)
		  			  {
		  				  DATA_TO_SEND[i+2] = CLOSE_CURRENT_DATA_t[i];
		  			  }
		  			  for(int i =50;i<92;i++)
		  			  {
		  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-49];
		  			  }
					  memset(DATA_RECEIVED_FROM_SLAVE,0x00,50);
					  memset(DATA_FOR_SLAVE,0xFF,50);
					  DATA_FOR_SLAVE[0] = MASTER_BIT;
					  DATA_FOR_SLAVE[1] = ID_S_PRESENT[1];
					  DATA_FOR_SLAVE[2] = GIVE_CURRENT;
					  slavemsg = osPoolAlloc(slavesendpool);
					  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
					  if(dataofslaveevt.status == osEventMessage)
					  {
						  dataofslavemsg = dataofslaveevt.value.p;
						  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
						  osPoolFree(dataofslavepool, dataofslavemsg);
					  }
		  			  for(int i =92;i<134;i++)
		  			  {
		  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-91];
		  			  }
					  memset(DATA_RECEIVED_FROM_SLAVE,0x00,50);
					  memset(DATA_FOR_SLAVE,0xFF,50);
					  DATA_FOR_SLAVE[0] = MASTER_BIT;
					  DATA_FOR_SLAVE[1] = ID_S_PRESENT[2];
					  DATA_FOR_SLAVE[2] = GIVE_DATA;
					  slavemsg = osPoolAlloc(slavesendpool);
					  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
					  if(dataofslaveevt.status == osEventMessage)
					  {
						  dataofslavemsg = dataofslaveevt.value.p;
						  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
						  osPoolFree(dataofslavepool, dataofslavemsg);
					  }
		  			  for(int i =134;i<176;i++)
		  			  {
		  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-133];
		  			  }
					  CRC_VALUE_FOR_SENDING_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_TO_SEND,(BUFFER_SIZE_ALL-1));
					  DATA_TO_SEND[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_SENDING_DATA;
					  switch(WHOM_TO_GIVE_DATA)
					  {
					  case MODEM_SYSTEM:
						  notification = ulTaskNotifyTake(0,5000);
						  if( (notification>=0))
			  			  {
			  			  notification =0;
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_TO_SEND[i];
						  }
						  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
			  			  }

						  break;

					  case LOCAL_SYSTEM:
				  			  if(CLOSE_CURRENT_DATA_t[0]==MY_TCU_ID)
			  			  {
			  				  notification = ulTaskNotifyTake(0,5000);
			  				  if( (notification>=0))
			  				  {
			  				  notification =0;
			  				  HAL_UART_Transmit(&huart6, DATA_TO_SEND, 200, 100);
			  				  }
			  			  }
						  break;
					  }
				  }
				  state =1;
			  }
//			  if(I_AM == I_AM_SLAVE)
//			  {
//				  dlevt = osMessageGet(DATA_LOG_qHandle, osWaitForever);
//				  if(dlevt.status == osEventMessage)
//				  {
//					  dlmsg = dlevt.value.p;
//					  NOTIFY_t = dlmsg->NOTIFY;
//					  osPoolFree(datalogpool, dlmsg);
//					  if(NOTIFY_t == 0x01)
//					  {
//						  NOTIFY_t =0x00;
//						  DATA_FOR_MASTER[0] = SLAVE_BIT;
//							for(int i =0;i<42;i++)
//							{
//								DATA_FOR_MASTER[i+1] = CLOSE_CURRENT_DATA_t[i];
//							}
//						  CRC_VALUE_FOR_MASTER = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FOR_MASTER,49);
//						  DATA_FOR_MASTER[49] = CRC_VALUE_FOR_MASTER;
//						  HAL_UART_Transmit(&huart5, DATA_FOR_MASTER, 50,100);
//					  }
//				  }
//				  state =1;
//			  }

	  }
	  state =1;
	  break;
	  case 1:
		  if(I_AM == I_AM_MASTER)
		  {
	  givememsg = osPoolAlloc(givepool);
	  givememsg->GIVE_ME_t = I_AM_DATA_LOGG;
	  osMessagePut(GIVE_ME_qHandle, (uint32_t)givememsg,100);
	  datalogevt = osMessageGet(DATA_LOGGING_qHandle, 5000);
	  if(datalogevt.status == osEventMessage)
	  {
		  datalogmsg = datalogevt.value.p;
		  memcpy(DATA_RECEIVED,datalogmsg->DATA_LOGGING_DATA,50);
		  osPoolFree(datalogpool,datalogmsg);
		  if(NUMBER_OF_SYSTEMS == DATA_SIZE_FOR_1SYS)
			  {
  			  memset(DATA_TO_SEND,0x00,BUFFER_SIZE_ALL);
  			  DATA_TO_SEND[0] = MODEM_BIT;
  			  DATA_TO_SEND[1] = MODEM_REPLY_BIT;
  			  for(int i=0;i<48;i++)
  			  {
  				  DATA_TO_SEND[i+2] = DATA_RECEIVED[i];
  			  }
  			  CRC_VALUE_FOR_SENDING_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_TO_SEND,(BUFFER_SIZE_ALL-1));
  			  DATA_TO_SEND[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_SENDING_DATA;
				  switch(WHOM_TO_GIVE_DATA)
				  {
				  case MODEM_SYSTEM:

				  			  notification = ulTaskNotifyTake(0,5000);
				  			  if( (notification>=0))
				  			  {
				  			  notification =0;
							  lorasendmsg = osPoolAlloc(lorasendpool);
							  for(int i =0;i<BUFFER_SIZE_ALL;i++)
							  {
								  lorasendmsg->LORA_SEND_DATA[i] = DATA_TO_SEND[i];
							  }
							  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
				  			  }

				  			  break;
				  		  case LOCAL_SYSTEM:
				  			  if(DATA_RECEIVED[0]==MY_TCU_ID)
				  			  {
					  			  notification = ulTaskNotifyTake(0,5000);
					  			  if( (notification>=0))
					  			  {
					  			  notification =0;
				  			  HAL_UART_Transmit(&huart6, DATA_TO_SEND, 200, 100);
					  			  }
				  			  }
				  			  break;
				  }

			  }
			  if(NUMBER_OF_SYSTEMS == DATA_SIZE_FOR_2SYS)
			  {
				  memset(DATA_RECEIVED_FROM_SLAVE,0x00,50);
					  memset(DATA_FOR_SLAVE,0xFF,50);
					  DATA_FOR_SLAVE[0] = MASTER_BIT;
					  DATA_FOR_SLAVE[1] = ID_S_PRESENT[0];
					  DATA_FOR_SLAVE[2] = GIVE_DATA;
					  slavemsg = osPoolAlloc(slavesendpool);
					  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
					  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
					  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
					  if(dataofslaveevt.status == osEventMessage)
					  {
						  dataofslavemsg = dataofslaveevt.value.p;
						  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
						  osPoolFree(dataofslavepool, dataofslavemsg);
					  }
					  memset(DATA_TO_SEND,0x00,BUFFER_SIZE_ALL);
					  DATA_TO_SEND[0] = MODEM_BIT;
					  DATA_TO_SEND[1] = MODEM_REPLY_BIT;
		  			  for(int i=0;i<48;i++)
		  			  {
		  				  DATA_TO_SEND[i+2] = DATA_RECEIVED[i];
		  			  }
		  			  for(int i =50;i<92;i++)
		  			  {
		  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-49];
		  			  }
					  CRC_VALUE_FOR_SENDING_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_TO_SEND,(BUFFER_SIZE_ALL-1));
					  DATA_TO_SEND[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_SENDING_DATA;
				  switch(WHOM_TO_GIVE_DATA)
				  {
				  case MODEM_SYSTEM:
		  			  notification = ulTaskNotifyTake(0,5000);
		  			  if( (notification>=0))
		  			  {
		  			  notification =0;
					  lorasendmsg = osPoolAlloc(lorasendpool);
					  for(int i =0;i<BUFFER_SIZE_ALL;i++)
					  {
						  lorasendmsg->LORA_SEND_DATA[i] = DATA_TO_SEND[i];
					  }
					  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
		  			  }

					  break;

				  case LOCAL_SYSTEM:
		  			  if(DATA_RECEIVED[0]==MY_TCU_ID)
		  			  {
			  			  notification = ulTaskNotifyTake(0,5000);
			  			  if( (notification>=0))
			  			  {
			  			  notification =0;
		  			  HAL_UART_Transmit(&huart6, DATA_TO_SEND, 200, 100);
			  			  }
		  			  }

					  break;
				  }
			  }
			  if(NUMBER_OF_SYSTEMS == DATA_SIZE_FOR_3SYS)
			  {
				  memset(DATA_FOR_SLAVE,0xFF,50);
						  DATA_FOR_SLAVE[0] = MASTER_BIT;
						  DATA_FOR_SLAVE[1] = ID_S_PRESENT[0];
						  DATA_FOR_SLAVE[2] = GIVE_DATA;
						  slavemsg = osPoolAlloc(slavesendpool);
						  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
						  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
						  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
						  if(dataofslaveevt.status == osEventMessage)
						  {
							  dataofslavemsg = dataofslaveevt.value.p;
							  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
     						  osPoolFree(dataofslavepool, dataofslavemsg);
						  }
						  memset(DATA_TO_SEND,0x00,BUFFER_SIZE_ALL);
						  DATA_TO_SEND[0] = MODEM_BIT;
						  DATA_TO_SEND[1] = MODEM_REPLY_BIT;
			  			  for(int i=0;i<48;i++)
			  			  {
			  				  DATA_TO_SEND[i+2] = DATA_RECEIVED[i];
			  			  }
			  			  for(int i =50;i<92;i++)
			  			  {
			  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-49];
			  			  }
						  memset(DATA_RECEIVED_FROM_SLAVE,0x00,50);
						  memset(DATA_FOR_SLAVE,0xFF,50);
						  DATA_FOR_SLAVE[0] = MASTER_BIT;
						  DATA_FOR_SLAVE[1] = ID_S_PRESENT[1];
						  DATA_FOR_SLAVE[2] = GIVE_DATA;
						  slavemsg = osPoolAlloc(slavesendpool);
						  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
						  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);
						  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
						  if(dataofslaveevt.status == osEventMessage)
						  {
							  dataofslavemsg = dataofslaveevt.value.p;

							  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);

							  osPoolFree(dataofslavepool, dataofslavemsg);
						  }
			  			  for(int i =92;i<134;i++)
			  			  {
			  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-91];
			  			  }
						  CRC_VALUE_FOR_SENDING_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_TO_SEND,(BUFFER_SIZE_ALL-1));
						  DATA_TO_SEND[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_SENDING_DATA;
				  switch(WHOM_TO_GIVE_DATA)
				  {
				  case MODEM_SYSTEM:

		  			  notification = ulTaskNotifyTake(0,5000);
		  			  if( (notification>=0))
		  			  {
		  			  notification =0;
					  lorasendmsg = osPoolAlloc(lorasendpool);
					  for(int i =0;i<BUFFER_SIZE_ALL;i++)
					  {
						  lorasendmsg->LORA_SEND_DATA[i] = DATA_TO_SEND[i];
					  }
					  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
		  			  }
					  break;

				  case LOCAL_SYSTEM:
		  			  if(DATA_RECEIVED[0]==MY_TCU_ID)
		  			  {
			  			  notification = ulTaskNotifyTake(0,5000);
			  			  if( (notification>=0))
			  			  {
			  			  notification =0;
			  			  HAL_UART_Transmit(&huart6, DATA_TO_SEND, 200, 100);
			  			  }
		  			  }

					  break;
				  }
			  }
			  if(NUMBER_OF_SYSTEMS == DATA_SIZE_FOR_4SYS)
			  {
				  memset(DATA_FOR_SLAVE,0xFF,50);
				  DATA_FOR_SLAVE[0] = MASTER_BIT;
				  DATA_FOR_SLAVE[1] = ID_S_PRESENT[0];
				  DATA_FOR_SLAVE[2] = GIVE_DATA;
				  slavemsg = osPoolAlloc(slavesendpool);
				  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
				  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);

				  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
				  if(dataofslaveevt.status == osEventMessage)
				  {
					  dataofslavemsg = dataofslaveevt.value.p;
					  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
					  osPoolFree(dataofslavepool, dataofslavemsg);
				  }

				  memset(DATA_TO_SEND,0x00,BUFFER_SIZE_ALL);
				  DATA_TO_SEND[0] = MODEM_BIT;
				  DATA_TO_SEND[1] = MODEM_REPLY_BIT;
	  			  for(int i=0;i<48;i++)
	  			  {
	  				  DATA_TO_SEND[i+2] = DATA_RECEIVED[i];
	  			  }
	  			  for(int i =50;i<92;i++)
	  			  {
	  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-49];
	  			  }
				  memset(DATA_RECEIVED_FROM_SLAVE,0x00,50);
				  memset(DATA_FOR_SLAVE,0xFF,50);
				  DATA_FOR_SLAVE[0] = MASTER_BIT;
				  DATA_FOR_SLAVE[1] = ID_S_PRESENT[1];
				  DATA_FOR_SLAVE[2] = GIVE_DATA;
				  slavemsg = osPoolAlloc(slavesendpool);
				  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
				  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);

				  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
				  if(dataofslaveevt.status == osEventMessage)
				  {
					  dataofslavemsg = dataofslaveevt.value.p;
					  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
					  osPoolFree(dataofslavepool, dataofslavemsg);
				  }
	  			  for(int i =92;i<134;i++)
	  			  {
	  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-91];
	  			  }
				  memset(DATA_RECEIVED_FROM_SLAVE,0x00,50);
				  memset(DATA_FOR_SLAVE,0xFF,50);
				  DATA_FOR_SLAVE[0] = MASTER_BIT;
				  DATA_FOR_SLAVE[1] = ID_S_PRESENT[2];
				  DATA_FOR_SLAVE[2] = GIVE_DATA;
				  slavemsg = osPoolAlloc(slavesendpool);
				  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
				  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);

				  dataofslaveevt = osMessageGet(DATA_OF_SLAVE_qHandle, 7000);
				  if(dataofslaveevt.status == osEventMessage)
				  {
					  dataofslavemsg = dataofslaveevt.value.p;
					  memcpy(DATA_RECEIVED_FROM_SLAVE,dataofslavemsg->DATA_FROM_SLAVE_t,50);
					  osPoolFree(dataofslavepool, dataofslavemsg);
				  }
	  			  for(int i =134;i<176;i++)
	  			  {
	  				  DATA_TO_SEND[i] = DATA_RECEIVED_FROM_SLAVE[i-133];
	  			  }
				  CRC_VALUE_FOR_SENDING_DATA = HAL_CRC_Calculate(&hcrc,(uint32_t*) DATA_TO_SEND,(BUFFER_SIZE_ALL-1));
				  DATA_TO_SEND[(BUFFER_SIZE_ALL-1)] = CRC_VALUE_FOR_SENDING_DATA;
				  switch(WHOM_TO_GIVE_DATA)
					  {

					  case MODEM_SYSTEM:
			  			  notification = ulTaskNotifyTake(0,5000);
			  			  if( (notification>=0))
			  			  {
			  			  notification =0;
						  lorasendmsg = osPoolAlloc(lorasendpool);
						  for(int i =0;i<BUFFER_SIZE_ALL;i++)
						  {
							  lorasendmsg->LORA_SEND_DATA[i] = DATA_TO_SEND[i];
						  }
						  osMessagePut(LORA_SEND_qHandle,(uint32_t) lorasendmsg,100);
			  			  }

						  break;

					  case LOCAL_SYSTEM:
			  			  if(DATA_RECEIVED[0]==MY_TCU_ID)
			  			  {
				  			  notification = ulTaskNotifyTake(0,5000);
				  			  if( (notification>=0))
				  			  {
				  			  notification =0;
			  			  HAL_UART_Transmit(&huart6, DATA_TO_SEND, 200, 100);
				  			  }
			  			  }

						  break;
					  }
			  }
			  state =0;
		  }
//		  if(I_AM == I_AM_SLAVE)
//		  {
//			  state =0;
//		  }
	  }
		  state =0;

	  break;
	  }
    osDelay(125);
  }
  /* USER CODE END Start_DATA_LOGGING */
}

/* USER CODE BEGIN Header_Start_CURRENT_PROCESS */
/**
* @brief Function implementing the CURRENT_PROCESS thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_CURRENT_PROCESS */
void Start_CURRENT_PROCESS(void const * argument)
{
  /* USER CODE BEGIN Start_CURRENT_PROCESS */


	static uint8_t CHANNEL_DATA[32]={0};
	uint8_t TEMP_DATA[40];
	uint16_t ADC_READING[16];
	float	ADC_RAEDING_VOLTS[16];
	int j =0;
	float CURRENT[16];
	uint8_t CURRENT_8BIT[32];
	float HS_TABLE_CURRENT[200][16];
	float FINAL_CURRENT[16] = {0};
	static float OFFSET = 33;
	static int k =0;
	CLOSE_CURRENT	*clcurrmsg;
	DATA_PROCESS *datap_msg;
	uint8_t CLOSE_CURRENT_VALUES[50];
	DUTY_Q	*dutymsg;
	osEvent dutyevt,evtcurr;
	static float OP_DUTY[16];
	float AVG_buffer_curr[200][16]={0};
	static int m =0;
	float AVG_CURRENT[16] = {0};
	WIFI_CURR *wificurrmsg;
	CURRENT_P *msgcurr;


  /* Infinite loop */
  for(;;)
  {
	  dutyevt = osMessageGet(DUTY_Q_qHandle, 0);
	  if(dutyevt.status == osEventMessage)
	  {
		  dutymsg = dutyevt.value.p;
		  for(int i =0;i<16;i++)
		  {
			  OP_DUTY[i] = (((float)(dutymsg->DUTY_t[i]))/(float)10000);
		  }
		  osPoolFree(dutypool, dutymsg);
	  }
	  evtcurr = osMessageGet(CURRENT_P_qHandle, 0);
	  if(evtcurr.status == osEventMessage)
	  {
		  msgcurr = evtcurr.value.p;
		  for(int i =0;i<40;i++)
	  {
			  TEMP_DATA[i]  = msgcurr->CURRENT_P_DATA[i];
	  }
		  osPoolFree(currppool, msgcurr);
		  if(TEMP_DATA[0] == ADC_12_CH_DATA)
		  {
			  for(int i =0;i<26;i++)
			  {
				  CHANNEL_DATA[i] = TEMP_DATA[i+4];
			  }
		  }
		  if(TEMP_DATA[36] == ADC_13_CH_DATA)
		  {
			  CHANNEL_DATA[26]=TEMP_DATA[30];
			  CHANNEL_DATA[27] = TEMP_DATA[31];
		  }
		  if(TEMP_DATA[37] == ADC_14_CH_DATA)
		  {
			  CHANNEL_DATA[28]=TEMP_DATA[32];
			  CHANNEL_DATA[29] = TEMP_DATA[33];
		  }
		  if(TEMP_DATA[38] == ADC_15_CH_DATA)
		  {
			  CHANNEL_DATA[30]=TEMP_DATA[34];
			  CHANNEL_DATA[31] = TEMP_DATA[35];
		  }


			for(int i =0;i<16;i++)
			{
			ADC_READING[i] = ((((((uint16_t)CHANNEL_DATA[j])<<8)&0xFF00)|((((uint16_t)CHANNEL_DATA[j+1]))&0x00FF))&0x0FFF);
			j=j+2;
			ADC_RAEDING_VOLTS[i] = ((ADC_READING[i]*3.3)/4096)-(OFFSET/100);

			CURRENT[i] = ((ADC_RAEDING_VOLTS[i])/(float)0.05);

			}
			memset(CHANNEL_DATA,0,sizeof(CHANNEL_DATA));

			j=0;


			for(int n =0;n<16;n++)
			{
				if((ADC_READING[n]>330)&&(ADC_READING[n]<2330))
				{
			      AVG_buffer_curr[m][n] = CURRENT[n];
				}
			}
			m++;
			  if(m>=200)
			  {
				  m=0;
			  }

			for(int p =0;p<16;p++)
			{
				for(int q =0;q<200;q++)
				{
					AVG_CURRENT[p] = AVG_CURRENT[p] + AVG_buffer_curr[q][p];
				}
				AVG_CURRENT[p] = AVG_CURRENT[p]/20;
			}


			  k=0;
						  for(int i =0;i<16;i++)
						  {
							  CURRENT_8BIT[k] = ((((int)AVG_CURRENT[i]) >>8)&0x00FF);
							  CURRENT_8BIT[k+1] = ((int)AVG_CURRENT[i])&0x00FF;
							  k=k+2;
							  if(k==32)
							  {
								  k=0;
							  }
						  }

						if(LOOP_RUNNING == CLOSE_LOOP)
						{
							clcurrmsg = osMailAlloc(closecurrmail, 100);
							wificurrmsg = osMailAlloc(wificurrmail,100);
							CLOSE_CURRENT_VALUES[0] = MY_TCU_ID;
							CLOSE_CURRENT_VALUES[1] = DATA_TYPE_CURRENT;
							for(int i =0;i<32;i++)
							{
								CLOSE_CURRENT_VALUES[i+2] = CURRENT_8BIT[i];
							}
							memcpy(clcurrmsg->CLOSE_CURRENT_DATA,CLOSE_CURRENT_VALUES,50);
							osMailPut(closecurrmail, clcurrmsg);
							memcpy(wificurrmsg->CURR_DATA_FOR_WIFI,CLOSE_CURRENT_VALUES,50);
							osMailPut(wificurrmail, wificurrmsg);
						}
						if(LOOP_RUNNING == OPEN_LOOP)
						{
							  datap_msg = osPoolAlloc(datappool);
							  wificurrmsg = osMailAlloc(wificurrmail,100);
							  for(int i =0;i<16;i++)
							  {
								  datap_msg->DATA_PROCESS_DATA[i]=(int)((AVG_CURRENT[i])*OP_DUTY[i]);
							  }
							  osMessagePut(DATA_PROCESS_qHandle, (uint32_t)datap_msg,100);
								clcurrmsg = osMailAlloc(closecurrmail, 100);
								CLOSE_CURRENT_VALUES[0] = MY_TCU_ID;
								CLOSE_CURRENT_VALUES[1] = DATA_TYPE_CURRENT;
								for(int i =0;i<32;i++)
								{
									CLOSE_CURRENT_VALUES[i+2] = CURRENT_8BIT[i];
								}
								memcpy(clcurrmsg->CLOSE_CURRENT_DATA,CLOSE_CURRENT_VALUES,50);
								memcpy(wificurrmsg->CURR_DATA_FOR_WIFI,CLOSE_CURRENT_VALUES,50);
								osMailPut(wificurrmail, wificurrmsg);
						}


		memset(AVG_CURRENT,0,sizeof(AVG_CURRENT));
		memset(FINAL_CURRENT,0,sizeof(FINAL_CURRENT));
		memset(HS_TABLE_CURRENT,0,sizeof(HS_TABLE_CURRENT));
	  }
	  osDelay(1);
  }
  /* USER CODE END Start_CURRENT_PROCESS */
}

/* USER CODE BEGIN Header_Start_PID_REGULATION */
/**
* @brief Function implementing the PID_REGULATION thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_PID_REGULATION */
void Start_PID_REGULATION(void const * argument)
{
  /* USER CODE BEGIN Start_PID_REGULATION */
	PID_DATA	*pidmsg;
	CURRENT_LIMIT_Q *currlmsg;

	osEvent pidevt,currlevt;

	static float DUTY_DATA[16];
	int whole_current_value =1;
	static int state =0,x=0,y=0;

	PCA9685_SoftwareReset();
	PCA9685_SetPwmFrequency(50);
	PCA9685_AutoIncrement(1);
  /* Infinite loop */
  for(;;)
  {
	  currlevt = osMessageGet(CURRENT_LIMIT_Q_qHandle, 0);
	  if(currlevt.status == osEventMessage)
	  {
		  currlmsg = currlevt.value.p;
		  current_limit_value = currlmsg->CURRENT_LIMIT_DATA;
		  whole_current_value = currlmsg->WHOLE_CURRENT_DATA;
		  osPoolFree(currlimitpool, currlmsg);
	  }

	  pidevt = osMessageGet(PID_DATA_qHandle, 0);
	  if(pidevt.status == osEventMessage)
	  {

		  pidmsg = pidevt.value.p;
		  for(int i =0;i<16;i++)
		  {
			  DUTY_DATA[i] = pidmsg->PID_DATA_t[i];
			  DUTY_DATA[i] = DUTY_DATA[i]/(float)1000;
		  }
		  osPoolFree(pidpool, pidmsg);

			  PCA9685_SoftwareReset();
			  PCA9685_SetPwmFrequency(50);
			  PCA9685_AutoIncrement(1);

			  if(whole_current_value>current_limit_value)
			  {
		 	  switch(state)
		 	  {
		 	  case 0:
		 		  for(int i =0;i<8;i++)
		 		  {
		 		  x = i*2;
		 		  y = x+1;
		 		 PCA9685_SetDuty(x, DUTY_DATA[x]);
		 		 PCA9685_SetDuty(y,0);
		 		  }
		 		  state =1;
		 		  break;
		 	  case 1:
		 		  for(int i =0;i<8;i++)
		 		  {

		 		  x = (i*2)+1;
		 		  y = x-1;
		 		 PCA9685_SetDuty(x, DUTY_DATA[x]);
		 		 PCA9685_SetDuty(y,0);
		 		  }
		 		  state =0;
		 		  break;

		 	  }
			  }
			  if(whole_current_value<current_limit_value)
			  {
				  for(int i =0;i<16;i++)
				  {
				 		 PCA9685_SetDuty(i, DUTY_DATA[i]);
				  }
			  }
	  }


    osDelay(1);
  }
  /* USER CODE END Start_PID_REGULATION */
}

/* USER CODE BEGIN Header_Start_DATA_LOG_FLASH */
/**
* @brief Function implementing the DATA_LOG_FLASH thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_DATA_LOG_FLASH */
void Start_DATA_LOG_FLASH(void const * argument)
{
  /* USER CODE BEGIN Start_DATA_LOG_FLASH */
	FLASH_LOGGING *logmsg;

	osEvent	logevt;

	RTC_TimeTypeDef gTime;
	RTC_DateTypeDef gDate;

	uint8_t DATA_TO_LOG[56];
	const uint32_t increment = 0x38;
	static uint32_t address_to_write = STARTING_ADD_REC_DATA;
	static uint32_t address_to_erase = STARTING_ADD_REC_DATA;
  /* Infinite loop */
  for(;;)
  {
	logevt = osMessageGet(FLASH_LOGGINGHandle, 0);
	if(logevt.status == osEventMessage)
	{
		logmsg = logevt.value.p;
		for(int i =6;i<56;i++)
		{
			DATA_TO_LOG[i] = logmsg->FLASH_DATA_LOGG[i-6];
		}
		osPoolFree(flashlogpool,logmsg);

		HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
		DATA_TO_LOG[0] = gDate.Date;
		DATA_TO_LOG[1] = gDate.Month;
		DATA_TO_LOG[2] = gDate.Year;
		DATA_TO_LOG[3] = gTime.Hours;
		DATA_TO_LOG[4] = gTime.Minutes;
		DATA_TO_LOG[5] = gTime.Seconds;

		  SLAVE_Write_Data(address_to_write, (char*)DATA_TO_LOG,56);
		  address_to_write = address_to_write + increment;
		  if(address_to_write >= ENDING_ADD_REC_DATA)
		  {
			  while(address_to_erase>=ENDING_ADD_REC_DATA)
			  {
			  SLAVE_Erase_4K(address_to_erase);
			  address_to_erase = address_to_erase+ 0x00001000;
			  }
		  }
	}
    osDelay(log_interval);
  }
  /* USER CODE END Start_DATA_LOG_FLASH */
}

/* USER CODE BEGIN Header_Start_FLASH_READ */
/**
* @brief Function implementing the FLASH_READ thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_FLASH_READ */
void Start_FLASH_READ(void const * argument)
{
  /* USER CODE BEGIN Start_FLASH_READ */
	FLASH_READ	*freadmsg;
	LORA_SEND	*loramsg;
	SLAVE_SEND	*slavemsg;

	osEvent freadevt;

	uint8_t READ_FROM;
	uint8_t DATA_FROM_FLASH[200];
	uint8_t DATA_OF_BIST[200];
	uint8_t CRC_VALUE_OF_SENDING_DATA;
	uint8_t REC_DATA[200];
	uint8_t DATA_FOR_SLAVE[50];
	uint32_t read_from_address;
	static uint32_t increment = 0xC4;
  /* Infinite loop */
  for(;;)
  {
	  freadevt = osMessageGet(FLASH_READ_qHandle, 0);
	  if(freadevt.status == osEventMessage)
	  {
		  freadmsg = freadevt.value.p;
		  READ_FROM = freadmsg->FLASH_READ_DATA[0];
		  osPoolFree(flashreadpool, freadmsg);
		  switch(READ_FROM)
		  {

		  case FETCH_ENNG_MODE:
			  read_from_address = STARTING_ADD_OF_ENGG_MODE;
			  SLAVE_Read_Data(read_from_address, (char*)DATA_FROM_FLASH,200);
			  DATA_FROM_FLASH[0] = MODEM_BIT;
			  DATA_FROM_FLASH[1] = MODEM_REPLY_BIT;
			  DATA_FROM_FLASH[2] = MY_TCU_ID;
			  DATA_FROM_FLASH[5] = TEMP_SIZE_UPDATE;
			  DATA_FROM_FLASH[29] = RADIO_ERROR_DATA_t;
			  DATA_FROM_FLASH[30] = LORA_ERROR_DATA;
			  DATA_FROM_FLASH[31] = TCU_ERROR_DATA_t;
			  DATA_FROM_FLASH[32] = TCU_ERROR_DATA_FLASH;
			  DATA_FROM_FLASH[33] = TCU_ERROR_DATA_PWM;
			  DATA_FROM_FLASH[34] = SDA_ERROR_DATA_t;
			  DATA_FROM_FLASH[35] = SDA_ERROR_DATA;

			  CRC_VALUE_OF_SENDING_DATA = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_FROM_FLASH,199);
			  DATA_FROM_FLASH[199] = CRC_VALUE_OF_SENDING_DATA;

			  switch(WHOM_TO_GIVE_DATA)
			  {
			  case MODEM_SYSTEM:
				  loramsg = osPoolAlloc(lorasendpool);
				  memcpy(loramsg->LORA_SEND_DATA,DATA_FROM_FLASH,200);
				  osMessagePut(LORA_SEND_qHandle, (uint32_t)loramsg,1000);
				  break;
			  case LOCAL_SYSTEM:
				  HAL_UART_Transmit(&huart6, DATA_FROM_FLASH, 200,1000);
				  break;
			  }

			  break;

		  case FETCH_DATA_REC:
			  read_from_address = STARTING_ADD_REC_DATA;

			  while(read_from_address>=ENDING_ADD_REC_DATA)
			  {
				  SLAVE_Read_Data(read_from_address, (char*)DATA_FROM_FLASH,196);
				  REC_DATA[0] = MODEM_BIT;
				  REC_DATA[1] = MODEM_REPLY_BIT;
				  REC_DATA[2] = MY_TCU_ID;

				  for(int i =0;i<196;i++)
				  {
					  REC_DATA[i+3] = (uint8_t)DATA_FROM_FLASH[i];
				  }
					CRC_VALUE_OF_SENDING_DATA = HAL_CRC_Calculate(&hcrc, (uint32_t*)REC_DATA,199);
					REC_DATA[199] = CRC_VALUE_OF_SENDING_DATA;
					switch(WHOM_TO_GIVE_DATA)
					{
					case MODEM_SYSTEM:
					loramsg = osPoolAlloc(lorasendpool);
					for(int i =0;i<200;i++)
					{
					loramsg->LORA_SEND_DATA[i] = REC_DATA[i];
					}
					osMessagePut(LORA_SEND_qHandle, (uint32_t)loramsg,100);
					memset(DATA_FROM_FLASH,0,200);

					break;
					case LOCAL_SYSTEM:
					HAL_UART_Transmit(&huart6, REC_DATA, 200,1000);
					memset(DATA_FROM_FLASH,0,200);

					break;
					}
					read_from_address = read_from_address+increment;

			  }
			  break;

		  case FETCH_BIST:
			  DATA_OF_BIST[0] = MODEM_BIT;
			  DATA_OF_BIST[1] = MODEM_REPLY_BIT;
			  DATA_OF_BIST[2] = MY_TCU_ID;
			  DATA_OF_BIST[3] = FETCH_BIST;
			  DATA_OF_BIST[26] = RADIO_ERROR_DATA_t;
			  DATA_OF_BIST[27] = LORA_ERROR_DATA;
			  DATA_OF_BIST[28] = TCU_ERROR_DATA_t;
			  DATA_OF_BIST[29] = TCU_ERROR_DATA_FLASH;
			  DATA_OF_BIST[30] = TCU_ERROR_DATA_PWM;
			  DATA_OF_BIST[31] = SDA_ERROR_DATA_t;
			  DATA_OF_BIST[32] = SDA_ERROR_DATA;

			  memset(DATA_FOR_SLAVE,0xFF,50);
			  DATA_FOR_SLAVE[0] = MASTER_BIT;
			  DATA_FOR_SLAVE[1] = ID_S_PRESENT[0];
			  DATA_FOR_SLAVE[2] = FETCH_BIST;
			  slavemsg = osPoolAlloc(slavesendpool);
			  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
			  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);

			  osDelay(2000);
			  DATA_OF_BIST[105] = BIST_DATA[0];
			  DATA_OF_BIST[106] = BIST_DATA[1];
			  DATA_OF_BIST[107] = BIST_DATA[2];
			  DATA_OF_BIST[108] = BIST_DATA[3];
			  DATA_OF_BIST[109] = BIST_DATA[4];
			  DATA_OF_BIST[110] = BIST_DATA[5];
			  DATA_OF_BIST[111] = BIST_DATA[6];
			  DATA_OF_BIST[112] = BIST_DATA[7];
			  DATA_OF_BIST[113] = BIST_DATA[8];
			  memset(BIST_DATA,0,10);

			  memset(DATA_FOR_SLAVE,0xFF,50);
			  DATA_FOR_SLAVE[0] = MASTER_BIT;
			  DATA_FOR_SLAVE[1] = ID_S_PRESENT[1];
			  DATA_FOR_SLAVE[2] = FETCH_BIST;
			  slavemsg = osPoolAlloc(slavesendpool);
			  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
			  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);

			  osDelay(2000);
			  DATA_OF_BIST[114] = BIST_DATA[0];
			  DATA_OF_BIST[115] = BIST_DATA[1];
			  DATA_OF_BIST[116] = BIST_DATA[2];
			  DATA_OF_BIST[117] = BIST_DATA[3];
			  DATA_OF_BIST[118] = BIST_DATA[4];
			  DATA_OF_BIST[119] = BIST_DATA[5];
			  DATA_OF_BIST[120] = BIST_DATA[6];
			  DATA_OF_BIST[121] = BIST_DATA[7];
			  DATA_OF_BIST[122] = BIST_DATA[8];
			  memset(BIST_DATA,0,10);

			  memset(DATA_FOR_SLAVE,0xFF,50);
			  DATA_FOR_SLAVE[0] = MASTER_BIT;
			  DATA_FOR_SLAVE[1] = ID_S_PRESENT[2];
			  DATA_FOR_SLAVE[2] = FETCH_BIST;
			  slavemsg = osPoolAlloc(slavesendpool);
			  memcpy(slavemsg->SLAVE_DATA,DATA_FOR_SLAVE,50);
			  osMessagePut(SLAVE_SEND_qHandle, (uint32_t)slavemsg,100);

			  osDelay(2000);
			  DATA_OF_BIST[123] = BIST_DATA[0];
			  DATA_OF_BIST[124] = BIST_DATA[1];
			  DATA_OF_BIST[125] = BIST_DATA[2];
			  DATA_OF_BIST[126] = BIST_DATA[3];
			  DATA_OF_BIST[127] = BIST_DATA[4];
			  DATA_OF_BIST[128] = BIST_DATA[5];
			  DATA_OF_BIST[129] = BIST_DATA[6];
			  DATA_OF_BIST[130] = BIST_DATA[7];
			  DATA_OF_BIST[131] = BIST_DATA[8];
			  memset(BIST_DATA,0,10);

			  CRC_VALUE_OF_SENDING_DATA = HAL_CRC_Calculate(&hcrc, (uint32_t*)DATA_OF_BIST,199);
			  DATA_OF_BIST[199] = CRC_VALUE_OF_SENDING_DATA;
			  osThreadSuspend(DATA_LOGGINGHandle);
			  switch(WHOM_TO_GIVE_DATA)
			  {
			  case MODEM_SYSTEM:
				  loramsg = osPoolAlloc(lorasendpool);
				  memcpy(loramsg->LORA_SEND_DATA,DATA_OF_BIST,BUFFER_SIZE_ALL);
				  osMessagePut(LORA_SEND_qHandle, (uint32_t)loramsg,100);
				  break;
			  case LOCAL_SYSTEM:
				  HAL_UART_Transmit(&huart6, DATA_OF_BIST, 200,100);
				  break;
			  }
			  break;
			  osThreadResume(DATA_LOGGINGHandle);
		  }
	  }
    osDelay(1);
  }
  /* USER CODE END Start_FLASH_READ */
}

/* USER CODE BEGIN Header_Start_WIFI_SEND */
/**
* @brief Function implementing the WIFI_SEND thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_WIFI_SEND */
void Start_WIFI_SEND(void const * argument)
{
  /* USER CODE BEGIN Start_WIFI_SEND */
	WIFI_TEMP	*wifitempmsg;
	WIFI_CURR	*wificurrmsg;
	GIVE_ME	*givememsg;
	FLASH_LOGGING *logmsg;

	osEvent tempevt,currevt;

	uint8_t DATA_FOR_WIFI[50];
  /* Infinite loop */
  for(;;)
  {
	if((I_AM == I_AM_MASTER))
	{
	givememsg = osPoolAlloc(givepool);
	givememsg->GIVE_ME_t = I_AM_WIFI;
	osMessagePut(GIVE_ME_qHandle, (uint32_t)givememsg,100);
	}

	tempevt = osMailGet(wifitempmail, 0);
	if(tempevt.status == osEventMail)
	{
	wifitempmsg = tempevt.value.p;
	memcpy(DATA_FOR_WIFI,wifitempmsg->TEMP_DATA_FOR_WIFI,50);
	osMailFree(wifitempmail, wifitempmsg);
	HAL_UART_Transmit(&huart4, DATA_FOR_WIFI, 50,1000);
	logmsg = osPoolAlloc(flashlogpool);
	memcpy(logmsg->FLASH_DATA_LOGG,DATA_FOR_WIFI,50);
	osMessagePut(FLASH_LOGGINGHandle, (uint32_t)logmsg,1000);
	}

	osDelay(300);

	currevt = osMailGet(wificurrmail, 0);
	if(currevt.status == osEventMail)
	{
	wificurrmsg = currevt.value.p;
	memcpy(DATA_FOR_WIFI,wificurrmsg->CURR_DATA_FOR_WIFI,50);
	osMailFree(wificurrmail, wificurrmsg);
	HAL_UART_Transmit(&huart4, DATA_FOR_WIFI, 50,1000);
	logmsg = osPoolAlloc(flashlogpool);
	memcpy(logmsg->FLASH_DATA_LOGG,DATA_FOR_WIFI,50);
	osMessagePut(FLASH_LOGGINGHandle, (uint32_t)logmsg,1000);
	}
  }
  /* USER CODE END Start_WIFI_SEND */
}

/* USER CODE BEGIN Header_Start_PROFILE_START */
/**
* @brief Function implementing the PROFILE_START thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_Start_PROFILE_START */
void Start_PROFILE_START(void const * argument)
{
  /* USER CODE BEGIN Start_PROFILE_START */
	RTC_TimeTypeDef gTime;
	RTC_DateTypeDef gDate;

	SETPOINT *sp_msg;

	uint8_t DATE;
	uint8_t MONTH;
	uint8_t YEAR;
	uint8_t HOUR;
	uint8_t MINUTE;
	uint8_t SECOND;
	uint16_t SETPOINT_t[16];
	static int z =0;
	static int j =4;
	static int COUNT=0;
	static uint8_t CURRENT_PROFILE[50]={0};
  /* Infinite loop */
  for(;;)
  {
		HAL_RTC_GetTime(&hrtc, &gTime, RTC_FORMAT_BIN);
		HAL_RTC_GetDate(&hrtc, &gDate, RTC_FORMAT_BIN);
		DATE = gDate.Date;
		MONTH = gDate.Month;
		YEAR = gDate.Year;
		HOUR = gTime.Hours;
		MINUTE = gTime.Minutes;
		SECOND = gTime.Seconds;

		if(YEAR>=CURRENT_PROFILE[42])
		{
			if(MONTH>=CURRENT_PROFILE[41])
			{
				if(DATE>=CURRENT_PROFILE[40])
				{
					if(HOUR>=CURRENT_PROFILE[43])
					{
						if(MINUTE>=CURRENT_PROFILE[44])
						{
							if(SECOND>=CURRENT_PROFILE[45])
							{
						    if(COUNT <=PROFILE_COUNT)
						    {
							j=4;
							for(int i =0;i<16;i++)
							{
								SETPOINT_t[i] = (((((uint16_t)CURRENT_PROFILE[j])<<8)&0xFF00)|((((uint16_t)CURRENT_PROFILE[j+1]))&0x00FF));
								j =j+2;
								if(j ==36)
								{
								j =4;
								}
							}
							sp_msg = osPoolAlloc(sppool);
							for(int i =0;i<16;i++)
							{
							sp_msg->SETPOINT_t[i] = SETPOINT_t[i];
							}
							osMessagePut(SETPOINT_qHandle, (uint32_t)sp_msg,100);
							for(int k =0;k<50;k++)
							{
							CURRENT_PROFILE[k] = PROFILES[z][k];
							}
							z=z+1;
							COUNT = COUNT+1;
							if(z==50)
							{
								z=0;
							}
							}
							}
						}
					}
				}
			}
		}
    osDelay(500);
  }
  /* USER CODE END Start_PROFILE_START */
}

/* TCU_SDA_TIMER_CALL function */
void TCU_SDA_TIMER_CALL(void const * argument)
{
  /* USER CODE BEGIN TCU_SDA_TIMER_CALL */
	NO_DUTY = 1;
	HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin,SET);
	SDA_ERROR_DATA = 1;
  /* USER CODE END TCU_SDA_TIMER_CALL */
}

/* Callback02 function */
void Callback02(void const * argument)
{
  /* USER CODE BEGIN Callback02 */

  /* USER CODE END Callback02 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7) {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
