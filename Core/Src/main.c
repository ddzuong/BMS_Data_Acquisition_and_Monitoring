/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef enum{
	total_voltage,
	current,
	balance_capacity,
	rate_capacity,
	cycle,
	production_date,
	balance_status,
	balance_status_High,
	protection_status,
	software_version,
	RSOC,
	FET_control_status,
	battery_series,
	ntc_num,
	ntc_sensor1,
	ntc_sensor2,
	
	Buffer_cnt
}Data_Type;

//typedef struct{
//	uint32_t ID;
//	uint8_t IDE;
//	uint8_t RTR;
//	uint8_t DLC;
//	uint8_t data[8];
//}TX_Mailbox;

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define Num_byte_Req_frame	7
#define Num_byte_RxFrame		34
#define CAN_TX_Buffer				34
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
CAN_HandleTypeDef hcan;

UART_HandleTypeDef huart1;

/* USER CODE BEGIN PV */
CAN_TxHeaderTypeDef tx_header1;
CAN_TxHeaderTypeDef tx_header2;
CAN_TxHeaderTypeDef tx_header3;
CAN_TxHeaderTypeDef tx_header4;
//-------UART-----
uint8_t request_frame_03[Num_byte_Req_frame] = {0xDD, 0xA5, 0x03, 0x00, 0xFF, 0xFD, 0x77};
uint8_t request_frame_04[Num_byte_Req_frame] = {0xDD, 0xA5, 0x04, 0x00, 0xFF, 0xFC, 0x77};
uint8_t Rx_buffer[Num_byte_RxFrame] = {};

static float data_buffer[Buffer_cnt];

//Cmd: 0x03 -----Data content interpretation-----
volatile uint8_t header;
volatile uint8_t command;
volatile uint8_t status;
volatile uint8_t data_len;
volatile float raw_total_voltage;
volatile float raw_current;
volatile float raw_balance_capa;
volatile float raw_rate_capa;
volatile float raw_cycle;
volatile float raw_prd_date;
volatile float raw_balance_status;
volatile float raw_balance_status_h;
volatile float raw_protection_status;
volatile float raw_sw_version;
volatile float raw_rsoc;
volatile float raw_fet_ctrl_status;
volatile float raw_batt_series;
volatile float raw_ntc_num;
volatile float raw_ntc_sensor1;
volatile float raw_ntc_sensor2;

//Debug data frame
volatile float d_total_voltage = 0.0f;
volatile float d_current = 0.00f;
volatile float d_balance_capa = 0.0f;
volatile float d_rate_capa = 0.0f;
volatile float d_cycle = 0.0f;
volatile float d_prd_date = 0.0f;
volatile float d_balance_status = 0.0f;
volatile float d_balance_status_h = 0.0f;
volatile float d_protection_status = 0.0f;
volatile float d_sw_version = 0.0f;
volatile float d_rsoc = 0.0f;
volatile float d_fet_ctrl_status = 0.0f;
volatile float d_batt_series = 0.0f;
volatile float d_ntc_num = 0.0f;
volatile float d_ntc_sensor1 = 0.0f;
volatile float d_ntc_sensor2 = 0.0f;

volatile uint8_t date = 0;
volatile uint8_t month = 0;
volatile uint16_t year = 0;
volatile float temp_sensor1 = 0.0f;
volatile float temp_sensor2 = 0.0f;

volatile HAL_StatusTypeDef can_tx_status;
volatile uint32_t can_tx_pending; //if pending = 1 mean frame does not move on mailbox, else mean frame moved on mailbox
volatile uint32_t can_free_mailbox; //number of free mailbox

//-----CAN-----
uint8_t tx_data1[8];
uint8_t tx_data2[8];
uint8_t tx_data3[8];
uint8_t tx_data4[3];
uint32_t tx_mailbox;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_CAN_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	//Doing sth
	HAL_UART_Receive_IT(&huart1, Rx_buffer, sizeof(Rx_buffer));
}

void UART_Raw_frame(){
	header = Rx_buffer[0];
	command = Rx_buffer[1];
	status = Rx_buffer[2];
	data_len = Rx_buffer[3];
	raw_total_voltage = (float)((Rx_buffer[4] << 8) | (Rx_buffer[5]));
	raw_current = (float)(int16_t)((Rx_buffer[6] << 8) | (Rx_buffer[7]));
	raw_balance_capa = (float)((Rx_buffer[8] << 8) | (Rx_buffer[9]));
	raw_rate_capa = (float)((Rx_buffer[10] << 8) | (Rx_buffer[11]));
	raw_cycle = (float)((Rx_buffer[12] << 8) | (Rx_buffer[13]));
	raw_prd_date = (float)((Rx_buffer[14] << 8) | (Rx_buffer[15]));
	raw_balance_status = (float)((Rx_buffer[16] << 8)| (Rx_buffer[17]));
	raw_balance_status_h = (float)((Rx_buffer[18] << 8) | (Rx_buffer[19]));
	raw_protection_status = (float)((Rx_buffer[20] << 8) | (Rx_buffer[21]));
	raw_sw_version = (float)(Rx_buffer[22]);
	raw_rsoc = (float)(Rx_buffer[23]);
	raw_fet_ctrl_status = (float)(Rx_buffer[24]);
	raw_batt_series = (float)(Rx_buffer[25]);
	raw_ntc_num	= (float)(Rx_buffer[26]);
	raw_ntc_sensor1 = (float)(((Rx_buffer[27]) << 8) | ((Rx_buffer[28])));
	raw_ntc_sensor2 = (float)(((Rx_buffer[29]) << 8) | ((Rx_buffer[30])));
}
void UART_Process_Frame(){
	//Decode raw data
	UART_Raw_frame();
	if(Rx_buffer[2] == 0x80){
		//Error frame
		return;
	}
	else if(Rx_buffer[2] == 0x00){
		if((Rx_buffer[0] == 0xDD) && (Rx_buffer[1] == 0x03)){
			data_buffer[total_voltage] = (float)raw_total_voltage/100.00f;
			d_total_voltage = (float)data_buffer[total_voltage]; //Debug
			
			data_buffer[current] = raw_current/100.00f;
			d_current = (float)data_buffer[current];
			
			data_buffer[balance_capacity] = raw_balance_capa/100;
			d_balance_capa = (float)data_buffer[balance_capacity];
			
			data_buffer[rate_capacity] = raw_rate_capa/100;
			d_rate_capa = (float)data_buffer[rate_capacity];
			
			data_buffer[cycle] = raw_cycle;
			d_cycle = (float)data_buffer[cycle];
			
			data_buffer[production_date] = raw_prd_date;
			d_prd_date = (float)data_buffer[production_date];
			
			data_buffer[balance_status] = raw_balance_status;
			d_balance_status = (float)data_buffer[balance_status];
			
			data_buffer[balance_status_High] = raw_balance_status_h;
			d_balance_status_h = (float)data_buffer[balance_status_High];
			
			data_buffer[protection_status] = raw_protection_status;
			d_protection_status = (float)data_buffer[protection_status];
			
			data_buffer[software_version] = raw_sw_version;
			d_sw_version = (float)data_buffer[software_version];
			
			data_buffer[RSOC] = raw_rsoc;
			d_rsoc = (float)data_buffer[RSOC];
			
			data_buffer[FET_control_status] = raw_fet_ctrl_status;
			d_fet_ctrl_status = (float)data_buffer[FET_control_status];
			
			data_buffer[battery_series] = raw_batt_series;
			d_batt_series = (float)data_buffer[battery_series];
			
			data_buffer[ntc_num] = raw_ntc_num;
			d_ntc_num = (float)data_buffer[ntc_num];
			
			data_buffer[ntc_sensor1] = raw_ntc_sensor1;
			d_ntc_sensor1 = (float)data_buffer[ntc_sensor1];
			
			data_buffer[ntc_sensor2] = raw_ntc_sensor2;
			d_ntc_sensor2 = (float)data_buffer[ntc_sensor2];
			
			date = (uint16_t)data_buffer[production_date] & 0x1F;
			month = (uint16_t)data_buffer[production_date] >> 5 & 0xF;
			year = 2000 + ((uint16_t)data_buffer[production_date] >> 9);
			temp_sensor1 = (data_buffer[ntc_sensor1] - 2731)/10.0f;
			temp_sensor2 = (data_buffer[ntc_sensor2] - 2731)/10.0f;
		}
	}
}

void CAN_Process(){
	
	//Encode frame 
	
	tx_header1.StdId = 0x100;
	tx_header1.IDE = CAN_ID_STD;
	tx_header1.RTR = CAN_RTR_DATA;
	tx_header1.DLC = 8;
	tx_header1.TransmitGlobalTime = DISABLE;
	
	tx_data1[0] = ((uint16_t)raw_total_voltage >> 8) & 0xFF;
	tx_data1[1] = (uint8_t)raw_total_voltage & 0xFF;
	tx_data1[2] = ((uint16_t)raw_current >> 8) & 0xFF;
	tx_data1[3] = (uint8_t)raw_current & 0xFF;
	tx_data1[4] = ((uint16_t)raw_balance_capa >> 8) & 0xFF;
	tx_data1[5] = (uint8_t)raw_balance_capa & 0xFF;
	tx_data1[6] = ((uint16_t)raw_rate_capa >> 8) & 0xFF;
	tx_data1[7] = (uint8_t)raw_rate_capa & 0xFF;
	
	
	tx_header2.StdId = 0x101;
	tx_header2.ExtId = DISABLE;
	tx_header2.IDE = CAN_ID_STD;
	tx_header2.RTR = CAN_RTR_DATA;
	tx_header2.DLC = 8;
	tx_header2.TransmitGlobalTime = DISABLE;
		
	tx_data2[0] = ((uint16_t)raw_cycle >> 8) & 0xFF;
	tx_data2[1] = ((uint8_t)raw_cycle) & 0xFF;
	tx_data2[2] = ((uint16_t)raw_prd_date >> 8) &0xFF;
	tx_data2[3] = (uint8_t)raw_prd_date & 0xFF;
	tx_data2[4] = ((uint16_t)raw_balance_status >> 8) & 0xFF;
	tx_data2[5] = (uint8_t)raw_balance_status & 0xFF;
	tx_data2[6] = ((uint16_t)raw_balance_status_h >> 8) & 0xFF;
	tx_data2[7] = (uint8_t)raw_balance_status_h & 0xFF;
	
	
	tx_header3.StdId = 0x102;
	tx_header3.ExtId = DISABLE;
	tx_header3.IDE = CAN_ID_STD;
	tx_header3.RTR = CAN_RTR_DATA;
	tx_header3.DLC = 8;
	tx_header3.TransmitGlobalTime = DISABLE;
		
	tx_data3[0] = ((uint16_t)raw_protection_status >> 8) & 0xFF;
	tx_data3[1] = (uint8_t)raw_protection_status & 0xFF;
	tx_data3[2] = (uint8_t)raw_sw_version & 0xFF;
	tx_data3[3] = (uint8_t)raw_rsoc & 0xFF;
	tx_data3[4] = (uint8_t)raw_fet_ctrl_status & 0xFF;
	tx_data3[5] = (uint8_t)raw_batt_series & 0xFF;
	tx_data3[6] = (uint8_t)raw_ntc_num & 0xFF;
	tx_data3[7] = ((uint16_t)raw_ntc_sensor1 >> 8) & 0xFF;
	
	
	tx_header4.StdId = 0x104;
	tx_header4.ExtId = DISABLE;
	tx_header4.IDE = CAN_ID_STD;
	tx_header4.RTR = CAN_RTR_DATA;
	tx_header4.DLC = 3;
	tx_header4.TransmitGlobalTime = DISABLE;
		
	tx_data4[0] = (uint8_t)raw_ntc_sensor1 & 0xFF;
	tx_data4[1] = ((uint16_t)raw_ntc_sensor2) & 0xFF;
	tx_data4[2] = (uint8_t)raw_ntc_sensor2 & 0xFF;
}

void CAN_Transmit(){
	
	if(HAL_CAN_GetTxMailboxesFreeLevel(&hcan) > 0){
		HAL_CAN_AddTxMessage(&hcan, &tx_header1, tx_data1, &tx_mailbox);
		HAL_CAN_AddTxMessage(&hcan, &tx_header2, tx_data2, &tx_mailbox);
		HAL_CAN_AddTxMessage(&hcan, &tx_header3, tx_data3, &tx_mailbox);
		
		if(HAL_CAN_GetTxMailboxesFreeLevel > 0){
			HAL_Delay(1);//Wait to until appear empty mailbox
			HAL_CAN_AddTxMessage(&hcan, &tx_header4, tx_data4, &tx_mailbox);
		}
		//HAL_CAN_AddTxMessage(&hcan, &tx_header4, tx_data4, &tx_mailbox);
		//can_tx_status = HAL_CAN_AddTxMessage(&hcan, &tx_header1, tx_data1, &tx_mailbox); //Debug
//		if(can_tx_status == HAL_OK){
//			HAL_Delay(1);
//			can_tx_pending = HAL_CAN_IsTxMessagePending(&hcan, tx_mailbox);
//			
//			can_free_mailbox = HAL_CAN_GetTxMailboxesFreeLevel(&hcan);
//		}
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

  /* USER CODE END 1 */

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
  MX_USART1_UART_Init();
  MX_CAN_Init();
  /* USER CODE BEGIN 2 */
	HAL_UART_Receive_IT(&huart1, Rx_buffer, sizeof(Rx_buffer)); 
	HAL_CAN_Start(&hcan);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
		for(uint8_t i = 0; i < Num_byte_Req_frame; i++){
			HAL_UART_Transmit(&huart1, request_frame_03, sizeof(request_frame_03), 50);
			
		}
		
		UART_Process_Frame();
		
		CAN_Process();
		CAN_Transmit();

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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief CAN Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 4;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_15TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = ENABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = ENABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

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
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

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
#ifdef USE_FULL_ASSERT
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
