/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2020 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "MY_DHT11.h"
#include "STM_MY_LCD204A.h"
#include "dwt_stm32_delay.h"


/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#define PERIOD 1000U	
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
//  GPIO_InitTypeDef GPIO_InitStruct = {0};
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

CAN_HandleTypeDef hcan1;
CAN_FilterTypeDef sfilterConfig;   //can filter configuration variable
CAN_TxHeaderTypeDef TxHeader;			//can Tx configuration variable
CAN_RxHeaderTypeDef RxHeader;			//can Tx configuration variable
uint32_t pTxMailbox;					//Tx mailbox
void CAN_Init(void); 

UART_HandleTypeDef huart5;

osThreadId myCanTaskHandle;
osThreadId myFlameTaskHandle;
osThreadId myDHT11TaskHandle;
osThreadId myMQ135TaskHandle;
osThreadId myEmergencyTaskHandle;
osMutexId myLcdMutexHandle;
osMutexId myUserMutexHandle;
/* USER CODE BEGIN PV */



/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_UART5_Init(void);

void StartCanTask(void const * argument);
void StartFlameTask(void const * argument);
void StartDHT11Task(void const * argument);
void StartMQ135Task(void const * argument);
void StartEmergencyTask(void const * argument);

/* USER CODE BEGIN PFP */
	uint8_t temp=0,humid=0;
	uint8_t flame = 0;
	uint8_t can_Data[4];
	uint8_t airQuality;
	uint8_t value1;
	uint8_t smoke = 0;
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

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
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_UART5_Init();
  /* USER CODE BEGIN 2 */

	HAL_ADC_Start(&hadc1);
	
	CAN_Init();
	
	DWT_Delay_Init();
	
	
	LCD204A_Begin4BIT(RS_GPIO_Port,RS_Pin,EN_Pin,D4_GPIO_Port,D4_Pin,D5_Pin,D6_Pin,D7_Pin);
	LCD204A_noBlink();
	LCD204A_noCursor();
	LCD204A_print(" Welcome To Factory");
	HAL_Delay(1500);
	LCD204A_clear();
	LCD204A_1stLine();
	LCD204A_print("Configuring...");
	HAL_Delay(3000);
	LCD204A_clear();

  /* USER CODE END 2 */

  /* Create the mutex(es) */
  /* definition and creation of myLcdMutex */
  osMutexDef(myLcdMutex);
  myLcdMutexHandle = osMutexCreate(osMutex(myLcdMutex));

  /* definition and creation of myUserMutex */
  osMutexDef(myUserMutex);
  myUserMutexHandle = osMutexCreate(osMutex(myUserMutex));

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of myCanTask */
  osThreadDef(myCanTask, StartCanTask, osPriorityNormal, 0, 128);
  myCanTaskHandle = osThreadCreate(osThread(myCanTask), NULL);

  /* definition and creation of myFlameTask */
  osThreadDef(myFlameTask, StartFlameTask, osPriorityLow, 0, 128);
  myFlameTaskHandle = osThreadCreate(osThread(myFlameTask), NULL);

  /* definition and creation of myDHT11Task */ 
  osThreadDef(myDHT11Task, StartDHT11Task, osPriorityLow, 0, 128);
  myDHT11TaskHandle = osThreadCreate(osThread(myDHT11Task), NULL);

  /* definition and creation of myMQ135Task */
  osThreadDef(myMQ135Task, StartMQ135Task, osPriorityLow, 0, 128);
  myMQ135TaskHandle = osThreadCreate(osThread(myMQ135Task), NULL);

  /* definition and creation of myEmergencyTask */
  osThreadDef(myEmergencyTask, StartEmergencyTask, osPriorityLow, 0, 128);
  myEmergencyTaskHandle = osThreadCreate(osThread(myEmergencyTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
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

  /** Configure the main internal regulator output voltage 
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 7;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }
  /** Initializes the CPU, AHB and APB busses clocks 
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
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
  hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV8;
  hadc1.Init.Resolution = ADC_RESOLUTION_8B;
  hadc1.Init.ScanConvMode = ENABLE;
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
  sConfig.Channel = ADC_CHANNEL_0;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief CAN1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_CAN1_Init(void)
{

  /* USER CODE BEGIN CAN1_Init 0 */

  /* USER CODE END CAN1_Init 0 */

  /* USER CODE BEGIN CAN1_Init 1 */

  /* USER CODE END CAN1_Init 1 */
  hcan1.Instance = CAN1;
  hcan1.Init.Prescaler = 21;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_13TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
  hcan1.Init.TimeTriggeredMode = DISABLE;
  hcan1.Init.AutoBusOff = DISABLE;
  hcan1.Init.AutoWakeUp = DISABLE;
  hcan1.Init.AutoRetransmission = DISABLE;
  hcan1.Init.ReceiveFifoLocked = DISABLE;
  hcan1.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN1_Init 2 */

  /* USER CODE END CAN1_Init 2 */

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
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA2_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA2_Stream0_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
 GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, D4_Pin|D5_Pin|D6_Pin|D7_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, DHT11_Pin|MOT1_Pin|MOT2_Pin|LED1_Pin 
                          |LED2_Pin|LED3_Pin|LED4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, EN_Pin|RS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : D4_Pin D5_Pin D6_Pin D7_Pin */
  GPIO_InitStruct.Pin = D4_Pin|D5_Pin|D6_Pin|D7_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pins : DHT11_Pin MOT1_Pin MOT2_Pin LED1_Pin 
                           LED2_Pin LED3_Pin LED4_Pin */
  GPIO_InitStruct.Pin = DHT11_Pin|MOT1_Pin|MOT2_Pin|LED1_Pin 
                          |LED2_Pin|LED3_Pin|LED4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : MQ_Digital_Pin */
  GPIO_InitStruct.Pin = MQ_Digital_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(MQ_Digital_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Flame_Pin */
  GPIO_InitStruct.Pin = Flame_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(Flame_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : EN_Pin RS_Pin */
  GPIO_InitStruct.Pin = EN_Pin|RS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartCanTask */
/**
  * @brief  Function implementing the myCanTask thread.
  * @param  argument: Not used 
  * @retval None
  */
/* USER CODE END Header_StartCanTask */
void StartCanTask(void const * argument)
{  
	uint8_t Data = 'C';
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
		/*
		if( !(flame) && smoke) {
		
		xSemaphoreTake(myLcdMutexHandle,portMAX_DELAY);
		
		LCD204A_4thLine();
		LCD204A_print("System OK");
			HAL_Delay(200);
		
		xSemaphoreGive(myLcdMutexHandle);
		}			
		*/
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,can_Data,&pTxMailbox);
		/*
		xSemaphoreTake(myUserMutexHandle,portMAX_DELAY);
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&Data,&pTxMailbox);
		xSemaphoreGive(myUserMutexHandle);
		*/
    osDelay(PERIOD);
		
  }
  /* USER CODE END 5 */ 
}

/* USER CODE BEGIN Header_StartFlameTask */
/**
* @brief Function implementing the myFlameTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartFlameTask */
void StartFlameTask(void const * argument)
{
	uint8_t Data = 'F';
	//uint8_t flame = 0;
  /* USER CODE BEGIN StartFlameTask */
  /* Infinite loop */
  for(;;)
  {
		
		//HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
		//osDelay(PERIOD);
		
		
		flame = HAL_GPIO_ReadPin(Flame_GPIO_Port,Flame_Pin);
		can_Data[0] = flame;
		
		if(flame){
	
				xTaskNotify(myEmergencyTaskHandle,0x01,eSetBits);
		
		} /*else {
		
		xSemaphoreTake(myLcdMutexHandle,portMAX_DELAY);
		
		LCD204A_4thLine();
		LCD204A_print("System OK");
		
		xSemaphoreGive(myLcdMutexHandle);
		}
			*/
		/*
		xSemaphoreTake(myUserMutexHandle,portMAX_DELAY);
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&Data,&pTxMailbox);
		xSemaphoreGive(myUserMutexHandle);		
    */
		osDelay(PERIOD);
		
		
  }
  /* USER CODE END StartFlameTask */
}

/* USER CODE BEGIN Header_StartDHT11Task */
/**
* @brief Function implementing the myDHT11Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDHT11Task */
void StartDHT11Task(void const * argument)
{

	//uint8_t Data = 'D';
  /* USER CODE BEGIN StartDHT11Task */
  /* Infinite loop */
  for(;;)
  {
		
		/*
		HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		osDelay(PERIOD);
		*/
		
		DHT11_GetTemp_Humidity(&temp,&humid);
		can_Data[1]= temp;
		can_Data[2]= humid;
			
		
		//osDelay(PERIOD);
		
							xSemaphoreTake(myLcdMutexHandle,portMAX_DELAY);
		
							LCD204A_1stLine();
							LCD204A_print("Temperature:");
							LCD204A_PrintInt(temp);
							LCD204A_print("C");
							LCD204A_2ndLine();
							LCD204A_print("Humidity:");
							LCD204A_PrintInt(humid);
							LCD204A_print("%");
						//	HAL_Delay(200);
		
							xSemaphoreGive(myLcdMutexHandle);
							
							
		if(temp>=40){
	       //temperature condition
				xTaskNotify(myEmergencyTaskHandle,0x03,eSetBits);
		} else if(humid>=70){
							//humidity condition	
							xTaskNotify(myEmergencyTaskHandle,0x04,eSetBits);
		} 
		
		/*
    xSemaphoreTake(myUserMutexHandle,portMAX_DELAY);
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&temp,&pTxMailbox);
		xSemaphoreGive(myUserMutexHandle);
    */
		osDelay(PERIOD);
		
  }
}

/* USER CODE BEGIN Header_StartMQ135Task */
/**
* @brief Function implementing the myMQ135Task thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartMQ135Task */
void StartMQ135Task(void const * argument)
{
	
	//uint8_t smokeThd;
	
	uint8_t Data = 'S';
	
  /* USER CODE BEGIN StartMQ135Task */
  /* Infinite loop */
  for(;;)
  {
		
		
		value1 = HAL_ADC_GetValue(&hadc1);
		smoke = HAL_GPIO_ReadPin(MQ_Digital_GPIO_Port,MQ_Digital_Pin);
		
		can_Data[3] = smoke;
		if(!smoke){
	
				xTaskNotify(myEmergencyTaskHandle,0x02,eSetBits);
			
		} /*else {

		
		xSemaphoreTake(myLcdMutexHandle,portMAX_DELAY);
		
		LCD204A_3rdLine();
		LCD204A_print("Air Qlt: OK");
		
		
		xSemaphoreGive(myUserMutexHandle);
		}*/
			//value1 = value1 * 100;
		//airQuality = (uint8_t)value1/255;
		
		
		//osDelay(PERIOD);
		
		/*
		smoke = HAL_ADC_GetValue(&hadc1);
		smokeThd = HAL_GPIO_ReadPin(MQ_Digital_GPIO_Port,MQ_Digital_Pin);
		*/
		
		/*
    xSemaphoreTake(myUserMutexHandle,portMAX_DELAY);
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&Data,&pTxMailbox);
		xSemaphoreGive(myUserMutexHandle);
		*/
    osDelay(PERIOD);
		
  }
  /* USER CODE END StartMQ135Task */
}
/* USER CODE BEGIN Header_StartEmergencyTask */
/**
* @brief Function implementing the myEmergencyTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartEmergencyTask */
void StartEmergencyTask(void const * argument)
{
	uint8_t Data = 'E';
	uint32_t notifyValue;
	
  /* USER CODE BEGIN StartEmergencyTask */
  /* Infinite loop */
  for(;;)
  {
		xTaskNotifyWait(pdFALSE,0xFF,&notifyValue,portMAX_DELAY);
		if((notifyValue & 0x01) != 0x00){
		//flame
		HAL_GPIO_TogglePin(LED1_GPIO_Port,LED1_Pin);
				
		xSemaphoreTake(myLcdMutexHandle,portMAX_DELAY);
		
	//	LCD204A_clear();
		LCD204A_3rdLine();
		LCD204A_print("  Fire in Factory");
		LCD204A_4thLine();
		LCD204A_print("    Evacuate Now");
		osDelay(1500);
		
		xSemaphoreGive(myUserMutexHandle);
			
		} else if((notifyValue & 0x02) != 0x00){
		
			//smoke
			HAL_GPIO_TogglePin(LED2_GPIO_Port,LED2_Pin);
		
			xSemaphoreTake(myLcdMutexHandle,portMAX_DELAY);
		
		//LCD204A_clear();
		LCD204A_3rdLine();
		LCD204A_print("  Smoke in Factory");
		LCD204A_4thLine();
		LCD204A_print("    Evacuate Now");
		osDelay(1500);
		
		xSemaphoreGive(myUserMutexHandle);
			
			
			
		} else if((notifyValue & 0x03) != 0x00){
		
			//temperature
			HAL_GPIO_TogglePin(LED3_GPIO_Port,LED3_Pin);
			
			xSemaphoreTake(myLcdMutexHandle,portMAX_DELAY);
		
		//LCD204A_clear();
		LCD204A_3rdLine();
		LCD204A_print("  High Temperature");
		LCD204A_4thLine();
		LCD204A_print("   Pause work Now");
		osDelay(1500);
		xSemaphoreGive(myUserMutexHandle);
			
			
		} else if((notifyValue & 0x04) != 0x00){
		
			//humidity
			HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
			
			xSemaphoreTake(myLcdMutexHandle,portMAX_DELAY);
		
		//LCD204A_clear();
		LCD204A_3rdLine();
		LCD204A_print("High Humidity");
		LCD204A_4thLine();
		LCD204A_print("Pause work Now");
		osDelay(1500);
		
		xSemaphoreGive(myUserMutexHandle);
			
		}
		
		
		LCD204A_clear();
		/*
		HAL_GPIO_TogglePin(LED4_GPIO_Port,LED4_Pin);
		osDelay(PERIOD);
		*/
		
		/*
   xSemaphoreTake(myUserMutexHandle,portMAX_DELAY);
		HAL_CAN_AddTxMessage(&hcan1,&TxHeader,&Data,&pTxMailbox);
		xSemaphoreGive(myUserMutexHandle);
    */
		osDelay(PERIOD);
		
		//osDelay(1);
  }
  /* USER CODE END StartEmergencyTask */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM3 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM3) {
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

  /* USER CODE END Error_Handler_Debug */
}

/********************************************************************************/
//CAN initialization function for tx and filter parameters 
/***********************************************************************************/
void CAN_Init(void)
{
	TxHeader.DLC=4;
	TxHeader.IDE=CAN_ID_STD;
	TxHeader.RTR=CAN_RTR_DATA;
	TxHeader.StdId=0x244;
	
	
	sfilterConfig.FilterBank=0;
	sfilterConfig.FilterIdHigh=0;
	//sfilterConfig.FilterIdHigh=0x245<<5;
	sfilterConfig.FilterIdLow=0;
	sfilterConfig.FilterMaskIdHigh=0;
	sfilterConfig.FilterMaskIdLow=0;
	sfilterConfig.FilterScale=CAN_FILTERSCALE_32BIT;
	sfilterConfig.FilterFIFOAssignment=CAN_FilterFIFO0;
	sfilterConfig.FilterActivation=ENABLE;
	
	HAL_CAN_ConfigFilter(&hcan1,&sfilterConfig);
	
	HAL_CAN_Start(&hcan1);
	
	HAL_CAN_ActivateNotification(&hcan1,CAN_IT_RX_FIFO0_MSG_PENDING);   //Enable the inteerupt of can receive
	
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
     tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
