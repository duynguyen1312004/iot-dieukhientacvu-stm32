/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "manual_lcd.h"
#include "manual_touch.h"
#include <stdio.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f4xx_hal.h"
#include "task2.h"
#include "sd_card_manager.h"
#include "rtc_utils.h"
#include "task.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;

CAN_HandleTypeDef hcan1;
CAN_HandleTypeDef hcan2;

I2C_HandleTypeDef hi2c2;

RTC_HandleTypeDef hrtc;

SD_HandleTypeDef hsd;

SPI_HandleTypeDef hspi1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
    .name = "defaultTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for canCommTask */
osThreadId_t canCommTaskHandle;
const osThreadAttr_t canCommTask_attributes = {
    .name = "canCommTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};
/* Definitions for tempFramTask */
osThreadId_t tempFramTaskHandle;
const osThreadAttr_t tempFramTask_attributes = {
    .name = "tempFramTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for sdCardTask */
osThreadId_t sdCardTaskHandle;
const osThreadAttr_t sdCardTask_attributes = {
    .name = "sdCardTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for ledBlinkTask */
osThreadId_t ledBlinkTaskHandle;
const osThreadAttr_t ledBlinkTask_attributes = {
    .name = "ledBlinkTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityLow,
};

/* Definitions for lcdUpdateTextTa */
osThreadId_t lcdUpdateTextTaHandle;
const osThreadAttr_t lcdUpdateTextTa_attributes = {
    .name = "lcdUpdateTextTa",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityBelowNormal,
};
/* Definitions for uiManagerTask */
osThreadId_t uiManagerTaskHandle;
const osThreadAttr_t uiManagerTask_attributes = {
    .name = "uiManagerTask",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityNormal,
};
/* Definitions for taskSwitchHandler */
osThreadId_t taskSwitchHandlerHandle;
const osThreadAttr_t taskSwitchHandler_attributes = {
    .name = "taskSwitchHandler",
    .stack_size = 128 * 4,
    .priority = (osPriority_t)osPriorityHigh,
};

/* USER CODE BEGIN PV */
uint8_t read_data[50], byte_read;
SD_HandleTypeDef hsd;
HAL_SD_CardInfoTypeDef SDCardInfo;

CAN_TxHeaderTypeDef TxHeader1;
CAN_RxHeaderTypeDef RxHeader2;

uint8_t TxData1[2];
uint8_t RxData2[2];

uint32_t TxMailbox1;

volatile uint8_t can2_received_temperature;
volatile uint8_t can2_received_group;
volatile uint8_t can2_received_data_flag = 0;
int current_task = -1;
GPIO_PinState current_button_state = GPIO_PIN_RESET;

TaskBox_t task_boxes[4];
TaskBox_t back_button_box;
int num_tasks;
char info_text_buffer[50];

osSemaphoreId_t uiUpdateSemaphore;
osSemaphoreId_t lcdUpdateSemaphore;
osMessageQueueId_t lcdTextQueue;
osMessageQueueId_t taskSwitchQueue;
osMessageQueueId_t uiTaskSwitchQueue;
osMutexId_t lcdMutex;
osMutexId_t lcdTextMutex;

char globalLcdText[50];
volatile uint8_t lcdTextReady = 0;

osThreadId_t currentActiveTask = NULL;

typedef struct
{
  char text[50];
} LcdTextMessage_t;

typedef struct
{
  char oldTaskName[20];
  char newTaskName[20];
  uint16_t oldColor;
  uint16_t newColor;
  int hasOldTask;
} UiTaskSwitchMessage_t;

typedef struct
{
  osThreadId_t taskHandle;
  int taskIndex;
} TaskSwitchMessage_t;

volatile uint8_t is_members_layout = 1;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SDIO_SD_Init(void);
static void MX_ADC1_Init(void);
static void MX_CAN1_Init(void);
static void MX_CAN2_Init(void);
static void MX_RTC_Init(void);
void StartDefaultTask(void *argument);
void StartCanCommTask(void *argument);
void StartTempFramTask(void *argument);
void StartSDCardTask(void *argument);
void StartLedBlinkTask(void *argument);
void StartLcdUpdateTextTask(void *argument);
void StartUIManagerTask(void *argument);
void TaskSwitchHandler(void *argument);

/* USER CODE BEGIN PFP */

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
  uint16_t start_y_tasks_row1 = INFO_BOX_Y + INFO_BOX_H + INFO_TO_TASK_SPACING;
  uint16_t start_y_tasks_row2 = start_y_tasks_row1 + TASK_BOX_H + SPACING_BETWEEN_BOXES;

  task_boxes[0] = (TaskBox_t){25, 80, 90, 60, "Task 02-1"};
  task_boxes[1] = (TaskBox_t){125, 80, 90, 60, "Task 02-2"};
  task_boxes[2] = (TaskBox_t){25, 150, 90, 60, "Task 02-3"};
  task_boxes[3] = (TaskBox_t){125, 150, 90, 60, "Task 02-4"};

  num_tasks = 4;

  char current_task_text[20];

  back_button_box = (TaskBox_t){
      125,
      230,
      75,
      40,
      "Back"};

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
  MX_SPI1_Init();
  MX_I2C2_Init();
  MX_SDIO_SD_Init();
  MX_FATFS_Init();
  MX_ADC1_Init();
  MX_CAN1_Init();
  MX_CAN2_Init();
  MX_RTC_Init(); /* USER CODE BEGIN 2 */
  CAN_FilterTypeDef sFilterConfig2;
  sFilterConfig2.FilterBank = 14;
  sFilterConfig2.FilterMode = CAN_FILTERMODE_IDMASK;
  sFilterConfig2.FilterScale = CAN_FILTERSCALE_32BIT;
  sFilterConfig2.FilterIdHigh = 0x0000;
  sFilterConfig2.FilterIdLow = 0x0000;
  sFilterConfig2.FilterMaskIdHigh = 0x0000;
  sFilterConfig2.FilterMaskIdLow = 0x0000;
  sFilterConfig2.FilterFIFOAssignment = CAN_RX_FIFO0;
  sFilterConfig2.FilterActivation = ENABLE;

  if (HAL_CAN_ConfigFilter(&hcan2, &sFilterConfig2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan1) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_Start(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }

  if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    Error_Handler();
  }

  TxHeader1.StdId = 0x124;
  TxHeader1.ExtId = 0x00;
  TxHeader1.RTR = CAN_RTR_DATA;
  TxHeader1.IDE = CAN_ID_STD;
  TxHeader1.DLC = 2;
  TxHeader1.TransmitGlobalTime = DISABLE;

  // SD_Init();

  My_RTC_InitAndSet(&hrtc);
  Manual_LCD_Init();
  Manual_LCD_DrawMembersLayout();
  
  // Infinite loop waiting for PA1 button press
  while(1)
  {
    // Check PA1 button state (GPIO_PIN_RESET when pressed)
    if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
    {
     
      HAL_Delay(10);
      // Check again to ensure button is really pressed
      if(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
      {
        // Wait until button is released
        while(HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1) == GPIO_PIN_RESET)
        {
          HAL_Delay(10);
        }
        
        // Exit the waiting loop
        break;
      }
    }
    
    // Small delay to prevent CPU from running too fast
    HAL_Delay(10);
  }
  

  Manual_LCD_DrawLayout();

  Manual_Touch_Init(&hspi1);

  /* USER CODE END 4 */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  lcdMutex = osMutexNew(NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  uiUpdateSemaphore = osSemaphoreNew(1, 0, NULL);
  lcdUpdateSemaphore = osSemaphoreNew(1, 0, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */ /* USER CODE BEGIN RTOS_QUEUES */
  lcdTextQueue = osMessageQueueNew(1, sizeof(LcdTextMessage_t), NULL);
  taskSwitchQueue = osMessageQueueNew(1, sizeof(TaskSwitchMessage_t), NULL);
  uiTaskSwitchQueue = osMessageQueueNew(1, sizeof(UiTaskSwitchMessage_t), NULL);
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* creation of canCommTask */
  canCommTaskHandle = osThreadNew(StartCanCommTask, NULL, &canCommTask_attributes);

  /* creation of tempFramTask */
  tempFramTaskHandle = osThreadNew(StartTempFramTask, NULL, &tempFramTask_attributes);

  /* creation of sdCardTask */
  sdCardTaskHandle = osThreadNew(StartSDCardTask, NULL, &sdCardTask_attributes);
  /* creation of ledBlinkTask */
  ledBlinkTaskHandle = osThreadNew(StartLedBlinkTask, NULL, &ledBlinkTask_attributes);

  /* creation of lcdUpdateTextTa */
  lcdUpdateTextTaHandle = osThreadNew(StartLcdUpdateTextTask, NULL, &lcdUpdateTextTa_attributes);
  /* creation of uiManagerTask */
  uiManagerTaskHandle = osThreadNew(StartUIManagerTask, NULL, &uiManagerTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* creation of taskSwitchHandler */ taskSwitchHandlerHandle = osThreadNew(TaskSwitchHandler, NULL, &taskSwitchHandler_attributes);

  osThreadSuspend(ledBlinkTaskHandle);
  osThreadSuspend(canCommTaskHandle);
  osThreadSuspend(tempFramTaskHandle);
  osThreadSuspend(sdCardTaskHandle);
  osThreadSuspend(defaultTaskHandle);
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */ while (1)
  {
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

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_LSI | RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 25;
  RCC_OscInitStruct.PLL.PLLN = 144;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
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
  hadc1.Init.ScanConvMode = DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
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
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
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
  hcan1.Init.Prescaler = 72;
  hcan1.Init.Mode = CAN_MODE_NORMAL;
  hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan1.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan1.Init.TimeSeg2 = CAN_BS2_1TQ;
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
 * @brief CAN2 Initialization Function
 * @param None
 * @retval None
 */
static void MX_CAN2_Init(void)
{

  /* USER CODE BEGIN CAN2_Init 0 */

  /* USER CODE END CAN2_Init 0 */

  /* USER CODE BEGIN CAN2_Init 1 */

  /* USER CODE END CAN2_Init 1 */
  hcan2.Instance = CAN2;
  hcan2.Init.Prescaler = 72;
  hcan2.Init.Mode = CAN_MODE_NORMAL;
  hcan2.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan2.Init.TimeSeg1 = CAN_BS1_2TQ;
  hcan2.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan2.Init.TimeTriggeredMode = DISABLE;
  hcan2.Init.AutoBusOff = DISABLE;
  hcan2.Init.AutoWakeUp = DISABLE;
  hcan2.Init.AutoRetransmission = DISABLE;
  hcan2.Init.ReceiveFifoLocked = DISABLE;
  hcan2.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN2_Init 2 */
  HAL_NVIC_SetPriority(CAN2_RX0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(CAN2_RX0_IRQn);
  /* USER CODE END CAN2_Init 2 */
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
  hi2c2.Init.ClockSpeed = 100000;
  hi2c2.Init.DutyCycle = I2C_DUTYCYCLE_2;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */
  /* USER CODE END I2C2_Init 2 */
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
 * @brief SDIO Initialization Function
 * @param None
 * @retval None
 */
static void MX_SDIO_SD_Init(void)
{

  /* USER CODE BEGIN SDIO_Init 0 */

  /* USER CODE END SDIO_Init 0 */

  /* USER CODE BEGIN SDIO_Init 1 */

  /* USER CODE END SDIO_Init 1 */
  hsd.Instance = SDIO;
  hsd.Init.ClockEdge = SDIO_CLOCK_EDGE_RISING;
  hsd.Init.ClockBypass = SDIO_CLOCK_BYPASS_DISABLE;
  hsd.Init.ClockPowerSave = SDIO_CLOCK_POWER_SAVE_DISABLE;
  hsd.Init.BusWide = SDIO_BUS_WIDE_1B;
  hsd.Init.HardwareFlowControl = SDIO_HARDWARE_FLOW_CONTROL_DISABLE;
  hsd.Init.ClockDiv = 0;
  /* USER CODE BEGIN SDIO_Init 2 */

  /* USER CODE END SDIO_Init 2 */
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
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
  /* USER CODE END SPI1_Init 2 */
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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, LED_1_Pin | LCD_RST_Pin | LCD_BL_Pin | LCD_CS_Pin | LCD_DC_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(TP_CS_GPIO_Port, TP_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : USER_BUTTON_Pin */
  GPIO_InitStruct.Pin = USER_BUTTON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(USER_BUTTON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : LED_1_Pin LCD_RST_Pin LCD_BL_Pin LCD_CS_Pin
                           LCD_DC_Pin */
  GPIO_InitStruct.Pin = LED_1_Pin | LCD_RST_Pin | LCD_BL_Pin | LCD_CS_Pin | LCD_DC_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_CS_Pin */
  GPIO_InitStruct.Pin = TP_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(TP_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : TP_IRQ_Pin */
  GPIO_InitStruct.Pin = TP_IRQ_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(TP_IRQ_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI4_IRQn, 5, 0);
  HAL_NVIC_EnableIRQ(EXTI4_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */
  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */
void SwitchActiveTask(osThreadId_t newTaskHandle, int taskIndex)
{
  if (currentActiveTask != NULL)
  {
    osThreadSuspend(currentActiveTask);
  }

  if (newTaskHandle != NULL)
  {
    osThreadResume(newTaskHandle);
    currentActiveTask = newTaskHandle;
  }
  else
  {
    currentActiveTask = NULL;
  }
}

void TaskSwitchHandler(void *argument)
{
  TaskSwitchMessage_t switchMsg;

  for (;;)
  {
    if (osMessageQueueGet(taskSwitchQueue, &switchMsg, NULL, osWaitForever) == osOK)
    {
      SwitchActiveTask(switchMsg.taskHandle, switchMsg.taskIndex);
    }
  }
}

void RequestTaskSwitchFromISR(osThreadId_t newTaskHandle, int taskIndex, BaseType_t *pxHigherPriorityTaskWoken)
{
  TaskSwitchMessage_t switchMsg;
  switchMsg.taskHandle = newTaskHandle;
  switchMsg.taskIndex = taskIndex;

  if (taskSwitchQueue == NULL)
  {
    return;
  }

  osStatus_t status = osMessageQueuePut(taskSwitchQueue, &switchMsg, 0, 0);
  if (status == osOK)
  {
    *pxHigherPriorityTaskWoken = pdTRUE;
  }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  BaseType_t xHigherPriorityTaskWoken = pdFALSE;

  if (GPIO_Pin == TP_IRQ_Pin)
  {
    if (HAL_GPIO_ReadPin(TP_IRQ_GPIO_Port, TP_IRQ_Pin) == GPIO_PIN_RESET)
    {
      Coordinate rawPoint, displayPoint;
      int old_task = current_task;
      LcdTextMessage_t textMsg;

      if (Manual_Touch_GetRawPoint(&rawPoint))
      {
        Manual_Touch_ApplyCalibration(&displayPoint, &rawPoint);

        for (int i = 0; i < num_tasks; i++)
        {
          if (displayPoint.x >= task_boxes[i].x &&
              displayPoint.x < (task_boxes[i].x + task_boxes[i].w) &&
              displayPoint.y >= task_boxes[i].y &&
              displayPoint.y < (task_boxes[i].y + task_boxes[i].h))
          {
            current_task = i;
            if (old_task != i)
            {
              UiTaskSwitchMessage_t switchMsg;

              if (old_task >= 0 && old_task < num_tasks)
              {
                strcpy(switchMsg.oldTaskName, task_boxes[old_task].name);
                switchMsg.oldColor = COLOR_BLACK;
                switchMsg.hasOldTask = 1;
              }
              else
              {
                switchMsg.oldTaskName[0] = '\0';
                switchMsg.hasOldTask = 0;
              }

              strcpy(switchMsg.newTaskName, task_boxes[i].name);
              switchMsg.newColor = COLOR_MAGENTA;
              osMessageQueuePut(uiTaskSwitchQueue, &switchMsg, 0, 0);
              osSemaphoreRelease(uiUpdateSemaphore);
              switch (i)
              {
              case 0:
                RequestTaskSwitchFromISR(ledBlinkTaskHandle, i, &xHigherPriorityTaskWoken);
                strcpy(textMsg.text, "Task 02-1: BLINK LED");
                osMessageQueuePut(lcdTextQueue, &textMsg, 0, 0);
                osSemaphoreRelease(lcdUpdateSemaphore);
                break;
              case 1:
                RequestTaskSwitchFromISR(canCommTaskHandle, i, &xHigherPriorityTaskWoken);
                break;
              case 2:
                 RequestTaskSwitchFromISR(sdCardTaskHandle, i, &xHigherPriorityTaskWoken);
                break;
              case 3:
                RequestTaskSwitchFromISR(tempFramTaskHandle, i, &xHigherPriorityTaskWoken);
                break;
              }

              xHigherPriorityTaskWoken = pdTRUE;
            }

            return;
          }
        }

        if (displayPoint.x >= back_button_box.x &&
            displayPoint.x < (back_button_box.x + back_button_box.w) &&
            displayPoint.y >= back_button_box.y &&
            displayPoint.y < (back_button_box.y + back_button_box.h))
        {
          current_task = -1;
          RequestTaskSwitchFromISR(NULL, -1, &xHigherPriorityTaskWoken);
          if (old_task >= 0 && old_task < num_tasks)
          {
            sprintf(textMsg.text, "Nhom %02d", GROUP_NUMBER);
            osMessageQueuePut(lcdTextQueue, &textMsg, 0, 0);
            osSemaphoreRelease(lcdUpdateSemaphore);

            UiTaskSwitchMessage_t switchMsg;
            strcpy(switchMsg.oldTaskName, task_boxes[old_task].name);
            switchMsg.oldColor = COLOR_BLACK;
            switchMsg.hasOldTask = 1;
            switchMsg.newTaskName[0] = '\0';
            switchMsg.hasOldTask = 1;

            osMessageQueuePut(uiTaskSwitchQueue, &switchMsg, 0, 0);
            osSemaphoreRelease(uiUpdateSemaphore);

            xHigherPriorityTaskWoken = pdTRUE;
          }

          return;
        }
      }
    }

    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
  }
}
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
  if (hcan->Instance == CAN2)
  {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &RxHeader2, RxData2) == HAL_OK)
    {
      if (RxHeader2.DLC >= 2)
      {
        can2_received_group = RxData2[0];
        can2_received_temperature = RxData2[1];
        can2_received_data_flag = 1;
      }
    }
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
 * @brief  Function implementing the defaultTask thread.
 * @param  argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN 5 */

  /* Infinite loop */
  for (;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/* USER CODE BEGIN Header_StartCanCommTask */
/**
 * @brief Function implementing the canCommTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartCanCommTask */
void StartCanCommTask(void *argument)
{
  /* USER CODE BEGIN StartCanCommTask */
  /* Infinite loop */
  LcdTextMessage_t textMsg;
  int can2_received_temperature_int;
  uint32_t consecutive_timeouts = 0;
  uint32_t max_consecutive_timeouts = 5;
  osStatus_t queue_status, semaphore_status;
  HAL_StatusTypeDef can_status;
  for (;;)
  {
    uint8_t current_temp_c = (uint8_t)Read_Internal_Temperature();
    uint8_t temp_to_send = current_temp_c;
    TxData1[0] = GROUP_NUMBER;
    TxData1[1] = temp_to_send;

    can_status = HAL_CAN_AddTxMessage(&hcan1, &TxHeader1, TxData1, &TxMailbox1);
    if (can_status != HAL_OK)
    {
      sprintf(textMsg.text, "CAN TX Error: %d", can_status);
      queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
      if (queue_status == osOK)
      {
        osSemaphoreRelease(lcdUpdateSemaphore);
      }

      osDelay(1000);
      continue;
    }

    uint32_t timeout = HAL_GetTick() + 1000; // 1 second timeout
    while (!can2_received_data_flag && HAL_GetTick() < timeout)
    {
      osDelay(10);
    }
    if (can2_received_data_flag)
    {
      can2_received_data_flag = 0;
      can2_received_temperature_int = (int)can2_received_temperature;
      consecutive_timeouts = 0;
      sprintf(textMsg.text, "Group: %d, temp: %d *C", can2_received_group, can2_received_temperature_int);

      queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
      if (queue_status == osOK)
      {
        semaphore_status = osSemaphoreRelease(lcdUpdateSemaphore);
        if (semaphore_status != osOK)
        {
          osDelay(50);
        }
      }
      else
      {
        osDelay(100);
      }
    }
    else
    {
      consecutive_timeouts++;

      if (consecutive_timeouts <= max_consecutive_timeouts)
      {
        sprintf(textMsg.text, "CAN timeout (%lu/%lu)", consecutive_timeouts, max_consecutive_timeouts);
      }
      else
      {
        sprintf(textMsg.text, "CAN comm lost - check bus");

        // Try to recover CAN communication
        HAL_CAN_Stop(&hcan1);
        osDelay(100);
        HAL_StatusTypeDef can_init_status = HAL_CAN_Start(&hcan1);

        if (can_init_status == HAL_OK)
        {
          sprintf(textMsg.text, "CAN restarted");
          consecutive_timeouts = 0; // Reset counter on successful restart
        }
        else
        {
          sprintf(textMsg.text, "CAN restart failed: %d", can_init_status);
        }
      }

      queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
      if (queue_status == osOK)
      {
        osSemaphoreRelease(lcdUpdateSemaphore);
      }
    }

    osDelay(500); // Delay to avoid flooding the CAN bus
  }
  /* USER CODE END StartCanCommTask */
}

/* USER CODE BEGIN Header_StartTempFramTask */
/**
 * @brief Function implementing the tempFramTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTempFramTask */
	void StartTempFramTask(void *argument)
	{
	  /* USER CODE BEGIN StartTempFramTask */
	  /* Infinite loop */
	  LcdTextMessage_t textMsg;
	  TemperatureLog_t log;
	  GPIO_PinState prev_btn_state = GPIO_PIN_RESET;

	  for (;;)
	  {
		// Read temperature from FRAM
		GPIO_PinState current_btn_state = HAL_GPIO_ReadPin(USER_BUTTON_GPIO_Port, USER_BUTTON_Pin);

		CheckUserButtonAndSaveTemp(&current_btn_state);

		if (current_btn_state == GPIO_PIN_SET && prev_btn_state == GPIO_PIN_RESET)
		{
		  // Button just pressed
		  if (ReadTempLogFromFRAM(&hi2c2, USER_TEMP_ADDR, &log) == HAL_OK)
		  {
			sprintf(textMsg.text, "FRAM Read OK: %d *C", log.temperature);
		  }
		  else
		  {
			sprintf(textMsg.text, "FRAM Read ERROR");
		  }

		  // Send message to LCD update task
		  osMessageQueuePut(lcdTextQueue, &textMsg, 0, 0);
		  osSemaphoreRelease(lcdUpdateSemaphore);
		}

		prev_btn_state = current_btn_state;
		current_button_state = current_btn_state;

		osDelay(10); // Small delay for button debouncing
	  }
	  /* USER CODE END StartTempFramTask */
	}

/* USER CODE BEGIN Header_StartSDCardTask */
/**
 * @brief Function implementing the sdCardTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartSDCardTask */
void StartSDCardTask(void *argument)
{
  /* USER CODE BEGIN StartSDCardTask */
  /* SD Card Task - Read file once only */
  LcdTextMessage_t textMsg;
  uint32_t error_count = 0;
  uint32_t max_retries = 3;
  osStatus_t queue_status, semaphore_status;
  uint8_t sd_read_completed = 0; // Flag to track if SD read was completed

  // Perform SD card read operation once
  while (!sd_read_completed && error_count <= max_retries)
  {
    byte_read = SD_ReadTeamFile(read_data, sizeof(read_data));
    if (byte_read > 0)
    {
      sprintf(textMsg.text, "SD: %s", read_data);
      sd_read_completed = 1;

      queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
      if (queue_status == osOK)
      {
        osSemaphoreRelease(lcdUpdateSemaphore);
      }
    }
    else
    {
      error_count++;
      sprintf(textMsg.text, "Read error (%d) try %lu", byte_read, error_count);

      queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
      if (queue_status == osOK)
      {
        semaphore_status = osSemaphoreRelease(lcdUpdateSemaphore);
        if (semaphore_status != osOK)
        {
        }
      }

      if (error_count <= max_retries)
      {
        osDelay(500 * error_count);

        if (SD_Init())
        {
          sprintf(textMsg.text, "SD reinitialized (attempt %lu)", error_count);
          queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
          if (queue_status == osOK)
          {
            osSemaphoreRelease(lcdUpdateSemaphore);
          }
          osDelay(1000);

          byte_read = SD_ReadTeamFile(read_data, sizeof(read_data));
          if (byte_read > 0)
          {
            sprintf(textMsg.text, "Recovery success: %s", read_data);
            sd_read_completed = 1;
          }
          else
          {
            sprintf(textMsg.text, "Recovery failed (%d)", byte_read);
          }

          // Send recovery result to LCD
          queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
          if (queue_status == osOK)
          {
            osSemaphoreRelease(lcdUpdateSemaphore);
          }
        }
        else
        {
          sprintf(textMsg.text, "SD reinit failed (attempt %lu)", error_count);
          queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
          if (queue_status == osOK)
          {
            osSemaphoreRelease(lcdUpdateSemaphore);
          }
        }
      }
      else
      {
        // Max retries exceeded - enter error state
        sprintf(textMsg.text, "SD card failed - check hardware");
        queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
        if (queue_status == osOK)
        {
          osSemaphoreRelease(lcdUpdateSemaphore);
        }
        break; // Exit the retry loop
      }
    }
  }

  // SD read operation completed (either successfully or after max retries)
  // Task now enters suspended state and waits to be resumed by touch interface
  for (;;)
  {
    // Task is suspended initially, will be resumed when selected via touch
    osThreadSuspend(osThreadGetId()); // Suspend self after completing SD operation

    // When resumed by touch interface, display the previously read content
    if (sd_read_completed && byte_read > 0)
    {
      sprintf(textMsg.text, "SD: %s", read_data);
    }
    else
    {
      sprintf(textMsg.text, "SD: Read failed");
    }

    // Send message to LCD
    queue_status = osMessageQueuePut(lcdTextQueue, &textMsg, 0, 1000);
    if (queue_status == osOK)
    {
      osSemaphoreRelease(lcdUpdateSemaphore);
    }

    // Small delay before suspending again
    osDelay(100);
  }
  /* USER CODE END StartSDCardTask */
}

/* USER CODE BEGIN Header_StartLedBlinkTask */
/**
 * @brief Function implementing the ledBlinkTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLedBlinkTask */
void StartLedBlinkTask(void *argument)
{
  /* USER CODE BEGIN StartLedBlinkTask */
  /* Infinite loop */
  for (;;)
  {
    // Task is suspended initially, will be resumed when selected
    Task2_LedBlink(GPIOB, GPIO_PIN_1, BLINK_SPEED_1000_MS);
  }
  /* USER CODE END StartLedBlinkTask */
}

/* USER CODE BEGIN Header_StartLcdUpdateTextTask */
/**
 * @brief Function implementing the lcdUpdateTextTa thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartLcdUpdateTextTask */
void StartLcdUpdateTextTask(void *argument)
{
  /* USER CODE BEGIN StartLcdUpdateTextTask */
  /* Infinite loop */
  LcdTextMessage_t textMsg;

  for (;;)
  {
    // Wait for signal to update text
    if (osSemaphoreAcquire(lcdUpdateSemaphore, osWaitForever) == osOK)
    {
      // Get message from queue
      if (osMessageQueueGet(lcdTextQueue, &textMsg, NULL, 100) == osOK)
      {
        // Acquire LCD mutex
        if (osMutexAcquire(lcdMutex, 1000) == osOK)
        {
          // Disable touch interrupt while updating LCD
          HAL_NVIC_DisableIRQ(EXTI4_IRQn);

          Manual_LCD_UpdateInfoText(textMsg.text);

          // Clear pending interrupt and re-enable
          __HAL_GPIO_EXTI_CLEAR_IT(TP_IRQ_Pin);
          HAL_NVIC_EnableIRQ(EXTI4_IRQn);

          osMutexRelease(lcdMutex);
        }
      }
    }
  }
  /* USER CODE END StartLcdUpdateTextTask */
}

/* USER CODE BEGIN Header_StartUIManagerTask */
/**
 * @brief Function implementing the uiManagerTask thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartUIManagerTask */
void StartUIManagerTask(void *argument)
{
  /* USER CODE BEGIN StartUIManagerTask */
  UiTaskSwitchMessage_t taskSwitchMsg;

  for (;;)
  {
    // Wait for signal to update UI
    if (osSemaphoreAcquire(uiUpdateSemaphore, osWaitForever) == osOK)
    {
      // Get consolidated task switch message
      if (osMessageQueueGet(uiTaskSwitchQueue, &taskSwitchMsg, NULL, 100) == osOK)
      {
        // Acquire LCD mutex
        if (osMutexAcquire(lcdMutex, 1000) == osOK)
        {
          // Disable touch interrupt while updating UI
          HAL_NVIC_DisableIRQ(EXTI4_IRQn);

          // Perform both UI operations atomically
          if (taskSwitchMsg.hasOldTask)
          {
            // Reset old task color
            Manual_LCD_RefillTaskBox(taskSwitchMsg.oldTaskName, taskSwitchMsg.oldColor);
          }

          if (taskSwitchMsg.newTaskName[0] != '\0')
          {
            // Highlight new task color
            Manual_LCD_RefillTaskBox(taskSwitchMsg.newTaskName, taskSwitchMsg.newColor);
          }

          // Clear pending interrupt and re-enable
          __HAL_GPIO_EXTI_CLEAR_IT(TP_IRQ_Pin);
          HAL_NVIC_EnableIRQ(EXTI4_IRQn);

          osMutexRelease(lcdMutex);
        }
      }
    }
  }
  /* USER CODE END StartUIManagerTask */
}

/* USER CODE BEGIN Header_StartTaskSwitchHandler */
/**
 * @brief Function implementing the taskSwitchHandler thread.
 * @param argument: Not used
 * @retval None
 */
/* USER CODE END Header_StartTaskSwitchHandler */
void StartTaskSwitchHandler(void *argument)
{
  /* USER CODE BEGIN StartTaskSwitchHandler */
  TaskSwitchMessage_t switchMsg;

  for (;;)
  {
    // Wait for task switch request
    if (osMessageQueueGet(taskSwitchQueue, &switchMsg, NULL, osWaitForever) == osOK)
    {
      SwitchActiveTask(switchMsg.taskHandle, switchMsg.taskIndex);
    }
  }
  /* USER CODE END StartTaskSwitchHandler */
}

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
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
