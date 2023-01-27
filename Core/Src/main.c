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
#include "fatfs.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
/* ----------------- Coments ----------------------------------
 * В этом проекте используется SPI3, чтобы считать голосовые файлы с microSD
 * и озвучить их посредством DAC с использованием DMA во время пробуждения по будильнику.
 * Время для RTC задается сперва текущее в main() а затем, также в main() задается время пробуждения.
 * В момент пробуждения срабатывает прерывание RTC-Alarm'a, обработка которого происходит в файле
 * stm32lxx_it.c посредством функций FreeRTOS. До этого момента крутится таск FirstTask, выводящий
 * посекундно время на PuTTY с помощью USART,а в момент прерывания вызывается SoundTask, озвучивающий
 * время пробуждения и выводящий сообщение "ALARM !!! ALARM !!! ALARM !!!" также с помощью USART
 * на экран PuTTY. Прервать звонок будильника можно с помощью голубой кнопки User Button,
 * расположенной на плате Nucleo-L152RE. В этот момент срабатывает другое прерывание, повторно вызывающее
 * FirstTask, обработка которого происходит в stm32lxx_it.c с помощью функций FreeRTOS.
 *
 * Чтобы переназначить время, надо повторно скомпилировать проект. Подсоединение проводов между
 * платой Nucleo-L152RE, платой microSD и колонками смотри на приложенных в папке проекта фото.
 */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SIZE_DAC_BUFF  512
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
RTC_HandleTypeDef hrtc;

SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim2;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;

osThreadId defaultTaskHandle;
/* USER CODE BEGIN PV */
char Numbers[60][3]={"00", "01", "02", "03", "04", "05", "06", "07", "08", "09",
                   "10", "11", "12", "13", "14", "15", "16", "17", "18", "19",
                   "20", "21", "22", "23", "24", "25", "26", "27", "28", "29",
                   "30", "31", "32", "33", "34", "35", "36", "37", "38", "39",
                   "40", "41", "42", "43", "44", "45", "46", "47", "48", "49",
                   "50", "51", "52", "53", "54", "55", "56", "57", "58", "59"};
char TimeArray[7][3];
char Param[20];
extern volatile uint16_t Timer1=0;

TaskHandle_t xHandle_1, xHandle_2, xHandle_Sound;
RTC_TimeTypeDef sTime, sAlarmTime;
RTC_DateTypeDef sDate, sAlarmDate;
RTC_AlarmTypeDef sAlarm;

FATFS SDFatFs;
char USER_Path[4]={'0','0','0','0'}; // logical drive path
FIL MyFile;
unsigned char DAC_Buff[SIZE_DAC_BUFF];
UINT cnt;

WPRESULT wp_open( FIL *file, const char *FileName, wp_format *format );
void wp_init (wp_format *format);
char wave_playback(const char *FileName);


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_RTC_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_SPI3_Init(void);
static void MX_TIM2_Init(void);
void StartDefaultTask(void const * argument);

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
  MX_RTC_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_FATFS_Init();
  MX_SPI3_Init();
  MX_TIM2_Init();
  /* USER CODE BEGIN 2 */

  /* USER CODE END 2 */

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
  /* definition and creation of defaultTask */
  //osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 512);
  //defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  HAL_UART_Transmit(&huart2, "Enter Start RTC Date & Time ", 29, 100);
  SetTime( &sDate, &sTime);
  HAL_UART_Transmit(&huart2, "Enter Alarm RTC Date & Time ", 29, 100);
  SetTime( &sAlarmDate, &sAlarmTime);

  sAlarm.AlarmTime=sAlarmTime;
  sAlarm.AlarmMask=RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask=RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel=RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay=sAlarmDate.Date;
  sAlarm.Alarm=RTC_ALARM_A;

  HAL_RTC_SetDate(&hrtc, &sDate, RTC_FORMAT_BCD);
  HAL_RTC_SetTime(&hrtc, &sTime, RTC_FORMAT_BCD);

  HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD);

  /* Чтобы таски создались, надо заблокировать строки кода, создающие defaultTask !!! Потому, что
   * defaultTask занимает слишком много памяти в куче ( 2160 Байт ) и для моих тасков не остается
   * места !!!
   */
	  if( xTaskCreate(prvFirstTask, "First", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle_1) != pdPASS){
		  HAL_UART_Transmit(&huart2, "FirstTask creating ERROR !!!\r\n", 30, 100 );
	  }

	  if( xTaskCreate(prvSoundTask, "Sound", configMINIMAL_STACK_SIZE, NULL, tskIDLE_PRIORITY, &xHandle_Sound) != pdPASS){
		  HAL_UART_Transmit(&huart2, "SoundTask creating ERROR !!!\r\n", 30, 100 );
	  }


	  vTaskStartScheduler();

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
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_LSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.LSIState = RCC_LSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLL_DIV2;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_RTC;
  PeriphClkInit.RTCClockSelection = RCC_RTCCLKSOURCE_LSI;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
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
  RTC_AlarmTypeDef sAlarm = {0};

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
  /** Enable the Alarm A
  */
  sAlarm.AlarmTime.Hours = 0x0;
  sAlarm.AlarmTime.Minutes = 0x0;
  sAlarm.AlarmTime.Seconds = 0x0;
  sAlarm.AlarmTime.SubSeconds = 0x0;
  sAlarm.AlarmTime.DayLightSaving = RTC_DAYLIGHTSAVING_NONE;
  sAlarm.AlarmTime.StoreOperation = RTC_STOREOPERATION_RESET;
  sAlarm.AlarmMask = RTC_ALARMMASK_NONE;
  sAlarm.AlarmSubSecondMask = RTC_ALARMSUBSECONDMASK_ALL;
  sAlarm.AlarmDateWeekDaySel = RTC_ALARMDATEWEEKDAYSEL_DATE;
  sAlarm.AlarmDateWeekDay = 0x1;
  sAlarm.Alarm = RTC_ALARM_A;
  if (HAL_RTC_SetAlarm_IT(&hrtc, &sAlarm, RTC_FORMAT_BCD) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN RTC_Init 2 */

  /* USER CODE END RTC_Init 2 */

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
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 10;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 39999;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 10;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

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
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

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
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_6|GPIO_PIN_7, GPIO_PIN_RESET);

  /*Configure GPIO pin : PC13 */
  GPIO_InitStruct.Pin = GPIO_PIN_13;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PA6 PA7 */
  GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_7;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI15_10_IRQn, 10, 0);
  HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);

}

/* USER CODE BEGIN 4 */

WPRESULT wp_open( FIL *file, const char *FileName, wp_format *format )
{
  unsigned char res;                          //dlya chraneniya resultata
  unsigned char buff[64];                     //bufer dlya zagolovka faila
  unsigned char cnt;                          //kolichestvo prochitannych bait
  unsigned long i;                            //wspom. peremennaya

  res = f_open ( file,FileName, FA_OPEN_EXISTING|FA_READ );
  if(res) return WP_ERROR_FILE;
  res = f_read ( file, &buff[0], 64, (UINT *)&cnt );
  if(res || cnt!=64) return WP_ERROR_FILE;

  if(buff[0]!='R' || buff[1]!='I' || buff[2]!='F' || buff[3]!='F')     res = 2;

  if(buff[8]!='W' || buff[9]!='A' || buff[10]!='V' || buff[11]!='E')   res = 2;

  if(buff[12]!='f' || buff[13]!='m' || buff[14]!='t' || buff[15]!=' ') res = 2;

  //bayty 20,21  - 1 i 0
  if(buff[20]!=1 || buff[21]!=0) res = 2;

  //wychislit razmer chanka "fmt "
  i = buff[16] | (buff[17]<<8) | (buff[18]<<16) | (buff[19]<<24);

  //posle chanka "fmt " dolgen sledovat chank "data"
  if(buff[20+i]!='d' || buff[21+i]!='a' || buff[22+i]!='t' || buff[23+i]!='a') res = 2;

  if(res==2)
  {
    f_close(file);
    return WP_ERROR_FILE;
  }

  //                    zagruzit format dannych
  format->Channels       = buff[22];
  format->SamplesPerSec  = buff[24] | (buff[25]<<8) | (buff[26]<<16) | (buff[27]<<24);
  format->AvgBytesPerSec = buff[28] | (buff[29]<<8) | (buff[30]<<16) | (buff[31]<<24);
  format->BitsPerSample  = buff[34] | (buff[35]<<8);
  format->Size           = buff[24+i]|(buff[25+i]<<8)|(buff[26+i]<<16)|(buff[27+i]<<24);

  //failovyi ukazatel na nachalo dannych
  //f_lseek(file,i+28);
  f_close(&MyFile);
  return WP_OK;
}

void wp_init (wp_format *format)
{
//??????????? ?????? ????? ???? ??? ?????? ?????? ?? ???
//RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;            //???????????? ????? GPIOA
RCC->AHBENR |= RCC_AHBENR_GPIOAEN;             // IO port A clock enabled
RCC->APB1ENR |= RCC_APB1ENR_DACEN;             //DAC interface clock enabled
DAC->CR      |= DAC_CR_DMAEN1;                 //DAC channel1 DMA mode enabled
DAC->CR      |= DAC_CR_BOFF1;                  //DAC channel1 output buffer disabled
//DAC->CR      &=~DAC_CR_BOFF1;
DAC->CR      |= DAC_CR_EN1;                    //DAC channel1 enabled
//????????? ?????? ????? ??? ??? ??????????? ?????? ?? ??????
//? ????????????? ??????? ?????? ??????? ?????? ????:
RCC->AHBENR         |= RCC_AHBENR_DMA1EN;      //DMA1 clock enabled
DMA1_Channel2->CPAR  = (uint32_t)&DAC->DHR8R1; //Base address of the peripheral data register from which the data will be read
												// Address of DAC channel1 8-bit right aligned data holding register
DMA1_Channel2->CMAR  = (uint32_t)&DAC_Buff[0]; //Base address of the memory area from which the data will be read.
DMA1_Channel2->CCR  |=  DMA_CCR_DIR;          //Data transfer direction - read from memory
DMA1_Channel2->CNDTR =  512;                   //Number of data to transfer
DMA1_Channel2->CCR  &= ~DMA_CCR_PINC;         //Peripheral increment mode disabled
DMA1_Channel2->CCR  |=  DMA_CCR_MINC;         //Memory increment mode enabled
DMA1_Channel2->CCR  &= ~DMA_CCR_PSIZE;        //Peripheral size - 8 bit
DMA1_Channel2->CCR  &= ~DMA_CCR_MSIZE;        //Memory size - 8 bit
DMA1_Channel2->CCR  |=  DMA_CCR_CIRC;         //Circular mode enabled
DMA1_Channel2->CCR  |=  DMA_CCR_PL;           //Channel priority level - very high
DMA1_Channel2->CCR  |=  DMA_CCR_EN;           //Channel enabled
//?????? ???????? ?????????? ???????? ????? ??? ????? ??????????? ?? TIM6
RCC->APB1ENR        |=  RCC_APB1ENR_TIM6EN;    //Timer 6 clock enabled
TIM6->PSC            =  0;                     //Prescaler value - 8000 ???
TIM6->ARR            =  (uint32_t)32000000/ format->SamplesPerSec; // Auto-reload value
//TIM6->ARR            = (uint32_t)459; // before 32000000/44100
TIM6->DIER          |=  TIM_DIER_UDE; // Update DMA request enabled.
}

char wave_playback(const char *FileName)
{
  FRESULT res;                                //??? ????????????? ????????? ??????????
  //FIL file;                                   //???????? ??????
  UINT cnt;                                   //?????????? ??????? ??????????? ????

  memset(DAC_Buff,'\0',sizeof(DAC_Buff));

  res = f_open( &MyFile, FileName, FA_OPEN_EXISTING|FA_READ );   //??????? ???? FileName ??? ??????
  if(res) return 1;
  res = f_lseek(&MyFile,0x2c);                  //??????????? ????????? ?? ?????? ???????? ??????
  if(res) return 2;
  f_read (&MyFile,&DAC_Buff[0],512,&cnt);       //????????? ????? ???? ???????
  if(res) return 3;
  TIM6->CR1 |= TIM_CR1_CEN;                   //Counter enable

  //                        ???????????????
  while(1)
  {
     while(!(DMA1->ISR & DMA_ISR_HTIF2)) {}   //Channel 2 Half Transfer flag
     f_read (&MyFile,&DAC_Buff[0],256,&cnt);    //????????? ?? ???????
     DMA1->IFCR |= DMA_ISR_HTIF2;             //???????? ????
     if(cnt<256)break;                        //???? ????? ?????

     while(!(DMA1->ISR & DMA_ISR_TCIF2)) {}   //Channel 2 Transfer Complete flag
     f_read (&MyFile,&DAC_Buff[256],256,&cnt);  //????????? ?? ???????
     DMA1->IFCR |= DMA_ISR_TCIF2;             //???????? ????
     if(cnt<256)break;                        //???? ????? ?????
  }

  TIM6->CR1 &= ~TIM_CR1_CEN;                  //Counter disable
  f_close(&MyFile);                             //??????? ????
  return 0;                                   //???????? ?????????? ?-??
}

void ParamToArray( char *pParam )
{
int count,num=0,ParamLength;
char *pStr=NULL, *pPrev=NULL;

    pStr=pPrev=pParam;
    ParamLength=strlen(pParam);
    for(count=0; count<ParamLength; count++){
      if(*pStr!=' ' && *pStr!='/' && *pStr!=':'){
          pStr++;
      }else{
          *pStr='\0';
          strcpy(TimeArray[num],pPrev);
          pStr++;
          pPrev=pStr;
          num++;
      }
    }
}
uint8_t DecToBCD( int num){
	return (uint8_t)((num%10)+(num-(num%10))/10*16);
}

int BCDToDec( uint8_t num){
	return (int)((num%16)+(num-(num%16))/16*10);
}

void SetTime( RTC_DateTypeDef *pDate, RTC_TimeTypeDef *pTime ){
char ch;
HAL_StatusTypeDef status;
int count=0, num=0, flag;

	do{
		memset( Param, '\0', sizeof(Param));
		ch='\0';
		flag=0;
		count=0;

  		HAL_UART_Transmit(&huart2, "in form <dd/mm/yy hh:mm:ss> : ", 29, 100);

		while(ch!='\r'){
			status=HAL_UART_Receive( &huart2, (uint8_t *)&ch, 1, 100);
			if(status==HAL_OK){
				HAL_UART_Transmit( &huart2, (uint8_t *)&ch, 1, 100);
				if( ch==127 && count>0) {
					count--;
				}else{
					if( count>=0 && ch!='\r'){
						Param[count]=ch;
						count++;
					}
				}
			}
		}

		Param[count]=' ';
		HAL_UART_Transmit(&huart2, "\n", 1, 100);

		ParamToArray(Param);

		for( count=0; count<6; count++){
	  		for( num=0; num<61; num++){
		  		if( (strcmp(TimeArray[count],Numbers[num]))==0 ){
					switch(count){
						case 0:  pDate->Date=DecToBCD(num);
								 if(num>=1 && num<=31 ){
									 flag=1;
								 }else{
									 flag=0;
									 count=6;
								 }
								 num=61;
						break;
						case 1:  pDate->Month=DecToBCD(num);
								 if(num>=1 && num<=12 ){
									 flag=1;
								 }else{
									 flag=0;
									 count=6;
								 }
								 num=61;
						break;
						case 2:  pDate->Year=DecToBCD(num);
								 if(num>=0 && num<=59 ){
									 flag=1;
								 }else{
									 flag=0;
									 count=6;
								 }
								 num=61;
						break;
						case 3:  pTime->Hours=DecToBCD(num);
								 if(num>=0 && num<=23 ){
									 flag=1;
								 }else{
									 flag=0;
									 count=6;
								 }
								 num=61;
						break;
						case 4:  pTime->Minutes=DecToBCD(num);
								 if(num>=0 && num<=59 ){
									 flag=1;
								 }else{
									 flag=0;
									 count=6;
								 }
								 num=61;
						break;
						case 5:  pTime->Seconds=DecToBCD(num);
								 if(num>=0 && num<=59 ){
									 flag=1;
								 }else{
									 flag=0;
									 count=6;
								 }
								 num=61;
						break;
					}
				}else if( num==60 ){ flag=0; num=61; count=6; }
			}
		}
		if( !flag ) HAL_UART_Transmit( &huart2, "\r\n WRONG TIME OR DATE !!!\r\n", 28, 40);
	}while( !flag );

	pDate->WeekDay = RTC_WEEKDAY_MONDAY;
	pTime->TimeFormat = 0x0;
	pTime->SubSeconds = 0x0;
	pTime->SecondFraction = 0x0;
	pTime->DayLightSaving = 0x0;
	pTime->StoreOperation = 0x0;
}



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
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM9 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM9) {
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
