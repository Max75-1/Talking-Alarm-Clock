/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32l1xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include <stdio.h>

#include "FreeRtOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
//#include "fatfs.h"

#include "sd.h"
//#include "ff.h"

#define SIZE_DAC_BUFF  512
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
/* USER CODE BEGIN Private defines */




typedef struct
{
  uint8_t  Channels;          // kolichestvo kanalov                                             /
  uint16_t SamplesPerSec;     //chastota diskretizacii signala, Hz
  uint8_t  BitsPerSample;      //razryadnost dannych (8,16)
  uint32_t Size;              //kolichestvo poleznych dannych, bait
  uint16_t AvgBytesPerSec;    //kolichestvo bait w sekundu ( potok dannych )
} wp_format;

//wp_format MyFormat;

typedef enum
{
  WP_OK = 0,                  //fail uspeshno wosproizveden
  WP_ERROR_FILE,              //fail ne otkryt( ne naiden, ne vernyi format i t.d.)
  WP_ERROR_READ_FILE,         //oshibka chteniya
  WP_STOP,                    //ostanovka wosproizvedeniya, fail zakryt
  WP_PAUSE,                   //pausa vosproizvedeniya
  WP_PLAY,                    //vozobnovit vosproizvedenie
  WP_NONE,                    //
  WP_ERROR                    //prosto oshibka
}WPRESULT;

void prvFirstTask   ( void *pvParameters );
void prvSecondTask   ( void *pvParameters );
void prvSoundTask ( void *pvParameters);
void SetTime( RTC_DateTypeDef *pDate, RTC_TimeTypeDef *pTime );
void ParamToArray( char *pParam );
uint8_t DecToBCD ( int num);
int BCDToDec (uint8_t num);

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
