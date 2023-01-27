#include "main.h"
#include "stm32l1xx_hal.h"
#include "fatfs.h"
#include "sd.h"

#include <string.h>
#include <stdio.h>

extern int BCDToDec (uint8_t num);

extern TaskHandle_t xHandle_1, xHandle_2;
extern UART_HandleTypeDef huart2;
extern char Numbers[60][3];
extern TIM_HandleTypeDef htim2;
extern RTC_TimeTypeDef sAlarmTime;
extern FATFS SDFatFs;
extern char USER_Path[4]; // logical drive path
extern FIL MyFile;
extern unsigned char DAC_Buff[512];
char str[23];

wp_format MyFormat;

extern WPRESULT wp_open( FIL *file, const char *FileName, wp_format *format );
extern void wp_init (wp_format *format);
extern char wave_playback(const char *FileName);

void prvSoundTask ( void *pvParameters)
{
	vTaskSuspend(NULL);

	vTaskSuspend(xHandle_1);
	vTaskPrioritySet( xHandle_1, tskIDLE_PRIORITY + 2 );

	HAL_TIM_Base_Start_IT(&htim2);

	disk_initialize(SDFatFs.drv);

	while(1){
		if(f_mount(&SDFatFs,(TCHAR const*)USER_Path,1)!=FR_OK){
			Error();
	     }else{

	    	 HAL_UART_Transmit(&huart2,"ALARM !!! ALARM !!! ALARM !!!\r\n", 32, 100);

	         /*memset( str, '\0', sizeof(str));
	         strcpy( str, Numbers[BCDToDec(sAlarmTime.Hours)] );
	         strcat( str, "h.wav");

	         //vTaskSuspend(xHandle_2);
	         wp_open(&MyFile,str,&MyFormat);
	         wp_init(&MyFormat);
	         wave_playback(str);
	         //vTaskResume(xHandle_2);

	         memset( str, '\0', sizeof(str));
	         strcpy( str, Numbers[BCDToDec(sAlarmTime.Minutes)] );
	         strcat( str, "m.wav");

	         //vTaskSuspend(xHandle_2);
	         wp_open(&MyFile,str,&MyFormat);
	         wp_init(&MyFormat);
	         wave_playback(str);
	         //vTaskResume(xHandle_2);*/

	         //vTaskSuspend(xHandle_2);
	         wp_open(&MyFile,"123.wav",&MyFormat);
	         wp_init(&MyFormat);
	         wave_playback("123.wav");
	         //vTaskResume(xHandle_2);

	         f_close(&MyFile);
	     }
	     f_mount(NULL,(TCHAR const*)USER_Path,1); // Unmount
	}

}


