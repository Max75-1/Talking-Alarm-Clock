#include "main.h"
#include "stm32l1xx_hal.h"
#include "cmsis_os.h"

extern RTC_HandleTypeDef hrtc;
extern UART_HandleTypeDef huart2;

void prvFirstTask   ( void *pvParameters )
{
RTC_TimeTypeDef sTime;
RTC_DateTypeDef sDate;
char RTCTime[40];
int PrevSeconds=999;

	while(1){

		HAL_RTC_GetDate(&hrtc, &sDate, RTC_FORMAT_BCD); 
		HAL_RTC_GetTime(&hrtc, &sTime, RTC_FORMAT_BCD);
		
		
		sprintf( RTCTime, "RTC Time is %02d/%02d/%02d %02d:%02d:%02d \r\n", BCDToDec(sDate.Date), BCDToDec(sDate.Month), BCDToDec(sDate.Year),
																			BCDToDec(sTime.Hours), BCDToDec(sTime.Minutes), BCDToDec(sTime.Seconds) );
		if( PrevSeconds==(int)sTime.Seconds ){
			
		}else{
			HAL_UART_Transmit(&huart2, (uint8_t *)RTCTime, 32, 40);
		}
		PrevSeconds=(int)sTime.Seconds;
		
		memset( RTCTime, '\0', sizeof(RTCTime));

	}
}
