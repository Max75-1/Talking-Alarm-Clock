#ifndef SD_H_ 
#define SD_H_ 
//-------------------------------------------------- 
#include "stm32l1xx_hal.h" 
#include <string.h> 
#include <stdlib.h> 
#include <stdint.h> 

#define CS_SD_GPIO_PORT GPIOA 
#define CS_SD_PIN GPIO_PIN_7
#define SD_CARDDETECT() HAL_GPIO_WritePin( GPIOA, GPIO_PIN_6, GPIO_PIN_SET);
#define SS_SD_SELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_RESET) 
#define SS_SD_DESELECT() HAL_GPIO_WritePin(CS_SD_GPIO_PORT, CS_SD_PIN, GPIO_PIN_SET) 
//#define LD_ON HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_SET); //Green
//#define LD_OFF HAL_GPIO_WritePin(GPIOA, GPIO_PIN_5, GPIO_PIN_RESET); //Green 

/* Card type flags (CardType) */ 
#define CT_MMC 0x01 /* MMC ver 3 */ 
#define CT_SD1 0x02 /* SD ver 1 */ 
#define CT_SD2 0x04 /* SD ver 2 */ 
#define CT_SDC (CT_SD1|CT_SD2) /* SD */ 
#define CT_BLOCK 0x08 /* Block addressing */ 
//-------------------------------------------------- 
//-------------------------------------------------- 
void SD_PowerOn(void);
uint8_t sd_ini(void);
void Error (void);
uint8_t SPIx_WriteRead(uint8_t Byte);
void SPI_SendByte(uint8_t bt);
uint8_t SPI_ReceiveByte(void);
void SPI_Release(void); 
uint8_t SD_Read_Block (uint8_t *buff, uint32_t lba);
uint8_t SD_Write_Block (uint8_t *buff, uint32_t lba);
uint8_t SPI_wait_ready(void);
//-------------------------------------------------- 
typedef struct sd_info { 
  volatile uint8_t type;//typ karty 
} sd_info_ptr; 
//-------------------------------------------------- 
#endif /* SD_H_ */ 