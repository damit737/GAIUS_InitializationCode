/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __W25Q32FV_H
#define __W25Q32FV_H

#include "main.h"

#define FLASH_CS_Select                 HAL_GPIO_WritePin(FLASH_GPIO_CS, FLASH_CS_PIN, GPIO_PIN_RESET)          // set PD13 Low
#define FLASH_CS_DeSelect               HAL_GPIO_WritePin(FLASH_GPIO_CS, FLASH_CS_PIN, GPIO_PIN_SET)            // set PD13 High

/*
#define FLASH_CS1_Select                HAL_GPIO_WritePin( FLASH_GPIO_CS1, FLASH_CS1_PIN, GPIO_PIN_RESET)       // set PD13 Low
#define FLASH_CS1_DeSelect              HAL_GPIO_WritePin( FLASH_GPIO_CS1, FLASH_CS1_PIN, GPIO_PIN_SET)         // set PD13 High

#define FLASH_WP_Select                 HAL_GPIO_WritePin(FLASH_GPIO_WP, FLASH_WP_PIN, GPIO_PIN_RESET);         // set PE3 Low
#define FLASH_WP_DeSelect               HAL_GPIO_WritePin(FLASH_GPIO_WP, FLASH_WP_PIN, GPIO_PIN_SET);           // set PE3 High

#define FLASH_HOLD_Select               HAL_GPIO_WritePin(FLASH_GPIO_HOLD, FLASH_HOLD_PIN, GPIO_PIN_RESET);     
#define FLASH_HOLD_DeSelect             HAL_GPIO_WritePin(FLASH_GPIO_HOLD, FLASH_HOLD_PIN, GPIO_PIN_SET);       

#define FLASH_HOLD1_Select              HAL_GPIO_WritePin(FLASH_GPIO_HOLD1, FLASH_HOLD1_PIN, GPIO_PIN_RESET);   
#define FLASH_HOLD1_DeSelect            HAL_GPIO_WritePin(FLASH_GPIO_HOLD1, FLASH_HOLD1_PIN, GPIO_PIN_SET);
*/

#define Write_Enable                    0x06
#define Volatile_SR_Write_Enable        0x50
#define Write_Disable                   0x04
#define Read_Status_Register_1          0x05
#define Write_Status_Register_1         0x01
#define Read_Status_Register_2          0x35
#define Write_Status_Register_2         0x31
#define Read_Status_Register_3          0x15
#define Write_Status_Register_3         0x11
#define Chip_Erase                      0xC7//0x60
#define Sector_Erase                    0x20
#define Erase_Program_Suspend           0x75
#define Erase_Program_Resume            0x7A
#define Power_Down                      0xB9
#define Release_Powre_Down              0xAB
#define Device_ID                       0x90
#define JEDEC_ID                        0x9F
#define Global_Block_Lock               0x7E
#define Global_Block_Unlock             0x98
#define Enter_QPI_Mode                  0x38
#define Enable_Reset                    0x66
#define Reset_Device                    0x99
#define Fast_Read_Data                  0x0B
#define Read_Data												0x03
#define _64K_Black_Erase                0xD8

#define _Manufacturer_ID								0xEF

#define FLASH_PAGE_SIZE                 256

uint8_t Inital_SPI_Flash(void);

uint8_t Flash_Read_Status(uint8_t Register);
uint8_t Flash_SPI_Send_Byte(uint8_t Flash_Command);

void Flash_Write_Enable(void);

void Flash_Write_Status(uint8_t Flash_Status_1,uint8_t Flash_Status_2);

void Flash_Block_Erase(unsigned long Block_Address);

unsigned char Flash_Byte_Program(unsigned long Flash_ADD,uint8_t Program_Data);
unsigned char Flash_Program(unsigned long ADDR, uint8_t* Data, uint16_t size);

void Flash_Read( unsigned int Flash_Start_Address, unsigned char *out, unsigned int size);

void Flash_Chip_Erase(void);
void Flash_Sector_Erase( unsigned long sector_addr );



#endif
