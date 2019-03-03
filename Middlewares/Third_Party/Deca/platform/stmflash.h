#ifndef __STMFLASH_H__
#define __STMFLASH_H__
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"
//////////////////////////////////////////////////////////////////////////////////////////////////////
#define STM32_FLASH_SIZE 	1024 	 		
#define STM32_FLASH_WREN 	1              	//(0£¬locked ;1£¬write  enable)
//////////////////////////////////////////////////////////////////////////////////////////////////////

//FLASH start address 
#define STM32_FLASH_BASE 0x08000000 	//STM32 FLASH start address 
 

//stm32f407 FLASH  sector start address 
#define ADDR_FLASH_SECTOR_0     ((uint32_t)0x08000000) 	// 16 Kbytes  
#define ADDR_FLASH_SECTOR_1     ((uint32_t)0x08004000) 	// 16 Kbytes  
#define ADDR_FLASH_SECTOR_2     ((uint32_t)0x08008000) 	// 16 Kbytes  
#define ADDR_FLASH_SECTOR_3     ((uint32_t)0x0800C000) 	//16 Kbytes  
#define ADDR_FLASH_SECTOR_4     ((uint32_t)0x08010000) 	//64 Kbytes  
#define ADDR_FLASH_SECTOR_5     ((uint32_t)0x08020000) 	//128 Kbytes  
#define ADDR_FLASH_SECTOR_6     ((uint32_t)0x08040000) 	//128 Kbytes  
#define ADDR_FLASH_SECTOR_7     ((uint32_t)0x08060000) 	//128 Kbytes  
#define ADDR_FLASH_SECTOR_8     ((uint32_t)0x08080000) 	//128 Kbytes  
#define ADDR_FLASH_SECTOR_9     ((uint32_t)0x080A0000) 	//128 Kbytes  
#define ADDR_FLASH_SECTOR_10    ((uint32_t)0x080C0000) 	//128 Kbytes  
#define ADDR_FLASH_SECTOR_11    ((uint32_t)0x080E0000) 	//128 Kbytes  

uint32_t STMFLASH_ReadWord(uint32_t faddr);		  	
void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite);		
void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead);   		
void FLASH_write(void);
void FLASH_read(void);
#endif

















