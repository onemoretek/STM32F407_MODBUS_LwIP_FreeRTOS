#include <string.h>
#include <stdio.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"
#include "usart.h"
#include "dw_main.h"
//#include "modbus.h"
#include "stmflash.h"
#include "delay.h"
#define SIZE 100	 			  				
#define FLASH_SAVE_ADDR  0X08030000				//Parameter save start address ,must not overwithe the code area, large than( code +0X08000000)

int save_x[SIZE];							
//////////////////////////////////////////////////////////////////////////////////
extern uint8_t IP_ADDRESS[4];
extern uint8_t NETMASK_ADDRESS[4];
extern uint8_t GATEWAY_ADDRESS[4];


uint32_t STMFLASH_ReadWord(uint32_t faddr)
{
		return *(volatile uint32_t*)faddr; 
}  

uint32_t STMFLASH_GetFlashSector(uint32_t addr)
{
		if(addr<ADDR_FLASH_SECTOR_1)return FLASH_SECTOR_0;
		else if(addr<ADDR_FLASH_SECTOR_2)return FLASH_SECTOR_1;
		else if(addr<ADDR_FLASH_SECTOR_3)return FLASH_SECTOR_2;
		else if(addr<ADDR_FLASH_SECTOR_4)return FLASH_SECTOR_3;
		else if(addr<ADDR_FLASH_SECTOR_5)return FLASH_SECTOR_4;
		else if(addr<ADDR_FLASH_SECTOR_6)return FLASH_SECTOR_5;
		else if(addr<ADDR_FLASH_SECTOR_7)return FLASH_SECTOR_6;
		else if(addr<ADDR_FLASH_SECTOR_8)return FLASH_SECTOR_7;
		else if(addr<ADDR_FLASH_SECTOR_9)return FLASH_SECTOR_8;
		else if(addr<ADDR_FLASH_SECTOR_10)return FLASH_SECTOR_9;
		else if(addr<ADDR_FLASH_SECTOR_11)return FLASH_SECTOR_10; 
		return FLASH_SECTOR_11;	
}

void STMFLASH_Write(uint32_t WriteAddr,uint32_t *pBuffer,uint32_t NumToWrite)	
{ 
//  FLASH_Status status = FLASH_COMPLETE;
//	HAL_StatusTypeDef HAL_FLASH_Program(uint32_t TypeProgram, uint32_t Address, uint64_t Data)
//	HAL_StatusTypeDef HAL_FLASH_Unlock(void);
//  HAL_StatusTypeDef HAL_FLASH_Lock(void);
//  HAL_StatusTypeDef HAL_FLASHEx_Erase(FLASH_EraseInitTypeDef *pEraseInit, uint32_t *SectorError)	
	HAL_StatusTypeDef status  =  HAL_OK;
	uint32_t addrx = 0;
	uint32_t endaddr = 0;	
	uint32_t SectorError = 0;
  if(WriteAddr < STM32_FLASH_BASE || WriteAddr % 4 )return;	
	HAL_FLASH_Unlock();									
	__HAL_FLASH_INSTRUCTION_CACHE_DISABLE();
 		
	addrx = WriteAddr;				
	endaddr = WriteAddr + NumToWrite * 4;	
	if(addrx < 0X1FFF0000)			
	{  
//status=FLASH_EraseSector(STMFLASH_GetFlashSector(addrx),VoltageRange_3);//VCC=2.7~3.6V 
				 FLASH_EraseInitTypeDef EraseInit; 
//#define FLASH_TYPEERASE_SECTORS         0x00000000U  /*!< Sectors erase only          */
//#define FLASH_TYPEERASE_MASSERASE       0x00000001U  /*!< Flash Mass erase activation */
//#define FLASH_VOLTAGE_RANGE_1        0x00000000U  /*!< Device operating range: 1.8V to 2.1V                */
//#define FLASH_VOLTAGE_RANGE_2        0x00000001U  /*!< Device operating range: 2.1V to 2.7V                */
//#define FLASH_VOLTAGE_RANGE_3        0x00000002U  /*!< Device operating range: 2.7V to 3.6V                */
//#define FLASH_VOLTAGE_RANGE_4        0x00000003U  /*!< Device operating range: 2.7V to 3.6V + External Vpp */				
				 EraseInit.TypeErase = FLASH_TYPEERASE_SECTORS;
   			 EraseInit.Sector  = STMFLASH_GetFlashSector( addrx );
				 EraseInit.NbSectors = 1; 			  
				 EraseInit.VoltageRange = FLASH_VOLTAGE_RANGE_3;
				 status = HAL_FLASHEx_Erase(&EraseInit, &SectorError);
				 if ( status != HAL_OK )
				 { 
						 printf("Erase flash Error\r\n");
						 return;
				 }

	}
	if(status == HAL_OK)
	{
		while(WriteAddr < endaddr)
		{
//#define FLASH_TYPEPROGRAM_BYTE        0x00000000U  /*!< Program byte (8-bit) at a specified address           */
//#define FLASH_TYPEPROGRAM_HALFWORD    0x00000001U  /*!< Program a half-word (16-bit) at a specified address   */
//#define FLASH_TYPEPROGRAM_WORD        0x00000002U  /*!< Program a word (32-bit) at a specified address        */
//#define FLASH_TYPEPROGRAM_DOUBLEWORD  0x00000003U  /*!< Program a double word (64-bit) at a specified address */			
			if(HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,WriteAddr,*pBuffer) != HAL_OK)
			{ 
				break;	
			}
			WriteAddr+=4;
			pBuffer++;
		} 
	}
  
	__HAL_FLASH_INSTRUCTION_CACHE_ENABLE();
	HAL_FLASH_Lock();
} 


void STMFLASH_Read(uint32_t ReadAddr,uint32_t *pBuffer,uint32_t NumToRead)   	
{
	uint32_t i;
	for(i=0;i<NumToRead;i++)
	{
		pBuffer[i]=STMFLASH_ReadWord(ReadAddr);
		ReadAddr += 4;
	}
}


void FLASH_write(void)
{
	uint16 o,q;

	save_x[0]=Flash_Usart_BaudRate;
	save_x[1]=Flash_Modbus_ADDR;
	save_x[2]=Flash_structure_Mode;
	save_x[3]=Flash_Device_Mode;
	save_x[4]=Flash_Device_ID;
	for(q=0;q<3;q++)
	{
			save_x[5+q]=Flash_MAJOR_BS_X_Y_Z[q];
	}
	for(o=0;o<6;o++)
	{
			for(q=0;q<4;q++)
			{
				save_x[8+o*4+q]=Flash_BS_EN_X_Y_Z[o][q];
			}
	}
/*
* uint8_t IP_ADDRESS[4];
* uint8_t NETMASK_ADDRESS[4];
* uint8_t GATEWAY_ADDRESS[4];
*/	
	save_x[32]=IP_ADDRESS[0];
	save_x[33]=IP_ADDRESS[1];
	save_x[34]=IP_ADDRESS[2];
	save_x[35]=IP_ADDRESS[3];

	save_x[36]=NETMASK_ADDRESS[0];
	save_x[37]=NETMASK_ADDRESS[1];
	save_x[38]=NETMASK_ADDRESS[2];
	save_x[39]=NETMASK_ADDRESS[3];

	save_x[40]=GATEWAY_ADDRESS[0];
	save_x[41]=GATEWAY_ADDRESS[1];
	save_x[42]=GATEWAY_ADDRESS[2];
	save_x[43]=GATEWAY_ADDRESS[3];
						
	save_x[99]=Flash_FLAG;
	STMFLASH_Write(FLASH_SAVE_ADDR,(uint32_t *)save_x,204);
}

void FLASH_read(void)
{
	uint16 o,q;
	STMFLASH_Read(FLASH_SAVE_ADDR,(uint32_t*)save_x,204);
	
	Flash_Usart_BaudRate =  save_x[0];
	Flash_Modbus_ADDR    =  save_x[1];
	Flash_structure_Mode =  save_x[2];
	Flash_Device_Mode    =  save_x[3];
	Flash_Device_ID      =  save_x[4];
	for(q=0;q<3;q++)
	{
			Flash_MAJOR_BS_X_Y_Z[q] = save_x[5+q];
	}
	
	for(o=0;o<6;o++)
	{
			for(q=0;q<4;q++)
			{
				Flash_BS_EN_X_Y_Z[o][q] = save_x[8+o*4+q];
			}
	}
	IP_ADDRESS[0] = save_x[32];
	IP_ADDRESS[1] = save_x[33];
	IP_ADDRESS[2] = save_x[34];
	IP_ADDRESS[3] = save_x[35];


	NETMASK_ADDRESS[0] = save_x[36];
	NETMASK_ADDRESS[1] = save_x[37];
	NETMASK_ADDRESS[2] = save_x[38];	
	NETMASK_ADDRESS[3] = save_x[39];
	
	GATEWAY_ADDRESS[0] = save_x[40];
	GATEWAY_ADDRESS[1] = save_x[41];
	GATEWAY_ADDRESS[2] = save_x[42];
	GATEWAY_ADDRESS[3] = save_x[43];
		Flash_FLAG = save_x[99];

	if(Flash_FLAG != 0xab) 
	{
			Flash_FLAG=0xab;
			Flash_Usart_BaudRate = 0x0007;
			Flash_Modbus_ADDR    = 0x0001;
			Flash_Device_Mode    = 0x0000;
			Flash_Device_ID      = 0x0000;
			Flash_structure_Mode = 0x0000;
			for(q=0;q<3;q++)  {
				Flash_MAJOR_BS_X_Y_Z[q]=0x0000;
			}
	
			for(o=0;o<6;o++) {
					for(q=0;q<4;q++) {
							Flash_BS_EN_X_Y_Z[o][q]=0x0000;
					}
			}
			IP_ADDRESS[0] = 192;
			IP_ADDRESS[1] = 168;
			IP_ADDRESS[2] = 1;
			IP_ADDRESS[3] = 121;


			NETMASK_ADDRESS[0] = 255;
			NETMASK_ADDRESS[1] = 255;
			NETMASK_ADDRESS[2] = 255;	
			NETMASK_ADDRESS[3] = 0;
			
			GATEWAY_ADDRESS[0] = 192;
			GATEWAY_ADDRESS[1] = 168;
			GATEWAY_ADDRESS[2] = 1;
			GATEWAY_ADDRESS[3] = 1;
			FLASH_write();
	}		
}