#ifndef __modbus_H
#define __modbus_H
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"

void MODBUS(unsigned char *buf,unsigned int length,unsigned int UsartNUM);
void MODBUS_ReadFromEEPROM(void);
void MODBUS_event(void);
void MODBUS_datain(void); 
void MODBUS_dataout(void); 
void MODBUS_xyz(uint16 ID,uint16 x,uint16 y,uint16 z,uint16 *dis_bb);
unsigned int CRC_Calculate(unsigned char *pdata,unsigned char num);
extern unsigned short modbus_reg[];



#endif	



