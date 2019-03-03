#ifndef __DELAY_H
#define __DELAY_H 			   
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"


void delay_init(uint8 SYSCLK);
void delay_ms(uint16 nms);
void delay_us(uint32 nus);

#endif





























