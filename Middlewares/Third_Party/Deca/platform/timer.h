#ifndef __TIMER_H
#define __TIMER_H
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"


void TIM4_Int_Init(uint16 arr,uint16 psc);
void TIM3_Int_Init(uint16 arr,uint16 psc);
void TIM2_Int_Init(uint16 arr,uint16 psc);
#endif
