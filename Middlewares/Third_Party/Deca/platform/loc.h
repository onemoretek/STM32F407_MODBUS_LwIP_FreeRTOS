#ifndef __LOC_H
#define __LOC_H
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"

 uint8 PersonPosition(double A_x,double A_y,double A_r, 
														double B_x,double B_y,double B_r,uint16 B_en,
														double C_x,double C_y,double C_r,uint16 C_en,
														double D_x,double D_y,double D_r,uint16 D_en,
														double E_x,double E_y,double E_r,uint16 E_en,
														double F_x,double F_y,double F_r,uint16 F_en,
														double G_x,double G_y,double G_r,uint16 G_en,double *point_out);
#endif
