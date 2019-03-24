/**
  *****************************************************************************
  * @file    device_rpt.h
  * @author  Zorb
  * @version V1.0.0
  * @date    2018-09-04
  * @brief   udp客户端的实现
  *****************************************************************************
  * @history
  *
  * 1. Date:2018-09-04
  *    Author:Zorb
  *    Modification:建立文件
  *
  *****************************************************************************
  */
#ifndef _DEVICE_RPT_H
#define _DEVICE_RPT_H

typedef enum {
    ugCmdDisc       = 1,
    ugCmdRpt        = 2,
    ugCmdDoUpgrade  = 3,
    ugCmdDonegrade  = 4,
    ugCmdQuerySta   = 5,
} ugCmdE;

//we should use net byte order
#pragma pack(1)
typedef struct {
   unsigned int sta;
   unsigned int ip;
   unsigned char mac[6];
   unsigned int msk;
   unsigned int gw;
} dev_info_t; 
#pragma pack()

int dev_rpt_udp_disc_ack_pkt(const ip_addr_t *addr, unsigned short port);

#endif //_DEVICE_RPT_

