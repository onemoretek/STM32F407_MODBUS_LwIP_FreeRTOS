/**
  *****************************************************************************
  * @file    device_rpt.c
  * @author  Zorb
  * @version V1.0.0
  * @date    2018-09-04
  * @brief   
  *****************************************************************************
  * @history
  *
  * 1. Date:2018-09-04
  *    Author:Zorb
  *    Modification:建立文件
  *
  *****************************************************************************
  */
#include "lwip.h"
#include "udp_client.h"
#include "device_rpt.h"


extern struct netif gnetif;

int dev_rpt_udp_disc_ack_pkt(void)
{
   int i;
   dev_info_t dev_info;
   unsigned int ip;
   unsigned int msk;
   unsigned int gw;

   ip = gnetif.ip_addr.addr; 
   msk = gnetif.netmask.addr;
   gw = gnetif.gw.addr;

   dev_info.sta = htonl(ugCmdDisc);
   dev_info.ip = htonl(ip);
   dev_info.msk = htonl(msk);
   dev_info.gw = htonl(gw);

   for (i = 0; i < 6; i++) 
      dev_info.mac[i] = gnetif.hwaddr[i];

   printf("dev_rpt_udp_disc_ack_pkt\r\n");
   
   udp_disc_client_raw_send((char *)&dev_info, sizeof(dev_info));

   return sizeof(dev_info);
}

