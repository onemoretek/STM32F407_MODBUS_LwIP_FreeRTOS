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

int dev_rpt_udp_disc_ack_pkt(const ip_addr_t *addr, unsigned short port)
{
   int i;
   disc_rply_fmt_t disc_rply;
   unsigned int ip;
   unsigned int msk;
   unsigned int gw;

   ip = gnetif.ip_addr.addr; 
   msk = gnetif.netmask.addr;
   gw = gnetif.gw.addr;

   disc_rply.cmd = htonl(ugCmdDisc);
   disc_rply.dataLen = sizeof(dev_info_t);
   disc_rply.devinfo.sta = htonl(UG_SUCCESSED);
   disc_rply.devinfo.ip = htonl(ip);
   disc_rply.devinfo.msk = htonl(msk);
   disc_rply.devinfo.gw = htonl(gw);

   for (i = 0; i < 6; i++) 
      disc_rply.devinfo.mac[i] = gnetif.hwaddr[i];

   printf("dev_rpt_udp_disc_ack_pkt\r\n");
   
   udp_disc_client_raw_send_to((char *)&disc_rply, sizeof(disc_rply), addr, port);

   return sizeof(disc_rply);
}

