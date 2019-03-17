#include "lwip_comm.h" 
#include "netif/etharp.h"
#include "ethernetif.h" 
#include "timers.h"
#include "lwip/tcpip.h" 
#include "lwip/mem.h"
#include "lwip/dhcp.h"
#include "delay.h"
#include "usart.h"  
#include <stdio.h>

void lwip_dhcp_task(void *pdata)
{
  u32 ip = 0;
  u32 msk = 0;
  u32 gw = 0;

  while (1) {
    ip = gnetif.ip_addr.addr; 
    msk = gnetif.netmask.addr;
    gw = gnetif.gw.addr;
    
    //printf("dhcp is ongoing...\r\n");  
    
    if (ip != 0 && netif_is_link_up(&gnetif)) {
      printf("ip :%d.%d.%d.%d\r\n", 
        ip&0xff, (ip>>8)&0xff, (ip>>16)&0xff, (ip>>24)&0xff); 
      printf("msk:%d.%d.%d.%d\r\n", 
        msk&0xff, (msk>>8)&0xff, (msk>>16)&0xff, (msk>>24)&0xff); 
      printf("gw :%d.%d.%d.%d\r\n", 
        gw&0xff, (gw>>8)&0xff, (gw>>16)&0xff, (gw>>24)&0xff);
      
      MyDhcpTaskDestroy();
      break;
    }
  }
}

