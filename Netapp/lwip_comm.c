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

void lwip_dump_ip_info(u32 ip, u32 msk, u32 gw) 
{
   printf("ip :%d.%d.%d.%d\r\n", 
     ip&0xff, (ip>>8)&0xff, (ip>>16)&0xff, (ip>>24)&0xff); 
   printf("msk:%d.%d.%d.%d\r\n", 
     msk&0xff, (msk>>8)&0xff, (msk>>16)&0xff, (msk>>24)&0xff); 
   printf("gw :%d.%d.%d.%d\r\n", 
     gw&0xff, (gw>>8)&0xff, (gw>>16)&0xff, (gw>>24)&0xff);

   return ;
}

void lwip_dhcp_task(void *pdata)
{
  u32 ip = 0;
  u32 msk = 0;
  u32 gw = 0;
  u8_t hwaddr[NETIF_MAX_HWADDR_LEN];

  printf("Dhcp is ongoing...\r\n"); 
  
  while (1) {
    for (ip = 0; ip < NETIF_MAX_HWADDR_LEN; ip++) 
      hwaddr[ip] = gnetif.hwaddr[ip];
   
    ip = gnetif.ip_addr.addr; 
    msk = gnetif.netmask.addr;
    gw = gnetif.gw.addr;
    
    if (ip != 0 && netif_is_link_up(&gnetif)
      /* && dhcp_supplied_address(&gnetif)*/) {
      printf("...\r\n");
      printf("Mac:%02x:%02x:%02x:%02x:%02x:%02x\r\n", 
        hwaddr[0], hwaddr[1], hwaddr[2], hwaddr[3], hwaddr[4], hwaddr[5]); 
      printf("ip :%d.%d.%d.%d\r\n", 
        ip&0xff, (ip>>8)&0xff, (ip>>16)&0xff, (ip>>24)&0xff); 
      printf("msk:%d.%d.%d.%d\r\n", 
        msk&0xff, (msk>>8)&0xff, (msk>>16)&0xff, (msk>>24)&0xff); 
      printf("gw :%d.%d.%d.%d\r\n", 
        gw&0xff, (gw>>8)&0xff, (gw>>16)&0xff, (gw>>24)&0xff);
      MyDhcpTaskDestroy();
      break;
    } else {
      vTaskDelay(10000);
      printf("..."); 
    }
  }
}

