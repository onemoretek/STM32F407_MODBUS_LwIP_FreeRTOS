#ifndef _LWIP_COMM_H
#define _LWIP_COMM_H 

typedef unsigned int u32;

void lwip_dhcp_task(void *pdata); 
extern void MyDhcpTaskDestroy(void);
extern struct netif gnetif;

#endif

