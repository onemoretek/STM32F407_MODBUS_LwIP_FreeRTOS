#ifndef __USART_H
#define __USART_H
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "stdio.h"	

#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
//串口数据接收缓存长度
#define USART1_RX_LENGTH_MAX    360      //(15 * 8  * 3)  是各种modbus 的最小公倍数
#define USART2_RX_LENGTH_MAX    20
#define USART3_RX_LENGTH_MAX    200

extern unsigned char USART2_OV_FLAG;
extern unsigned char USART1_OV_FLAG;
extern unsigned char usart1_rx_buf[USART1_RX_LENGTH_MAX];//串口1的接收数据缓存
extern unsigned int usart1_rx_length;
extern unsigned char usart3_rx_buf[USART3_RX_LENGTH_MAX];
extern unsigned int usart3_rx_length;
extern unsigned char time3_usart1,time3_usart2,time3_usart3;

//如果想串口中断接收，请不要注释以下宏定义
void uart_init(unsigned long pclk2);
void U1SendChar(int ch);
void Usart1_SendString(unsigned char *data,unsigned int num);
void U3SendChar(int ch);   //串口3发送字符
void Usart3_Init(unsigned long pclk2); //串口3初始化
void Usart3_SendString(unsigned char *data,unsigned int num);//串口3发送字符串


#endif	


