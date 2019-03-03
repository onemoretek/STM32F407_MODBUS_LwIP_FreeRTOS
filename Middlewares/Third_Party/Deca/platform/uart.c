#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include <string.h>
#include <stdio.h>
#include "port.h"
#include "dw_main.h"
#include "uart.h"
#include "delay.h"
#if 0
unsigned char usart1_rx_buf[USART1_RX_LENGTH_MAX];//串口1的接收数据缓存
unsigned int usart1_rx_length=0;
unsigned char usart2_rx_buf[USART2_RX_LENGTH_MAX];//串口2的接收数据缓存
unsigned int usart2_rx_length=0;
unsigned char usart3_rx_buf[USART3_RX_LENGTH_MAX];
unsigned int usart3_rx_length=0;
unsigned char time3_usart1,time3_usart2,time3_usart3;//time3用于各串口接收的变量
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_UCOS
#include "includes.h"					//ucos 使用	  
#endif
 

//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  

#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
	/* Whatever you require here. If the only file you are using is */ 
	/* standard output using printf() for debugging, no file handling */ 
	/* is required. */ 
}; 
/* FILE is typedef’ d in stdio.h. */ 
FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)

{

//USART_SendData(USART1, (unsigned char) ch);// USART1 ???? USART2 ?

//while (!(USART1->SR & USART_FLAG_TXE));

return (ch);


}

//end
//////////////////////////////////////////////////////////////////

#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序

//void USART1_IRQHandler(void)
//{
//	
////		if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//		{
//				time3_usart1 = 0;
////        usart1_rx_buf[usart1_rx_length] = USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
//        if(usart1_rx_length < USART1_RX_LENGTH_MAX)
//            usart1_rx_length++;
// 
//	}

//} 
#endif			

//void USART3_IRQHandler()	//串口3中断
//{
//	
////	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
//		{
//		time3_usart3 = 0;
////        usart3_rx_buf[usart3_rx_length] = USART_ReceiveData(USART3);
//        if(usart3_rx_length < USART3_RX_LENGTH_MAX)
//            usart3_rx_length++; 
//		}
//	
//}

//初始化IO 串口1
//pclk2:PCLK2时钟频率(Mhz)
//bound:波特率
//CHECK OK
//091209
void uart_init(unsigned long pclk2)
{  	 
//USART_InitTypeDef USART_InitStructure;
//	GPIO_InitTypeDef GPIO_InitStructure;
//	NVIC_InitTypeDef NVIC_InitStructure;
	/* USARTx configured as follow:
		  - BaudRate = 115200 baud
		  - Word Length = 8 Bits
		  - One Stop Bit
		  - No parity
		  - Hardware flow control disabled (RTS and CTS signals)
		  - Receive and transmit enabled
	*/
//	USART_InitStructure.USART_BaudRate = pclk2 ;
//	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//	USART_InitStructure.USART_StopBits = USART_StopBits_1;
//	USART_InitStructure.USART_Parity = USART_Parity_No;
//	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;

//	/* Enable GPIO clock */
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);

	//For EVB1000 -> USART2_REMAP = 0

	/* Enable the USART2 Pins Software Remapping */

//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);


//	/* Configure USART Tx as alternate function push-pull */
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	/* Configure USART Rx as input floating */
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);

//	/* USART configuration */
//	USART_Init(USART1, &USART_InitStructure);
//	
//   NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
//	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
//	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
//	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
//	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
//	
//	 // Enable USART1 Receive interrupts 使能串口接收中断  
//   USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);  
//	/* Enable USART */
//	USART_Cmd(USART1, ENABLE);
	
	
	
	
	
}


void Usart3_Init(unsigned long pclk2) //串口3初始化
{
//	USART_InitTypeDef USART_InitStructure;  
//  NVIC_InitTypeDef NVIC_InitStructure;   
//    GPIO_InitTypeDef GPIO_InitStructure;    //声明一个结构体变量，用来初始化GPIO  
//   //使能串口的RCC时钟  
//   RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB , ENABLE); //使能UART3所在GPIOB的时钟  
//   RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);  
//  
//   //串口使用的GPIO口配置  
//   // Configure USART2 Rx (PB.11) as input floating    
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;  
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;  
//   GPIO_Init(GPIOB, &GPIO_InitStructure);  
//  
//   // Configure USART2 Tx (PB.10) as alternate function push-pull  
//   GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;  
//   GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;  
//   GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;  
//   GPIO_Init(GPIOB, &GPIO_InitStructure);  
//  
//   //配置串口  
//   USART_InitStructure.USART_BaudRate =pclk2;  
//   USART_InitStructure.USART_WordLength = USART_WordLength_8b;  
//   USART_InitStructure.USART_StopBits = USART_StopBits_1;  
//   USART_InitStructure.USART_Parity = USART_Parity_No;  
//   USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;  
//   USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;  
//  
//  
//   // Configure USART3   
//   USART_Init(USART3, &USART_InitStructure);//配置串口3  
//  
//	
//	//串口中断配置  
//   //Configure the NVIC Preemption Priority Bits     
//   NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);  
//  
//   // Enable the USART3 Interrupt   
//   NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;  
//   NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  
//   NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;  
//   NVIC_Init(&NVIC_InitStructure);  
//	
//	
//  // Enable USART1 Receive interrupts 使能串口接收中断  
//   USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);  
//   //串口发送中断在发送数据时开启  
//   //USART_ITConfig(USART2, USART_IT_TXE, ENABLE);  
//	
//	
//	
//   // Enable the USART3   
//   USART_Cmd(USART3, ENABLE);//使能串口3  
//  
//   
//      
      
}
void U3SendChar(int ch)   //串口3发送字符
{
	//while(!(USART2->SR & 0x00000040));
	//USART_SendData(USART2,c);
	/* e.g. write a character to the USART */
//	USART_SendData(USART3, ch);

	/* Loop until the end of transmission */
//	while (USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);
}

void U1SendChar(int ch) 
{
		//while(!(USART2->SR & 0x00000040));
	//USART_SendData(USART2,c);
	/* e.g. write a character to the USART */
	//USART_SendData(USART1, ch);

	/* Loop until the end of transmission */
//	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
}

void Usart1_SendString(unsigned char *data,unsigned int num)//串口1发送字符串
{
	unsigned int i;
	for(i = 0;i < num;i++)
	{
		U1SendChar(data[i]);
	}
}

void Usart3_SendString(unsigned char *data,unsigned int num)//串口3发送字符串
{
	unsigned int i;
	for(i = 0;i < num;i++)
	{
		U3SendChar(data[i]);
	}
}
#endif 



