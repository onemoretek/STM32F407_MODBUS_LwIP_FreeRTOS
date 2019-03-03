/*! ----------------------------------------------------------------------------
 *  @file    dw_main.c
 *  @brief   Double-sided two-way ranging (DS TWR) initiator example code
 *
 *           This is a simple code example which acts as the initiator in a DS TWR distance measurement exchange. This application sends a "poll"
 *           frame (recording the TX time-stamp of the poll), and then waits for a "response" message expected from the "DS TWR responder" example
 *           code (companion to this application). When the response is received its RX time-stamp is recorded and we send a "final" message to
 *           complete the exchange. The final message contains all the time-stamps recorded by this application, including the calculated/predicted TX
 *           time-stamp for the final message itself. The companion "DS TWR responder" example application works out the time-of-flight over-the-air
 *           and, thus, the estimated distance between the two devices.
 *这是一个简单的代码示例，它作为SS TWR距离测量交换机中的应答器。此应用程序等待“投票”。
*消息（记录投票的RX时间戳）期望从“SS TWR发起者”示例代码（与此应用程序配套），以及
*然后发送一个“响应”消息来完成交换。响应消息包含该应用程序记录的所有时间戳，
*包括响应消息本身的计算/预测的TX时间戳。同伴“SS TWR发起者”示例应用
*计算出飞行时间在空中，因此，估计的距离之间的两个设备。
 * @attention
 *
 * Copyright 2015 (c) Decawave Ltd, Dublin, Ireland.
 *
 * All rights reserved.
 *
 * @author Decawave
 */
 
 /*! ----------------------------------------------------------------------------
 *本程序基于官方提供程序修改！
 *支持一对一测距，多基站多标签二维定位
 * @author 广州联网科技有限公司
 */
 
#include <string.h>
#include <stdio.h>
#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"
#include "usart.h"
#include "stmflash.h"
#include "dw_main.h"
#include "delay.h"
#include "modbus.h"
#include "timer.h"
#include "loc.h"

#define BOARD_NAME	"STM32F407ZGT6-DEV"
#define NETWORK_CARD "LAN8742A_PHY"
#define SYSTEM_VER	"1.0.0"
typedef signed long long int64;
typedef unsigned long long uint64;
typedef unsigned long long uint64;

/******************************************************************************
														内存FLASH区数据
*******************************************************************************/
uint16   Flash_Usart_BaudRate;       //设备串口通讯波特率 0：4800  1：9600 2：14400 3：19200 4：38400 5：56000 6：57600 7：115200  8：128000 9：256000
uint16   Flash_Modbus_ADDR;        	//Modbus ID号 
uint16   Flash_structure_Mode;     	//0:为二维平面3基站模式
uint16   Flash_Device_Mode;       		//设备模式 0：标签 1：次基站 2：主基站
uint16   Flash_Device_ID;        	  //高8位为次基站ID，范围0~6  低8位为标签ID 0~247    （程序内部 标签ID为0~247  次基站ID为248~245  主基站ID为255）
uint16   Flash_MAJOR_BS_X_Y_Z[3];    //6次基站的位置，      X,Y,Z各两字节
uint16   Flash_BS_EN_X_Y_Z[6][4];    //6次基站的位置，    使能标志：0为关  1为开   X,Y,Z各两字节
uint16   Flash_FLAG;	 								//标志位

/******************************************************************************
														 计算使用变量
*******************************************************************************/
uint16   Calculate_EN;       			//测量使能
uint16   Calculate_TAG_ID;					//测量标签ID
uint16		Calculate_FLAG;						//测量状态标志
uint16		Calculate_TAG_X;					//测量标签的X轴
uint16		Calculate_TAG_Y;					//测量标签的Y轴
uint16		Calculate_TAG_Z;     			//测量标签的Z轴
uint16		Calculate_Station_TAG[7]; //测量标签的距离


/******************************************************************************
														系统流程循环标志位
*******************************************************************************/
uint8 SYS_Calculate_ACTIVE_FLAG=0;   				//系统循环标志位主动测距函数
uint8 SYS_Calculate_PASSIVE_FLAG=0;   				//系统循环标志位被动测距函数
uint8 SYS_BS_FLAG=0;    				//系统循环标志位-次基站
uint8 SYS_MAJOR_BS_FLAG=0;     //系统循环标志位-主基站
uint8 SYS_BS_TAG_FLAG=0;       //次基站收到需要测距标签的ID
uint8 SYS_BS_MESSAGE_FLAG=0;       //主基站联系次基站标志位
uint8 SYS_BS_MESSAGE_Timer_FLAG=0;       //主基站等待次基站回复允许错误数据次数缓存区

/******************************************************************************
														系统流程自定义
*******************************************************************************/
#define KALMAN_Q 3        	  //卡尔曼滤波-Q
#define KALMAN_R 10						//卡尔曼滤波-R
uint16 Time_time4_Cuo=0;   			 //记录时间戳-测试用
uint16 Time_time4_Cuo_buf[10];   //记录时间戳缓存组-测试用
uint32 MODBUS_BaudRate[10]={4800,9600,14400,19200,38400,56000,57600,115200,128000,256000};    //波特率列表
int16 SYS_dis_buf_t[7];  	//主基站获取到的距离数据缓存区
uint16 TAG_ID;   							//测距标签ID
uint16 ERROR_FLAG;  						//测距错误计算次数标志位，达到一定次数跳出

uint16 LED_FLAG=0;     		//系统指示灯记录标志位


/******************************************************************************
														DWM1000测距计算变量
*******************************************************************************/
// DWM1000通讯数据包                 A_ID  B_ID  帧    命令  
static uint8 Send_get_dist_msg[] = {0X00, 0x00, 0x00, 0xCA,'1','1','1','1','2','2','2','2','3','3','3','3','4','4','4','4','5','5','5','5','6','6','6','6','D','D'};
	
#define RX_BUF_LEN 30               //DWM1000接收数据包长度
static uint8 rx_buffer[RX_BUF_LEN]; //DWM1000接收数据包缓存区
	
static int16_t dist[20];    //LP低通滤波缓存
static double tof;             //光速
static double distance,dist2;  //理论距离 ，估算距离
int32_t dis;             		//测距距离缓存   
uint32 frame_len;        		//DWM1000收发数据包长度缓存
static uint32 Time_ts[6];  					//飞行时间缓存记录
	
/* Frame sequence number, incremented after each transmission. 帧序列号，每次传输后递增。 */
static uint32 frame_seq_nb = 0;

/* Hold copy of status register state here for reference, so reader can examine it at a breakpoint. 
在这里保存状态寄存器状态的副本以供参考，以便读者可以在断点上检查它。*/
static uint32 status_reg = 0;

/* UWB microsecond (uus) to device time unit (dtu, around 15.65 ps) conversion factor.
UWB微秒（UUS）到设备时间单位（DTU，约15.65 ps）的转换系数。
 * 1 uus = 512 / 499.2 祍 and 1 祍 = 499.2 * 128 dtu. 
  1 UUS＝512／499.2，1＝499.2×128 DTU。*/
#define UUS_TO_DWT_TIME 65536
#define SPEED_OF_LIGHT 299702547

/******************************************************************************
														静态函数声明
*******************************************************************************/
static uint64 get_tx_timestamp_u64(void);
static uint64 get_rx_timestamp_u64(void);
static void final_msg_set_ts(uint8 *ts_field, uint64 ts);
static void final_msg_set_dist(uint8 *ts_field, uint32 dist);

static void System_Info(void)
{
	printf("+------------------------------------------+\r\n");
	printf("|          FreeRTOS+LwIP System            |\r\n");
	printf("|     Copyright(c)2019 Onemore Tek..     |\r\n");
	printf("|           All rights reserved.           |\r\n");
	printf("+------------------------------------------+\r\n");
	printf(" Product: %s\r\n", BOARD_NAME);
	printf(" Network Card:%s\r\n",NETWORK_CARD);
	printf(" Version: %s\r\n", SYSTEM_VER);
	printf(" Release: %s %s\r\n", __DATE__, __TIME__);
	printf("+------------------------------------------+\r\n");
}
/******************************************************************************
												    基站呼叫标签并进行测距
*******************************************************************************/
int32_t DW1000send(uint8_t A_ID,uint8_t B_ID) //主动模式
{
		   
			  uint32 i;
				if(SYS_Calculate_ACTIVE_FLAG==0)
				{
						for(i=0;i<30;i++)
						{
								Send_get_dist_msg[i]=0x00;
						}
						Send_get_dist_msg[0] =  A_ID;	//UWB POLL 包数据
						Send_get_dist_msg[1] =  B_ID;//UWB Fianl 包数据
						Send_get_dist_msg[2] = frame_seq_nb;
						Send_get_dist_msg[3]=0XAB; 						
						dwt_writetxdata(sizeof(Send_get_dist_msg), Send_get_dist_msg, 0);//将Poll包数据传给DW1000，将在开启发送时传出去
						dwt_writetxfctrl(sizeof(Send_get_dist_msg), 0);//设置超宽带发送数据长度
						dwt_setrxaftertxdelay(0);
						dwt_setrxtimeout(5000);						//设置接收超时时间
						dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);//开启发送，发送完成后等待一段时间开启接收，等待时间在dwt_setrxaftertxdelay中设置;
						SYS_Calculate_ACTIVE_FLAG=1;
				}			
				if(SYS_Calculate_ACTIVE_FLAG==1)
				{			
						if((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))//不断查询芯片状态直到成功接收或者发生错误
						{
							SYS_Calculate_ACTIVE_FLAG=2;
						}
						else return 0;
				}
				if(SYS_Calculate_ACTIVE_FLAG==2)
				{
						if(frame_seq_nb<0xFFFFFFFF)frame_seq_nb++;
						else frame_seq_nb=0;
						if (status_reg & SYS_STATUS_RXFCG)//如果成功接收
						{									
								SYS_Calculate_ACTIVE_FLAG=3;
						}
						else 
						{
							 
								dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);		
							SYS_Calculate_ACTIVE_FLAG=0;
							  return	0;	
						}
				}
								
        if(SYS_Calculate_ACTIVE_FLAG==3)
				{
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//清楚寄存器标志位
            frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;	//获得接收到的数据长度
            dwt_readrxdata(rx_buffer, frame_len, 0);   //读取接收数据
            
						if (rx_buffer[3]==0xBC)//判断接收到的数据是否是response数据
            {  
								SYS_Calculate_ACTIVE_FLAG=4;
						}
						else 
						{
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);	
							SYS_Calculate_ACTIVE_FLAG=0;
							return 0;							
						}		
				}
				if(SYS_Calculate_ACTIVE_FLAG==4)
				{
							  for(i=0;i<30;i++)
								{
									 Send_get_dist_msg[i]=rx_buffer[i];
								}								
                Time_ts[0] = get_tx_timestamp_u64();										//获得POLL发送时间T1
                Time_ts[3] = get_rx_timestamp_u64();										//获得RESPONSE接收时间T4
                final_msg_set_ts(&Send_get_dist_msg[4],Time_ts[0]);//将T1写入发送数据
                final_msg_set_ts(&Send_get_dist_msg[16],Time_ts[3]);//将T4写入发送数据
							
							  Send_get_dist_msg[3]=0XCD; 
                Send_get_dist_msg[2] = frame_seq_nb;
                dwt_writetxdata(sizeof(Send_get_dist_msg), Send_get_dist_msg, 0);//将发送数据写入DW1000
                dwt_writetxfctrl(sizeof(Send_get_dist_msg), 0);//设定发送数据长度
								dwt_setrxaftertxdelay(0);
								dwt_setrxtimeout(5000);						//设置接收超时时间
                dwt_starttx(DWT_START_TX_IMMEDIATE| DWT_RESPONSE_EXPECTED);//设定为延迟发送
								SYS_Calculate_ACTIVE_FLAG=5;						
					}
				
					if(SYS_Calculate_ACTIVE_FLAG==5)
					{
								if ((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))//不断查询芯片状态直到成功接收或者发生错误
								{ 
									SYS_Calculate_ACTIVE_FLAG=6;
								}
								else return 0;
					}
					if(SYS_Calculate_ACTIVE_FLAG==6)
					{
								
							if(frame_seq_nb<0xFFFFFFFF)frame_seq_nb++;
							else frame_seq_nb=0;
							if (status_reg & SYS_STATUS_RXFCG)//如果成功接收
							{	
									SYS_Calculate_ACTIVE_FLAG=7;	
							}
							else 
							{
									
									dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);	
									SYS_Calculate_ACTIVE_FLAG=0;								
									return	0;								
							}
					}
					if(SYS_Calculate_ACTIVE_FLAG==7)
					{
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//清楚寄存器标志位
							frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;	//获得接收到的数据长度
							dwt_readrxdata(rx_buffer, frame_len, 0);   //读取接收数据						
							if (rx_buffer[3]==0xDE)//判断接收到的数据是否是response数据
							{
									SYS_Calculate_ACTIVE_FLAG=8;
							}
							else 
							{
									dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);	
									SYS_Calculate_ACTIVE_FLAG=0;
									return 0;
							}
					}
					if(SYS_Calculate_ACTIVE_FLAG==8)
					{
							uint32 Time_ts_F[6];
							double Ra, Rb, Da, Db;
               int64 tof_dtu;
							 for(i=0;i<30;i++)
							{								
									Send_get_dist_msg[i]=rx_buffer[i];
							}
							final_msg_get_ts(&rx_buffer[4], &Time_ts_F[0]);
							final_msg_get_ts(&rx_buffer[8], &Time_ts_F[1]);
							final_msg_get_ts(&rx_buffer[12], &Time_ts_F[2]);
							final_msg_get_ts(&rx_buffer[16], &Time_ts_F[3]);
							final_msg_get_ts(&rx_buffer[24], &Time_ts_F[5]);									
							Time_ts[0]= (uint32)Time_ts_F[0];
							Time_ts[1]= (uint32)Time_ts_F[1];
							Time_ts[2]= (uint32)Time_ts_F[2];
							Time_ts[3]= (uint32)Time_ts_F[3];
							Time_ts[4]= (uint32)get_tx_timestamp_u64();	
							Time_ts[5]= (uint32)Time_ts_F[5];

							Ra = (double)(Time_ts[3] - Time_ts[0]);//Tround1 = T4 - T1  
              Rb = (double)(Time_ts[5] - Time_ts[2]);//Tround2 = T6 - T3 
              Da = (double)(Time_ts[4] - Time_ts[3]);//Treply2 = T5 - T4  
              Db = (double)(Time_ts[2] - Time_ts[1]);//Treply1 = T3 - T2  
              tof_dtu = (int64)((Ra * Rb - Da * Db) / (Ra + Rb + Da + Db));//计算公式
              tof = tof_dtu * DWT_TIME_UNITS;
              distance = tof * SPEED_OF_LIGHT;//距离=光速*飞行时间
							dist2 = distance - dwt_getrangebias(config.chan,(float)distance, config.prf);//距离减去矫正系数	
							dis = dist2*100;//dis 为单位为cm的距离		
							SYS_Calculate_ACTIVE_FLAG=0;
							return dis;			
					}				
					return 0;   
}



/******************************************************************************
												    标签应答基站并进行测距
*******************************************************************************/
int32_t DW1000receive(uint8_t B_ID) //被动
{
				uint32 i;
	      if(SYS_Calculate_PASSIVE_FLAG==0)  //打开接收
				{
						dwt_setrxtimeout(0);//设定接收超时时间，0位没有超时时间
						dwt_rxenable(0);//打开接收
						SYS_Calculate_PASSIVE_FLAG=1;
				}
				if(SYS_Calculate_PASSIVE_FLAG==1)  //等待接收
				{
						if((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))//不断查询芯片状态直到接收成功或者出现错误
						{
							SYS_Calculate_PASSIVE_FLAG=2;
						}
						else return 0;
				}
			  
				if(SYS_Calculate_PASSIVE_FLAG==2)  //验证是否成功接收
				{
						if (status_reg & SYS_STATUS_RXFCG)//成功接收
						{
								SYS_Calculate_PASSIVE_FLAG=3;
						}
						else
						{
								/* Clear RX error events in the DW1000 status register. */
								dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
								SYS_Calculate_PASSIVE_FLAG=0;							
								return 0;
						}
				}
				if(SYS_Calculate_PASSIVE_FLAG==3)   //判断是否为有效数据包
				{
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);//清楚标志位
							frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;//获得接收数据长度
							dwt_readrxdata(rx_buffer, frame_len, 0);//读取接收数据
							if (rx_buffer[3]==0xAB&&B_ID==rx_buffer[1])//判断数据
							{       
									SYS_Calculate_PASSIVE_FLAG=4;
							}
							else 
							{	
								  SYS_Calculate_PASSIVE_FLAG=0;
									return 0;
							}
					}
					if(SYS_Calculate_PASSIVE_FLAG==4)  //发送数据后打开接收
					{
							for(i=0;i<30;i++)
							{
									Send_get_dist_msg[i]=rx_buffer[i];
							}							
							Time_ts[1] = get_rx_timestamp_u64();//获得Poll包接收时间T2
							final_msg_set_ts(&Send_get_dist_msg[8],Time_ts[1]);//将T2写入发送数据
							Send_get_dist_msg[2] = frame_seq_nb;
							Send_get_dist_msg[3]=0XBC; 
							dwt_writetxdata(sizeof(Send_get_dist_msg), Send_get_dist_msg, 0);//写入发送数据
							dwt_writetxfctrl(sizeof(Send_get_dist_msg), 0);//设定发送长度
							dwt_setrxaftertxdelay(0);//设置发送后开启接收，并设定延迟时间
							dwt_setrxtimeout(5000);						//设置接收超时时间
							dwt_starttx(DWT_START_TX_IMMEDIATE | DWT_RESPONSE_EXPECTED);//立即发送，等待接收      
							SYS_Calculate_PASSIVE_FLAG=5;
					}							
					if(SYS_Calculate_PASSIVE_FLAG==5)  //等待接收
					{
								if ((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))///不断查询芯片状态直到接收成功或者出现错误
								{ 
										SYS_Calculate_PASSIVE_FLAG=6;
								}
								else return 0;
					}
					if(SYS_Calculate_PASSIVE_FLAG==6)  //验证是否成功接收
					{							
							if(frame_seq_nb<0xFFFFFFFF)frame_seq_nb++;
							else frame_seq_nb=0;
							if (status_reg & SYS_STATUS_RXFCG)//接收成功
							{
									SYS_Calculate_PASSIVE_FLAG=7;
							}
							else
							{
                   /* Clear RX error events in the DW1000 status register. */
                   dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);			
									SYS_Calculate_PASSIVE_FLAG=0;
									return 0;
							}
					}
					if(SYS_Calculate_PASSIVE_FLAG==7)  //判断是否为有效数据
					{															
							dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//清楚标志位
							frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;//数据长度
							dwt_readrxdata(rx_buffer, frame_len, 0);//读取接收数据
							if (rx_buffer[3]==0xCD&&B_ID==rx_buffer[1])//判断是否为Fianl包
							{
								SYS_Calculate_PASSIVE_FLAG=8;
							}
							else 
							{
								SYS_Calculate_PASSIVE_FLAG=0;
								return 0;
							}
					}						
					if(SYS_Calculate_PASSIVE_FLAG==8)  //返回数据
					{						
							for(i=0;i<30;i++)
							{
									Send_get_dist_msg[i]=rx_buffer[i];
							}
												
              /* Retrieve response transmission and final reception timestamps. */
							Send_get_dist_msg[3]=0XDE; 
							Time_ts[2] = get_tx_timestamp_u64();//获得response发送时间T3
							Time_ts[5] = get_rx_timestamp_u64();//获得final接收时间T6
							final_msg_set_ts(&Send_get_dist_msg[12],Time_ts[2]);//将T3写入发送数据
							final_msg_set_ts(&Send_get_dist_msg[24],Time_ts[5]);//将T6写入发送数据
							dwt_writetxdata(sizeof(Send_get_dist_msg), Send_get_dist_msg, 0);//写入发送数据
							dwt_writetxfctrl(sizeof(Send_get_dist_msg), 0);//设定发送长度
							dwt_starttx(DWT_START_TX_IMMEDIATE );//设定为延迟发送
							SYS_Calculate_PASSIVE_FLAG=9;
					}						
					if(SYS_Calculate_PASSIVE_FLAG==9)  //验证是否发送完成
					{						
							if (dwt_read32bitreg(SYS_STATUS_ID) & SYS_STATUS_TXFRS)//不断查询芯片状态直到发送完成
							{ 
									dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_TXFRS);//清楚标志位										
									SYS_Calculate_PASSIVE_FLAG=0;
//									GPIO_Toggle(LED_GPIO,LED);
									return 1;
							}
							else return 0;
					 }									
				 return 0;	
}


/******************************************************************************
												    主基站通讯次基站进行测距并等待接收数据
*******************************************************************************/
int32_t DW1000send_dist_msg(uint8_t A_ID,uint8_t B_ID,uint8_t C_ID) //通讯次基站测距
{
				uint32 i=0;
		   if(SYS_BS_MESSAGE_FLAG==0)
			 {
				
						SYS_BS_MESSAGE_Timer_FLAG=0;//标志位清零
						for(i=0;i<30;i++)
						{
								Send_get_dist_msg[i]=0x00;
						}
						Send_get_dist_msg[0] =  A_ID;	//UWB POLL 包数据
						Send_get_dist_msg[1] =  B_ID;//UWB Fianl 包数据
						Send_get_dist_msg[2] = frame_seq_nb;
						Send_get_dist_msg[3]=0XEF; 
						Send_get_dist_msg[4]=C_ID;
						dwt_writetxdata(sizeof(Send_get_dist_msg), Send_get_dist_msg, 0);//将Poll包数据传给DW1000，将在开启发送时传出去
						dwt_writetxfctrl(sizeof(Send_get_dist_msg), 0);//设置超宽带发送数据长度
						dwt_setrxaftertxdelay(0);
						dwt_starttx(DWT_START_TX_IMMEDIATE);//开启发送
						dwt_setrxtimeout(6000);//设定接收超时时间，0位没有超时时间				
						deca_sleep(7);//休眠固定时间	
						SYS_BS_MESSAGE_FLAG=1;					
			 }
			 if(SYS_BS_MESSAGE_FLAG==1)
			 {
					dwt_rxenable(0);//打开接收
				  SYS_BS_MESSAGE_FLAG=2;
			 }
			 if(SYS_BS_MESSAGE_FLAG==2)
			 {				
				  
						if((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))//不断查询芯片状态直到成功接收或者发生错误
						{
							
								SYS_BS_MESSAGE_FLAG=3;							
						}	
						else return 0;
				}
			 if(SYS_BS_MESSAGE_FLAG==3)
			 {
						if(frame_seq_nb<0xFFFFFFFF)frame_seq_nb++;
						else frame_seq_nb=0;
						if (status_reg & SYS_STATUS_RXFCG)//如果成功接收
						{
							SYS_BS_MESSAGE_FLAG=4;
						}
						else 
						{
								dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);		
								if(SYS_BS_MESSAGE_Timer_FLAG>7)  //错误数据少于7次重新接收，大于7次重新通讯测距
								{
										SYS_BS_MESSAGE_FLAG=0;	
								}
								else
								{
										SYS_BS_MESSAGE_FLAG=1;	//如果错误就重新接收
										SYS_BS_MESSAGE_Timer_FLAG++;
								}						
								//SYS_BS_MESSAGE_FLAG=0;//不初始化 因为等待回信过程，有测距程序会收到无效数据包
								return 0;
						}
				}
			  if(SYS_BS_MESSAGE_FLAG==4)
				{	
								dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG | SYS_STATUS_TXFRS);//清楚寄存器标志位
								frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFLEN_MASK;	//获得接收到的数据长度
								dwt_readrxdata(rx_buffer, frame_len, 0);   //读取接收数据
								if (rx_buffer[3]==0xFF)//判断接收到的数据是否是response数据
								{     
								
										SYS_BS_MESSAGE_FLAG=5;									
								}
								else 
								{
										dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);		
									//SYS_BS_MESSAGE_FLAG=0;//不初始化 因为等待回信过程，有测距程序会收到无效数据包											
										if(SYS_BS_MESSAGE_Timer_FLAG>7)  //错误数据少于7次重新接收，大于7次重新通讯测距
										{
												SYS_BS_MESSAGE_FLAG=0;	
										}
										else
										{
											  SYS_BS_MESSAGE_FLAG=1;	//如果错误就重新接收
												SYS_BS_MESSAGE_Timer_FLAG++;
										}
										return 0;
								}
				}
				if(SYS_BS_MESSAGE_FLAG==5)
				{
							uint32 rec_dist;
							final_msg_get_ts(&rx_buffer[4], &rec_dist);
							SYS_BS_MESSAGE_FLAG=0;
							return rec_dist;
				}
				return 0;
			
}


/******************************************************************************
												    次基站应答主基站进行测距并返回数据
*******************************************************************************/
int32_t DW1000send_dist_msg_last(uint8_t A_ID,uint8_t B_ID,uint32_t dist) //次基站回应主基站
{
		   
			  uint32 i;
				for(i=0;i<30;i++)
				{
						Send_get_dist_msg[i]=0x00;
				}
				Send_get_dist_msg[0] =  A_ID;	//UWB POLL 包数据
				Send_get_dist_msg[1] =  B_ID;//UWB Fianl 包数据
			  Send_get_dist_msg[2] = frame_seq_nb;
			  Send_get_dist_msg[3]=0XFF; 
				final_msg_set_dist(&Send_get_dist_msg[4], dist);
        dwt_writetxdata(sizeof(Send_get_dist_msg), Send_get_dist_msg, 0);//将Poll包数据传给DW1000，将在开启发送时传出去
        dwt_writetxfctrl(sizeof(Send_get_dist_msg), 0);//设置超宽带发送数据长度
				dwt_setrxaftertxdelay(0);
				dwt_setrxtimeout(5000);						//设置接收超时时间
        dwt_starttx(DWT_START_TX_IMMEDIATE);//开启发送   				
				return 0;
			
}


/******************************************************************************
												    次基站等待主基站下达测距命令信号
*******************************************************************************/
uint8_t DW1000rec_dist_msg(uint8_t B_ID) //次基站等待信号
{
				
		if(SYS_BS_FLAG==0)
		{
        dwt_setrxtimeout(0);//设定接收超时时间，0位没有超时时间
        dwt_rxenable(0);//打开接收
				SYS_BS_FLAG=1;
		}
		if(SYS_BS_FLAG==1)
		{
			
        if((status_reg = dwt_read32bitreg(SYS_STATUS_ID)) & (SYS_STATUS_RXFCG | SYS_STATUS_ALL_RX_ERR))//不断查询芯片状态直到接收成功或者出现错误
        { 
					SYS_BS_FLAG=2;
				}
				else return 0;
			
		}		
		if(SYS_BS_FLAG==2)
		{
        if (status_reg & SYS_STATUS_RXFCG)//成功接收
        {
						SYS_BS_FLAG=3;	 
				}
        else
        {
            /* Clear RX error events in the DW1000 status register. */
            dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_ALL_RX_ERR);
					SYS_BS_FLAG=0;
					return 0;
        }
		}
		if(SYS_BS_FLAG==3)
		{
        dwt_write32bitreg(SYS_STATUS_ID, SYS_STATUS_RXFCG);//清楚标志位    
        frame_len = dwt_read32bitreg(RX_FINFO_ID) & RX_FINFO_RXFL_MASK_1023;//获得接收数据长度
				dwt_readrxdata(rx_buffer, frame_len, 0);//读取接收数据	
        if (rx_buffer[3]==0xEF&&B_ID==rx_buffer[1])//判断
        {
						SYS_BS_FLAG=4;
						return rx_buffer[4];
         }
				else 
				{
						SYS_BS_FLAG=0;
						return 0;
				}	
		}
		return 0;		
}

/******************************************************************************
												         初始化函数
*******************************************************************************/
void dw_init(void)
{
	  System_Info();
    peripherals_init();//初始化外设
//	  FLASH_read();
//    reset_DW1000();//重启DW1000 /* Target specific drive of RSTn line into DW1000 low for a period. */
//    spi_set_rate_low();//降低SPI频率
	  port_set_dw1000_slowrate();  //max SPI before PLLs configured is ~4M
    dwt_initialise(DWT_LOADUCODE);//初始化DW1000
    //spi_set_rate_high();//回复SPI频率
	  port_set_dw1000_fastrate();  //max SPI before PLLs configured is ~4M
	  
	  printf("dw1000local.deviceID = 0x%x\r\n",dwt_readdevid() );
    dwt_configure(&config);//配置DW1000

    dwt_setrxantennadelay(RX_ANT_DLY);		//设置接收天线延迟
    dwt_settxantennadelay(TX_ANT_DLY);		//设置发射天线延迟	
}
/******************************************************************************
												         主函数
*******************************************************************************/
int dw_main(void)
{

		{	
			//	MODBUS_event();
				switch(Flash_Device_Mode)
				{
						case 0:  //标签
						{			
								int32_t dis_buf=0;							
								dis_buf=DW1000receive(Flash_Device_ID&0x00FF);
								if(dis_buf!=0)
								{
										if(LED_FLAG>6)
										{
//												GPIO_Toggle(LED_GPIO,LED);
												LED_FLAG=0;
										}
										else LED_FLAG++;
								}
						}	
						break;
						
						case 1:  //次基站
						{								
								if(SYS_BS_FLAG<4) 
								{
									SYS_BS_TAG_FLAG=DW1000rec_dist_msg(((Flash_Device_ID>>8)&0xFF)+248);
							  }
								if(SYS_BS_FLAG==4)
								{
									int32_t dis_buf=0;
									dis_buf=DW1000send(((Flash_Device_ID>>8)&0xFF)+248,SYS_BS_TAG_FLAG);							
									if(dis_buf!=0)
									{
											DW1000send_dist_msg_last(((Flash_Device_ID>>8)&0xFF)+248,255,dis_buf);	//发送距离
										  if(LED_FLAG>6)
											{
//													GPIO_Toggle(LED_GPIO,LED);
													LED_FLAG=0;
											}
											else LED_FLAG++;
											SYS_BS_FLAG=0;	
									}
								}
						}						
						break;
						
						case 2:  //主基站
						{

							if(Flash_structure_Mode==0) //测距模式
							{
								int8_t o=0;
								if((SYS_MAJOR_BS_FLAG!=0)&&(Calculate_TAG_ID != TAG_ID))//检测过程中标签ID被修改就取消检测重新开始  防止检测过程被修改
									{
											SYS_MAJOR_BS_FLAG=0;							
									}
									if(SYS_MAJOR_BS_FLAG==0)
									{
										
										//Time_time4_Cuo_buf[SYS_MAJOR_BS_FLAG]=Time_time4_Cuo;  //测距时间戳记录-测试用
											if(Calculate_EN==0x01||Calculate_EN==0x02||Calculate_EN==0x03||Calculate_EN==0x04)		
											{
													TAG_ID=Calculate_TAG_ID;
																							
													SYS_MAJOR_BS_FLAG=1;	
													Calculate_FLAG=0;                                    //状态标志为正常
											}										
									}
									if(SYS_MAJOR_BS_FLAG==1) //主基站与标签测距
									{										   
										//Time_time4_Cuo_buf[SYS_MAJOR_BS_FLAG]=Time_time4_Cuo; //测距时间戳记录-测试用
										  Calculate_FLAG=1;                                    //状态标志为测距中						
											SYS_dis_buf_t[0]=DW1000send(255,TAG_ID);		//填入寄存器MODBUS														
											if(SYS_dis_buf_t[0]!=0)
											{
													SYS_MAJOR_BS_FLAG=2;
												
													//deca_sleep(RNG_DELAY_MS);//休眠固定时间
												  Calculate_FLAG=0;                                    //状态标志为正常
											}
										
									}		
									if(SYS_MAJOR_BS_FLAG==2)
									{									
									dist[0] = LP(SYS_dis_buf_t[0],0);//LP 为低通滤波器，让数据更稳定		
									
										Calculate_TAG_X=0; //赋予到寄存器
										Calculate_TAG_Y=0; //赋予到寄存器
										Calculate_TAG_Z=0; //赋予到寄存器		
									
										Calculate_Station_TAG[0]=dist[0];//赋予A基站测距值
											for(o=0;o<6;o++)   //赋予B~G基站测距值
											{
												
													Calculate_Station_TAG[o+1]=0;																								
											}									
											
											if(Calculate_EN==0x03||Calculate_EN==0x04)//自动输出
											{
												MODBUS_xyz(TAG_ID,Calculate_TAG_X,Calculate_TAG_Y,Calculate_TAG_Z,Calculate_Station_TAG);									
											}
											
											if(Calculate_EN==0x01||Calculate_EN==0x03)//单次检测后清零使能
											{												
												Calculate_EN=0;
											}	
											
												if(LED_FLAG>6)
											{
//													GPIO_Toggle(LED_GPIO,LED);
													LED_FLAG=0;
											}
											else LED_FLAG++;
											SYS_MAJOR_BS_FLAG=0;
										}
							}						
								if(Flash_structure_Mode==1)//二维定位模式
							 {									
									int8_t o=0,i=0;								  
								  if((SYS_MAJOR_BS_FLAG!=0)&&(Calculate_TAG_ID != TAG_ID))//检测过程中标签ID被修改就取消检测重新开始  防止检测过程被修改
									{
											SYS_MAJOR_BS_FLAG=0;													
									}
									/*****************************************
													         等待指令
									*****************************************/
									if(SYS_MAJOR_BS_FLAG==0)
									{
										Calculate_FLAG=0;                                    //状态标志为正常
										Time_time4_Cuo_buf[SYS_MAJOR_BS_FLAG]=Time_time4_Cuo;//测距时间戳记录-测试用
											if(Calculate_EN==0x01||Calculate_EN==0x02||Calculate_EN==0x03||Calculate_EN==0x04)		
											{
													TAG_ID=Calculate_TAG_ID;
													Calculate_FLAG=1;												
													SYS_MAJOR_BS_FLAG=1;			
													ERROR_FLAG=0;                                      //错误标志为归0
											}										
									}
									/*****************************************
													开始定位，主基站与标签测距
									*****************************************/
									if(SYS_MAJOR_BS_FLAG==1) //主基站与标签测距
									{
										  Calculate_FLAG=1;                                //状态标志为A基站检测中
										Time_time4_Cuo_buf[SYS_MAJOR_BS_FLAG]=Time_time4_Cuo;//测距时间戳记录-测试用
											SYS_dis_buf_t[0]=DW1000send(255,TAG_ID);		//填入寄存器MODBUS																		
											if(SYS_dis_buf_t[0]!=0)
											{
													SYS_MAJOR_BS_FLAG=2;											
													//deca_sleep(RNG_DELAY_MS);//休眠固定时间
													ERROR_FLAG=0;                                      //错误标志为归0
											}		
																		
									}					
									/*****************************************
													 次基站与标签测距依次进行
									*****************************************/
									for(i=0;i<6;i++) //次基站与标签测距 
								  {
											if(SYS_MAJOR_BS_FLAG==2+i) //次基站与标签测距
											{														
												   Calculate_FLAG=2+i;                  //状态标志为次基站
												Time_time4_Cuo_buf[SYS_MAJOR_BS_FLAG]=Time_time4_Cuo;//测距时间戳记录-测试用
												  if(Flash_BS_EN_X_Y_Z[i][0]==1)
													{																										
															SYS_dis_buf_t[1+i]=DW1000send_dist_msg(255,248+i,TAG_ID);
															if(SYS_dis_buf_t[1+i]!=0)
															{
																	SYS_MAJOR_BS_FLAG++;															
															    //deca_sleep(RNG_DELAY_MS);//休眠固定时间
																	ERROR_FLAG=0;                                      //错误标志为归0
															}																						
													}
													else
													{
															SYS_MAJOR_BS_FLAG++;
													}
													
											}
											
									}								
									/*****************************************
													   测距结束，处理数据并输出
									*****************************************/
						      if(SYS_MAJOR_BS_FLAG==8)
									{
										  double clua_x_y[2];
										  uint8 i;
										  uint8 cla_flag=0;
											Time_time4_Cuo_buf[SYS_MAJOR_BS_FLAG]=Time_time4_Cuo;//测距时间戳记录-测试用
									    
											for(i=0;i<7;i++)
											{											
													dist[i] = LP(SYS_dis_buf_t[i],i+7*TAG_ID);//LP 为低通滤波器，让数据更稳定		
											}
									
											cla_flag=PersonPosition(Flash_MAJOR_BS_X_Y_Z[0],Flash_MAJOR_BS_X_Y_Z[1],dist[0],
																		 Flash_BS_EN_X_Y_Z[0][1],Flash_BS_EN_X_Y_Z[0][2],dist[1],Flash_BS_EN_X_Y_Z[0][0],
																		 Flash_BS_EN_X_Y_Z[1][1],Flash_BS_EN_X_Y_Z[1][2],dist[2],Flash_BS_EN_X_Y_Z[1][0],
																		 Flash_BS_EN_X_Y_Z[2][1],Flash_BS_EN_X_Y_Z[2][2],dist[3],Flash_BS_EN_X_Y_Z[2][0],
																		 Flash_BS_EN_X_Y_Z[3][1],Flash_BS_EN_X_Y_Z[3][2],dist[4],Flash_BS_EN_X_Y_Z[3][0],
																		 Flash_BS_EN_X_Y_Z[4][1],Flash_BS_EN_X_Y_Z[4][2],dist[5],Flash_BS_EN_X_Y_Z[4][0],
																		 Flash_BS_EN_X_Y_Z[5][1],Flash_BS_EN_X_Y_Z[5][2],dist[6],Flash_BS_EN_X_Y_Z[5][0],
																			clua_x_y);
											
										 if(cla_flag!=0) 
										  {
												  
													Calculate_FLAG=8;                               //状态显示计算错误
													Calculate_TAG_X=0; //赋予到寄存器
													Calculate_TAG_Y=0; //赋予到寄存器
													Calculate_TAG_Z=0; //赋予到寄存器			
										  }
										 else
											 
											{
												 Calculate_FLAG=0;
												 clua_x_y[0] = KalmanFilter(clua_x_y[0],KALMAN_Q,KALMAN_R,0+2*TAG_ID);    //卡尔曼滤波
												 clua_x_y[1] = KalmanFilter(clua_x_y[1],KALMAN_Q,KALMAN_R,1+2*TAG_ID);		//卡尔曼滤波
												 Calculate_TAG_X=(int)(clua_x_y[0]); //赋予到寄存器
												 Calculate_TAG_Y=(int)(clua_x_y[1]); //赋予到寄存器
												 Calculate_TAG_Z=0; //赋予到寄存器											
											}
										
											/******************************///赋予值
											
											Calculate_Station_TAG[0]=dist[0];
											for(o=0;o<6;o++)   //赋予到寄存器
											{
												
													if(Flash_BS_EN_X_Y_Z[o][0]==1) Calculate_Station_TAG[o+1]=dist[o+1];
												
													else Calculate_Station_TAG[o+1]=0;
													
											}								
											/******************************/
												if(Calculate_EN==0x03||Calculate_EN==0x04)//自动输出
											{
												MODBUS_xyz(TAG_ID,Calculate_TAG_X,Calculate_TAG_Y,Calculate_TAG_Z,Calculate_Station_TAG);									

											}
											if(Calculate_EN==0x01||Calculate_EN==0x03)//单次检测后清零使能
											{												
												Calculate_EN=0;
											}
										
											if(LED_FLAG>6)
											{
		//											GPIO_Toggle(LED_GPIO,LED);
													LED_FLAG=0;
											}
											else LED_FLAG++;
											SYS_MAJOR_BS_FLAG=0;

											
											Time_time4_Cuo_buf[9]=Time_time4_Cuo;				
											/*											
											{
												uint8 lkj;
												for(lkj=0;lkj<10;lkj++)
												{
													printf("buf[%d]time: %d \n",lkj,Time_time4_Cuo_buf[lkj]); 
												}
											
											}
											*/
											
									}			

									if(ERROR_FLAG>1000)   //测距发生错误
									{				
										
										if(Calculate_EN==0x03||Calculate_EN==0x04)//自动输出
											{
												MODBUS_xyz(TAG_ID,Calculate_TAG_X,Calculate_TAG_Y,Calculate_TAG_Z,Calculate_Station_TAG);									
											}
											if(Calculate_EN==0x01||Calculate_EN==0x03)//单次检测后清零使能
											{												
												Calculate_EN=0;
											}
										SYS_Calculate_ACTIVE_FLAG=0;
										SYS_BS_MESSAGE_FLAG=0; //				
									  SYS_MAJOR_BS_FLAG=0; //程序标志位归位							
										ERROR_FLAG=0;        //错误标志归0
										
									}
									
									
						   
							}		

					}
			}								
		}
		
		
	return 0;
		
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_tx_timestamp_u64()
 *
 * @brief Get the TX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_tx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readtxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn get_rx_timestamp_u64()
 *
 * @brief Get the RX time-stamp in a 64-bit variable.
 *        /!\ This function assumes that length of time-stamps is 40 bits, for both TX and RX!
 *
 * @param  none
 *
 * @return  64-bit value of the read time-stamp.
 */
static uint64 get_rx_timestamp_u64(void)
{
    uint8 ts_tab[5];
    uint64 ts = 0;
    int i;
    dwt_readrxtimestamp(ts_tab);
    for (i = 4; i >= 0; i--)
    {
        ts <<= 8;
        ts |= ts_tab[i];
    }
    return ts;
}

/*! ------------------------------------------------------------------------------------------------------------------
 * @fn final_msg_set_ts()
 *
 * @brief Fill a given timestamp field in the final message with the given value. In the timestamp fields of the final
 *        message, the least significant byte is at the lower address.
 *
 * @param  ts_field  pointer on the first byte of the timestamp field to fill
 *         ts  timestamp value
 *
 * @return none
 */
static void final_msg_set_ts(uint8 *ts_field, uint64 ts)
{
    int i;
    for (i = 0; i < FINAL_MSG_TS_LEN; i++)
    {
        ts_field[i] = (uint8) ts;
        ts >>= 8;
    }
}
static void final_msg_set_dist(uint8 *ts_field, uint32 dist)
{
    int i;
    for (i = 0; i < 4; i++)
    {
        ts_field[i] = (uint8) dist;
        dist >>= 8;
    }
}
/*****************************************************************************************************************************************************
 * NOTES:
 *
 * 1. The sum of the values is the TX to RX antenna delay, experimentally determined by a calibration process. Here we use a hard coded typical value
 *    but, in a real application, each device should have its own antenna delay properly calibrated to get the best possible precision when performing
 *    range measurements.
 * 2. The messages here are similar to those used in the DecaRanging ARM application (shipped with EVK1000 kit). They comply with the IEEE
 *    802.15.4 standard MAC data frame encoding and they are following the ISO/IEC:24730-62:2013 standard. The messages used are:
 *     - a poll message sent by the initiator to trigger the ranging exchange.
 *     - a response message sent by the responder allowing the initiator to go on with the process
 *     - a final message sent by the initiator to complete the exchange and provide all information needed by the responder to compute the
 *       time-of-flight (distance) estimate.
 *    The first 10 bytes of those frame are common and are composed of the following fields:
 *     - byte 0/1: frame control (0x8841 to indicate a data frame using 16-bit addressing).
 *     - byte 2: sequence number, incremented for each new frame.
 *     - byte 3/4: PAN TAG_ID (0xDECA).
 *     - byte 5/6: destination address, see NOTE 3 below.
 *     - byte 7/8: source address, see NOTE 3 below.
 *     - byte 9: function code (specific values to indicate which message it is in the ranging process).
 *    The remaining bytes are specific to each message as follows:
 *    Poll message:
 *     - no more data
 *    Response message:
 *     - byte 10: activity code (0x02 to tell the initiator to go on with the ranging exchange).
 *     - byte 11/12: activity parameter, not used here for activity code 0x02.
 *    Final message:
 *     - byte 10 -> 13: poll message transmission timestamp.
 *     - byte 14 -> 17: response message reception timestamp.
 *     - byte 18 -> 21: final message transmission timestamp.
 *    All messages end with a 2-byte checksum automatically set by DW1000.
 * 3. Source and destination addresses are hard coded constants in this example to keep it simple but for a real product every device should have a
 *    unique TAG_ID. Here, 16-bit addressing is used to keep the messages as short as possible but, in an actual application, this should be done only
 *    after an exchange of specific messages used to define those short addresses for each device participating to the ranging exchange.
 * 4. Delays between frames have been chosen here to ensure proper synchronisation of transmission and reception of the frames between the initiator
 *    and the responder and to ensure a correct accuracy of the computed distance. The user is referred to DecaRanging ARM Source Code Guide for more
 *    details about the timings involved in the ranging process.
 * 5. This timeout is for complete reception of a frame, i.e. timeout duration must take into account the length of the expected frame. Here the value
 *    is arbitrary but chosen large enough to make sure that there is enough time to receive the complete response frame sent by the responder at the
 *    110k data rate used (around 3 ms).
 * 6. In a real application, for optimum performance within regulatory limits, it may be necessary to set TX pulse bandwidth and TX power, (using
 *    the dwt_configuretxrf API call) to per device calibrated values saved in the target system or the DW1000 OTP memory.
 * 7. dwt_writetxdata() takes the full size of the message as a parameter but only copies (size - 2) bytes as the check-sum at the end of the frame is
 *    automatically appended by the DW1000. This means that our variable could be two bytes shorter without losing any data (but the sizeof would not
 *    work anymore then as we would still have to indicate the full length of the frame to dwt_writetxdata()). It is also to be noted that, when using
 *    delayed send, the time set for transmission must be far enough in the future so that the DW1000 IC has the time to process and start the
 *    transmission of the frame at the wanted time. If the transmission command is issued too late compared to when the frame is supposed to be sent,
 *    this is indicated by an error code returned by dwt_starttx() API call. Here it is not tested, as the values of the delays between frames have
 *    been carefully defined to avoid this situation.
 * 8. We use polled mode of operation here to keep the example as simple as possible but all status events can be used to generate interrupts. Please
 *    refer to DW1000 User Manual for more details on "interrupts". It is also to be noted that STATUS register is 5 bytes long but, as the event we
 *    use are all in the first bytes of the register, we can use the simple dwt_read32bitreg() API call to access it instead of reading the whole 5
 *    bytes.
 * 9. As we want to send final TX timestamp in the final message, we have to compute it in advance instead of relying on the reading of DW1000
 *    register. Timestamps and delayed transmission time are both expressed in device time units so we just have to add the desired response delay to
 *    response RX timestamp to get final transmission time. The delayed transmission time resolution is 512 device time units which means that the
 *    lower 9 bits of the obtained value must be zeroed. This also allows to encode the 40-bit value in a 32-bit words by shifting the all-zero lower
 *    8 bits.
 * 10. In this operation, the high order byte of each 40-bit timestamps is discarded. This is acceptable as those time-stamps are not separated by
 *     more than 2**32 device time units (which is around 67 ms) which means that the calculation of the round-trip delays (needed in the
 *     time-of-flight computation) can be handled by a 32-bit subtraction.
 * 11. The user is referred to DecaRanging ARM application (distributed with EVK1000 product) for additional practical example of usage, and to the
 *     DW1000 API Guide for more details on the DW1000 driver functions.
1。该值的总和是通过校准过程实验确定的TX到RX天线延迟。这里我们使用硬编码的典型值。
但是，在实际应用中，每个设备都应该有自己的天线延迟适当校准，以在执行时获得最佳可能的精度。
距离测量。
2。这里的消息与脱钩ARM应用程序（EVK1000套件）一起使用。他们遵守IEEE。
802.15.4标准的MAC数据帧编码，它们遵循ISO/IEC:2473062/2013标准。所使用的消息是：
-发起者发送的轮询消息以触发测距交换。
-响应方发送的响应消息，允许发起方继续执行进程。
-由发起方发送的最终消息以完成交换，并提供响应者所需的所有信息来计算
飞行时间（距离）估计。
这些帧的前10个字节是常见的，并且由以下字段组成：
-字节0/1：帧控制（0x88 41，用16位寻址指示数据帧）。
-字节2：序列号，为每个新帧增加。
-字节3/4：PANTAGIGID（0xDECA）。
-字节5/6：目的地址，见下面的注释3。
-字节7/8：源地址，见下文注释3。
-字节9：函数代码（特定值来指示它在测距过程中的哪个消息）。
其余的字节对每个消息都是特定的：
民意测验：
-不再有数据
响应消息：
-字节10：活动代码（0x02告诉发起者继续进行测距交换）。
-字节11/12：活动参数，此处不用于活动代码0x02。
最后消息：
-字节10＞13：轮询消息传输时间戳。
-字节14＞17：响应消息接收时间戳。
-字节18＞21：最后的消息传输时间戳。
所有消息以DW1000自动设置的2字节校验和结束。
三。在这个例子中，源地址和目的地址是硬编码常量，以保持简单，但对于实际产品，每个设备都应该有一个
独特的TAGJID。这里，16位寻址被用来保持消息尽可能短，但是，在实际应用中，这应该只做。
在交换特定消息之后，用于为参与测距交换的每个设备定义那些短地址。
4。这里选择帧之间的延迟，以确保发送器之间的帧的发送和接收的适当同步。
和应答器，保证计算距离的正确精度。用户被称为更多的脱角臂源代码指南。
有关测距过程中的计时的细节。
5。此超时是为了完全接收帧，即超时持续时间必须考虑预期帧的长度。这里的价值
是任意的，但选择大到足以确保有足够的时间来接收响应者发送的完整响应帧。
使用1K数据速率（约3毫秒）。
6。在实际应用中，为了在调节范围内达到最佳性能，可能需要设置TX脉冲带宽和TX功率，（使用
DWTSCOMPUTURTXRF API调用到在目标系统或DW1000 OTP存储器中保存的每个设备校准值。
7。DWTJWRIGETXDATA（）将消息的完整大小作为参数，但仅在帧结尾的校验和时复制（大小为2）字节。
由DW1000自动追加。这意味着我们的变量可以在不丢失任何数据的情况下短两个字节（但是sieof不会）。
工作，然后我们仍然必须指示框架的完整长度为DWTTWORGETXDATA（））。还需要注意的是，使用时
延迟发送，传输的时间设置在未来必须足够远，因此DW1000 IC有时间来处理和启动
在需要的时间内传输帧。如果传输命令发出的时间太晚，与帧被发送时相比，
这是由DWTSARSTTTX（）API调用返回的错误代码所指示的。这里没有测试，因为帧之间的延迟值有
被仔细定义以避免这种情况。
8。我们在这里使用轮询模式来保持示例尽可能简单，但是所有状态事件都可以用来产生中断。拜托
有关“中断”的详细信息，请参阅DW1000用户手册。还需要注意的是，状态寄存器是5字节长，但作为事件我们
使用都在寄存器的第一个字节中，我们可以使用简单的DWTSRead 32位Read（）API调用来访问它，而不是读取整个5个字节。
字节。
9。当我们想要在最终的消息中发送最终的TX时间戳时，我们必须提前计算它，而不是依赖于DW1000的读取。
寄存器。时间戳和延迟传输时间都表示在设备时间单位中，所以我们只需将期望的响应延迟添加到
响应RX时间戳以获得最终传输时间。延迟传输时间分辨率是512个设备时间单位，这意味着
所获得的值的低9位必须被归零。这还允许通过移位全零下位来在32位字中编码40位值。
8位。
10。在这个操作中，丢弃每个40位时间戳的高阶字节。这是可以接受的，因为那些时间戳没有分开。
超过2×32的设备时间单位（大约67毫秒），这意味着计算往返延迟（需要在
飞行时间计算可以通过32位减法来处理。
11。用户被称为脱角ARM应用程序（用EVK1000产品分发），用于附加的实际使用示例，以及
DW1000 API指南中关于DW1000驱动程序功能的更多细节。
 ****************************************************************************************************************************************************/
