#include "deca_device_api.h"
#include "deca_regs.h"
#include "deca_sleep.h"
#include "port.h"
#include "usart.h"
#include "stmflash.h"
#include "modbus.h"
#include "dw_main.h"
#include "uart.h"
#include "../../../../Netapp/udp_client.h"

#define ADDR 1   //取消
#define NULL 0
#define MODBUS_LENGTH 100
unsigned int UsartNUM;
unsigned short modbus_reg[MODBUS_LENGTH];
uint8 canbuf[8]={0xff,0x00,0x30,0x31,0x32,0x01,0x02,0x03};
const unsigned char auchCRCHi[] = /* CRC锟斤拷位锟街节憋拷*/
{ 	 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40,
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41,
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40, 0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 
	0x00, 0xC1, 0x81, 0x40, 0x01, 0xC0, 0x80, 0x41, 0x01, 0xC0, 0x80, 0x41, 0x00, 0xC1, 0x81, 0x40 
} ; 

const unsigned char auchCRCLo[] = /* CRC锟斤拷位锟街节憋拷*/ 
{ 
	0x00, 0xC0, 0xC1, 0x01, 0xC3, 0x03, 0x02, 0xC2, 0xC6, 0x06, 0x07, 0xC7, 0x05, 0xC5, 0xC4, 0x04, 0xCC,
	0x0C, 0x0D, 0xCD, 0x0F, 0xCF, 0xCE, 0x0E, 0x0A, 0xCA, 0xCB, 0x0B, 0xC9, 0x09, 0x08, 0xC8, 0xD8, 
	0x18, 0x19, 0xD9, 0x1B, 0xDB, 0xDA, 0x1A, 0x1E, 0xDE, 0xDF, 0x1F, 0xDD, 0x1D, 0x1C, 0xDC, 0x14, 
	0xD4, 0xD5, 0x15, 0xD7, 0x17, 0x16, 0xD6, 0xD2, 0x12, 0x13, 0xD3, 0x11, 0xD1, 0xD0, 0x10, 0xF0, 
	0x30, 0x31, 0xF1, 0x33, 0xF3, 0xF2, 0x32, 0x36, 0xF6, 0xF7, 0x37, 0xF5, 0x35, 0x34, 0xF4, 0x3C, 
	0xFC, 0xFD, 0x3D, 0xFF, 0x3F, 0x3E, 0xFE, 0xFA, 0x3A, 0x3B, 0xFB, 0x39, 0xF9, 0xF8, 0x38, 0x28, 
	0xE8, 0xE9, 0x29, 0xEB, 0x2B, 0x2A, 0xEA, 0xEE, 0x2E, 0x2F, 0xEF, 0x2D, 0xED, 0xEC, 0x2C, 0xE4, 
	0x24, 0x25, 0xE5, 0x27, 0xE7, 0xE6, 0x26, 0x22, 0xE2, 0xE3, 0x23, 0xE1, 0x21, 0x20, 0xE0, 0xA0, 
	0x60, 0x61, 0xA1, 0x63, 0xA3, 0xA2, 0x62, 0x66, 0xA6, 0xA7, 0x67, 0xA5, 0x65, 0x64, 0xA4, 0x6C, 
	0xAC, 0xAD, 0x6D, 0xAF, 0x6F, 0x6E, 0xAE, 0xAA, 0x6A, 0x6B, 0xAB, 0x69, 0xA9, 0xA8, 0x68, 0x78, 
	0xB8, 0xB9, 0x79, 0xBB, 0x7B, 0x7A, 0xBA, 0xBE, 0x7E, 0x7F, 0xBF, 0x7D, 0xBD, 0xBC, 0x7C, 0xB4, 
	0x74, 0x75, 0xB5, 0x77, 0xB7, 0xB6, 0x76, 0x72, 0xB2, 0xB3, 0x73, 0xB1, 0x71, 0x70, 0xB0, 0x50, 
	0x90, 0x91, 0x51, 0x93, 0x53, 0x52, 0x92, 0x96, 0x56, 0x57, 0x97, 0x55, 0x95, 0x94, 0x54, 0x9C, 
	0x5C, 0x5D, 0x9D, 0x5F, 0x9F, 0x9E, 0x5E, 0x5A, 0x9A, 0x9B, 0x5B, 0x99, 0x59, 0x58, 0x98, 0x88, 
	0x48, 0x49, 0x89, 0x4B, 0x8B, 0x8A, 0x4A, 0x4E, 0x8E, 0x8F, 0x4F, 0x8D, 0x4D, 0x4C, 0x8C, 0x44, 
	0x84, 0x85, 0x45, 0x87, 0x47, 0x46, 0x86, 0x82, 0x42, 0x43, 0x83, 0x41, 0x81, 0x80, 0x40
} ;	 

/******************************************************************************
												    CRC校验
*******************************************************************************/
unsigned int CRC_Calculate(unsigned char *pdata,unsigned char num)
{
  unsigned char uchCRCHi = 0xFF ;               
	unsigned char  uchCRCLo = 0xFF ;               
	unsigned char uIndex ;                
	while(num --)                    
	{
		uIndex = uchCRCHi^*pdata++ ;           
		uchCRCHi = uchCRCLo^auchCRCHi[uIndex];
		uchCRCLo = auchCRCLo[uIndex];
	}
	return (uchCRCHi << 8 | uchCRCLo) ;
}
/******************************************************************************
												    数据应答发送函数
*******************************************************************************/
void MODBUS_Return(unsigned char *buf,unsigned int length)
{
	if(UsartNUM==1) 
		Usart1_SendString(buf,length);
	
	if(UsartNUM==3) 	
		Usart3_SendString(buf,length);

   if (0 != remote_host_master_connected)
      udp_modbus_client_raw_send(buf, length);

}

/******************************************************************************
												    数据接收函数
*******************************************************************************/
void MODBUS_Read(unsigned int startaddr,unsigned int number)
{
    //01 03 number 00 00 01 01 02 02 CRCH CRCL
    unsigned char send_length;
    unsigned char send_buf[100];
    unsigned char i;
    unsigned int crc;
	
    send_length = 0;
    send_buf[send_length++] = Flash_Modbus_ADDR;
    send_buf[send_length++] = 0x03;
    send_buf[send_length++] = number*2;
    for (i = 0;i < number;i++)
    {
        send_buf[send_length++] = modbus_reg[startaddr+i]/256;
        send_buf[send_length++] = modbus_reg[startaddr+i]%256;
    }
    crc = CRC_Calculate(send_buf,send_length);
    send_buf[send_length++] = crc/256;
    send_buf[send_length++] = crc%256;
    MODBUS_Return(send_buf,send_length);
}
/******************************************************************************
												    发送MODBUS协议字符串
*******************************************************************************/
void MODBUS_WriteString(unsigned int addr,unsigned int code)
{
    unsigned char send_buf[8];
	unsigned int crc;
	send_buf[0] = Flash_Modbus_ADDR;
	send_buf[1] = 16;
	send_buf[2] = (addr / 256);
	send_buf[3] = (addr % 256);
	send_buf[4] = (code / 256);
	send_buf[5] = (code % 256);
	crc = CRC_Calculate(send_buf,6);
	send_buf[6] = (crc / 256);
  send_buf[7] = (crc % 256);
	MODBUS_Return(send_buf,8);
}

/******************************************************************************
												    MODBUS接收数据处理函数
*******************************************************************************/
void MODBUS(unsigned char *buf,unsigned int length,unsigned int NUM)
{
	unsigned int startaddr,number,code;
	unsigned int crc;
	UsartNUM=NUM;
	  crc = CRC_Calculate(buf,length-2);

    if ( buf[0] == Flash_Modbus_ADDR && buf[length-2] == (crc / 256) && buf[length-1] == (crc % 256))
    {
			unsigned int Write_EN=0;
	
			MODBUS_datain();
        switch(buf[1])
        {
            case 0x03:    //读取数据&长度
                startaddr = buf[2]*256 + buf[3];
                number = buf[4]*256 + buf[5];
								if ((startaddr+number) > MODBUS_LENGTH)
								{	
									break;
								}					
								MODBUS_Read(startaddr,number);
                break;
            case 0x06:      //写一个数据
                startaddr = buf[2]*256 + buf[3];
                code = buf[4]*256 + buf[5];
								if ((startaddr) > MODBUS_LENGTH)
								{	
									break;
								}
                modbus_reg[startaddr] = code;
                MODBUS_Return(buf,length);
								Write_EN=1;
								break;
								
							
						case 0x10:     //写入数据
								startaddr = buf[2]*256 + buf[3];
								number = buf[4]*256 + buf[5];
								if ((startaddr+number) > MODBUS_LENGTH)
								{	
									break;
								}
								for (code = 0;code < number;code++)
								{
									modbus_reg[startaddr + code] = buf[7+code*2]*256 + buf[8+code*2];
								}			
								MODBUS_WriteString(startaddr,number);
								Write_EN=1;  //记得修改，只有FLASH参数变了才需要写入FLASH。
								break;	
						
						
            default:
                break;
        }
				MODBUS_dataout();
			  if(Write_EN==1) FLASH_write();
    }
}


/******************************************************************************
												    串口接收事件
*******************************************************************************/
void MODBUS_event() 
{
	if ((usart1_rx_length) && (time3_usart1 >= (10-Flash_Usart_BaudRate)*5))
		{

			//Usart3_SendString("123",3);
			MODBUS(usart1_rx_buf,usart1_rx_length,1);
			usart1_rx_length = 0;
			time3_usart1 = 0;
		}
		if ((usart3_rx_length) && (time3_usart3 >= (10-Flash_Usart_BaudRate)*5))
		{

			//Usart3_SendString("123",3);
			MODBUS(usart3_rx_buf,usart3_rx_length,3);
			usart3_rx_length = 0;
			time3_usart3 = 0;
		}
}
/******************************************************************************
												    数据导入MODBUS寄存器表
*******************************************************************************/
void MODBUS_datain() 
{
	uint16 o,q;
	
	/*
	uint16   Flash_Usart_BaudRate;   //设备串口通讯波特率 0：4800  1：9600 2：14400 3：19200 4：38400 5：56000 6：57600 7：115200  8：128000 9：256000
	uint16   Flash_Modbus_ADDR;        //Modbus ID号 
	uint16   Flash_structure_Mode;     //0:为二维平面3基站模式
	uint16   Flash_Device_Mode;       //设备模式 0：标签 1：次基站 2：主基站
	uint16   Flash_Device_ID;         //高8位为次基站ID，范围0~6  低8位为标签ID 0~247    （程序内部 标签ID为0~247  次基站ID为248~245  主基站ID为255）
	uint16   Flash_EN_X_Y_Z[7][4];   //1主基站+6次基站的位置，    使能标志：0为关  1为开   X,Y,Z各两字节
	uint16   Flash_FLAG;	 //标志位
	
	uint16   Calculate_EN;       //测量使能
	uint16   Calculate_TAG_ID;		//测量标签ID
	uint16		Calculate_FLAG;			//测量状态标志
	uint16		Calculate_TAG_X;		//测量标签的X轴
	uint16		Calculate_TAG_Y;		//测量标签的Y轴
	uint16		Calculate_TAG_Z;     //测量标签的Z轴
	uint16		Calculate_Station_TAG[7]; //测量标签的Z轴
		*/
	
	modbus_reg[0]=Flash_Usart_BaudRate;
	modbus_reg[1]=Flash_Modbus_ADDR;
	modbus_reg[2]=Flash_structure_Mode;
	modbus_reg[3]=Flash_Device_Mode;
	modbus_reg[4]=Flash_Device_ID;
	for(q=0;q<3;q++)
		{
				modbus_reg[q+5]=Flash_MAJOR_BS_X_Y_Z[q];
		}
		for(o=0;o<6;o++)
		{
			for(q=0;q<4;q++)
			{
					modbus_reg[8+o*4+q]=Flash_BS_EN_X_Y_Z[o][q];
			}
		}
	modbus_reg[32]=Calculate_EN;
	modbus_reg[33]=Calculate_TAG_ID;
	modbus_reg[34]=Calculate_FLAG;
	modbus_reg[35]=Calculate_TAG_X;
	modbus_reg[36]=Calculate_TAG_Y;
	modbus_reg[37]=Calculate_TAG_Z; 
		for(o=0;o<7;o++)
	{
		modbus_reg[38+o]=Calculate_Station_TAG[o];
	}
	modbus_reg[49]=firmware_version; 	
	
}
/******************************************************************************
												   从MODBUS寄存器表导出到输出
*******************************************************************************/
void MODBUS_dataout() 
{
		uint16 o,q;
	  //读写的数据才需要！只读的不用赋予！
		if(modbus_reg[0]<=9) Flash_Usart_BaudRate=modbus_reg[0];
		if(modbus_reg[1]<=255) Flash_Modbus_ADDR=modbus_reg[1];
		if(modbus_reg[2]<=1) Flash_structure_Mode=modbus_reg[2];
		if(modbus_reg[3]<=3) Flash_Device_Mode=modbus_reg[3];
		if((((modbus_reg[4]>>8)&0xFF)<=6)&&((modbus_reg[4]&0xFF)<=247)) Flash_Device_ID=modbus_reg[4];
	
		for(q=0;q<3;q++)
		{
				Flash_MAJOR_BS_X_Y_Z[q]=modbus_reg[q+5];
		}
		for(o=0;o<6;o++)
		{			
			for(q=0;q<4;q++)
			{  
				  if(q == 0) 
					{					
							if((modbus_reg[8+o*4+q]<=1))						
							Flash_BS_EN_X_Y_Z[o][q]=modbus_reg[8+o*4+q];
					}
					else
					{
							Flash_BS_EN_X_Y_Z[o][q]=modbus_reg[8+o*4+q];
					}
					
			}
		}
		
		if(modbus_reg[32]<=4) Calculate_EN=modbus_reg[32];
		
		if(modbus_reg[33]<=247)Calculate_TAG_ID=modbus_reg[33];

}
/******************************************************************************
												    自动输出XYZ坐标信息
*******************************************************************************/
void MODBUS_xyz(uint16 ID,uint16 x,uint16 y,uint16 z,uint16 *dis_bb)
{
    //01 03 number 00 00 01 01 02 02 CRCH CRCL
    unsigned char send_length;
    unsigned char send_buf[100];
    unsigned int crc;
		unsigned int i;
    send_length = 0;
    send_buf[send_length++] = Flash_Modbus_ADDR;
    send_buf[send_length++] = 0x03;
    send_buf[send_length++] = 24;
		
	 send_buf[send_length++] = ID/256;
	 send_buf[send_length++] = ID%256;
	
		send_buf[send_length++] = Calculate_FLAG/256;
	  send_buf[send_length++] = Calculate_FLAG%256;
	
    send_buf[send_length++] = x/256;
    send_buf[send_length++] = x%256;
   
		send_buf[send_length++] = y/256;
    send_buf[send_length++] = y%256;
		
		send_buf[send_length++] = z/256;
    send_buf[send_length++] = z%256;
    for(i=0;i<7;i++)
		{
		send_buf[send_length++] =dis_bb[i]/256;
    send_buf[send_length++] =dis_bb[i]%256;
		}
		
		crc = CRC_Calculate(send_buf,send_length);
    send_buf[send_length++] = crc/256;
    send_buf[send_length++] = crc%256;
    MODBUS_Return(send_buf,send_length);
}




