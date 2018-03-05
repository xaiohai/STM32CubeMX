#ifndef __CAN1_H
#define __CAN1_H	 	    
#include "includes.h"	 

#define CAN1_BUFF_SIZE  32

typedef struct
{
	u32 ID;
	unsigned char LEN;
	u8 buff[8];
}CAN1_DATA;

extern CAN1_DATA CAN1_BUFF[CAN1_BUFF_SIZE];
extern CAN1_DATA CAN1_DATA_One;
//CAN1接收RX0中断使能
#define CAN1_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
										 							 				    
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode,uint16_t K_value);//CAN初始化
 
u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id);						//发送数据

u8 CAN1_Receive_Msg(u8 *buf);							//接收数据

void TIM3_Int_Init(uint16_t k);

uint8_t CAN1_Buffer_Data_Entry(void);

//初始化程序
/*

	CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6, CAN_Mode_Normal,50);//CAN初始化环回模式,波特率500Kbps
	CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6, CAN_Mode_Normal);		//CAN初始化环回模式,波特率500Kbps
	
	*/
//测试程序
/*

int j = 0;
		for(j=0;j<100000;j++);
		CAN1_DATA_One.buff[0] = 1;
		CAN1_DATA_One.buff[1] = 3;
		CAN1_DATA_One.buff[2] = 1;
		CAN1_DATA_One.buff[3] = 4;
		CAN1_DATA_One.buff[4] = 5;
		CAN1_DATA_One.buff[5] = 2;
		CAN1_DATA_One.buff[6] = 1;
		CAN1_DATA_One.buff[7] = 0;
		CAN1_DATA_One.LEN = 8;
		CAN1_DATA_One.ID = 0x12;
		CAN2_DATA_One.buff[0] = 1;
		CAN2_DATA_One.buff[1] = 3;
		CAN2_DATA_One.buff[2] = 1;
		CAN2_DATA_One.buff[3] = 4;
		CAN2_DATA_One.buff[4] = 5;
		CAN2_DATA_One.buff[5] = 2;
		CAN2_DATA_One.buff[6] = 1;
		CAN2_DATA_One.buff[7] = 0;
		CAN2_DATA_One.LEN = 8;
		CAN2_DATA_One.ID = 0x12;
//		while(CAN1_Send_Msg(CAN1_DATA_One.buff,
//														CAN1_DATA_One.LEN,
//														CAN1_DATA_One.ID));
//		printf ("CAN1_DATA_One.LEN = %d  \r\n",CAN1_DATA_One.LEN);
		for(j=0;j<16;j++)
		CAN1_Buffer_Data_Entry();
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<100000;j++);
		for(j=0;j<16;j++)
		CAN2_Buffer_Data_Entry();
		
		*/
#endif

















