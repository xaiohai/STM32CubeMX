#ifndef __CAN2_H
#define __CAN2_H
#include "includes.h"

#define CAN2_BUFF_SIZE  32

typedef struct
{
	u32 ID;
	unsigned char LEN;
	u8 buff[8];
}CAN2_DATA;

extern CAN2_DATA CAN2_BUFF[CAN2_BUFF_SIZE];
extern CAN2_DATA CAN2_DATA_One;
extern uint16_t CAN2_Location;
extern uint16_t CAN2_Cache_Size;
//CAN1接收RX0中断使能
#define CAN2_RX0_INT_ENABLE	1		//0,不使能;1,使能.								    
										 							 				    
u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN初始化
 
u8 CAN2_Send_Msg(u8* msg,u8 len,uint32_t id);				//发送数据

u8 CAN2_Receive_Msg(u8 *buf);							//接收数据

uint8_t CAN2_Buffer_Data_Entry(void);
#endif

















