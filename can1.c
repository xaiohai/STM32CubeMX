#include "can1.h"
#include "can2.h"
//CAN初始化
//tsjw:重新同步跳跃时间单元.范围:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:时间段2的时间单元.   范围:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:时间段1的时间单元.   范围:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :波特率分频器.范围:1~1024; tq=(brp)*tpclk1
//波特率=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,普通模式;CAN_Mode_LoopBack,回环模式;
//Fpclk1的时钟在初始化的时候设置为42M,如果设置CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//则波特率为:42M/((6+7+1)*6)=500Kbps
//返回值:0,初始化OK;
//其他,初始化失败; 
/*******************************************************/
CAN1_DATA CAN1_DATA_One;//作为传递数据的工具被定义
CAN1_DATA CAN1_BUFF[CAN1_BUFF_SIZE];//数据缓冲区
uint16_t CAN1_Location = 0;//标志位
uint16_t CAN1_Cache_Size = 0;//已缓存数据大小

/*******************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode,uint16_t K_value)
{

	GPIO_InitTypeDef 				GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
	#if CAN1_RX0_INT_ENABLE 
	NVIC_InitTypeDef  NVIC_InitStructure;
	#endif
	//使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	

	//初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化PA11,PA12

	//引脚复用映射配置
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1

	//CAN单元设置
	CAN_InitStructure.CAN_TTCM=ENABLE;//非时间触发通信模式   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//软件自动离线管理	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
	CAN_InitStructure.CAN_NART=ENABLE;	//禁止报文自动传送 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
	CAN_InitStructure.CAN_SJW=tsjw;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
	CAN_Init(CAN1, &CAN_InitStructure);   // 初始化CAN1 

	//配置过滤器
	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //过滤器0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0240;////32位ID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffe0;//32位MASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0006;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化



	#if CAN1_RX0_INT_ENABLE

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // 主优先级为1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // 次优先级为0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif

	TIM3_Int_Init(K_value);
	return 0;
}   

#if CAN1_RX0_INT_ENABLE	//使能RX0中断
//中断服务函数			    
void CAN1_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	int i=0;

	CAN_Receive(CAN1, 0, &RxMessage);
	for(i=0;i<8;i++)
		printf("%d  ",RxMessage.Data[i]);
	printf("CAN1  \r\n");

}
#endif

//can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
//len:数据长度(最大为8)				     
//msg:数据指针,最大为8个字节.
//返回值:0,成功;
//其他,失败;
u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=id;	 	// 标准标识符为0
	TxMessage.RTR=0;		 	// 消息类型为数据帧，一帧8位
	TxMessage.DLC=8;	 		// 发送两帧信息
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];				 // 第一帧信息          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
	if(i>=0XFFF)return 1;
	CAN1_Cache_Size--;
	return 0;		
}
//can口接收数据查询
//buf:数据缓存区;	 
//返回值:		0,无数据被收到;
//					其他,接收的数据长度;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
	u32 i;
	CanRxMsg RxMessage;
	if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//读取数据	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}


//通用定时器3中断初始化
//arr：自动重装值。
//psc：时钟预分频数
//定时器溢出时间计算方法:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=定时器工作频率,单位:Mhz
//这里使用的是定时器3!
void TIM3_Int_Init(uint16_t k)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///使能TIM3时钟

	TIM_TimeBaseInitStructure.TIM_Period = 125-1; 	//自动重装载值
	TIM_TimeBaseInitStructure.TIM_Prescaler=84 * k;  //定时器分频
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //向上计数模式
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//初始化TIM3

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //允许定时器3更新中断
	TIM_Cmd(TIM3,ENABLE); //使能定时器3

	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //定时器3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x03; //抢占优先级1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

//定时器3中断服务函数
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //溢出中断
	{
		if(CAN1_Cache_Size > 0)
		{
			if(CAN1_Location < CAN1_BUFF_SIZE)
			{
				while(CAN1_Send_Msg(CAN1_BUFF[CAN1_Location].buff,
														CAN1_BUFF[CAN1_Location].LEN,
														CAN1_BUFF[CAN1_Location].ID));
				CAN1_Location++;
				if(CAN1_Location >= CAN1_BUFF_SIZE)
				{
					CAN1_Location = 0;
				}
			}
		}
		if(CAN2_Cache_Size > 0)
		{
			if(CAN2_Location < CAN2_BUFF_SIZE)
			{
				while(CAN2_Send_Msg(CAN1_BUFF[CAN2_Location].buff,
														CAN1_BUFF[CAN2_Location].LEN,
														CAN1_BUFF[CAN2_Location].ID));
				CAN2_Location++;
				if(CAN2_Location >= CAN2_BUFF_SIZE)
				{
					CAN2_Location = 0;
				}
			}
		}
	}	
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //清除中断标志位
}

uint8_t CAN1_Buffer_Data_Entry(void)
{
	uint16_t Data_value = 0;
	if(CAN1_Cache_Size < (CAN1_BUFF_SIZE))
	{
//		printf ("1 CAN1_Cache_Size = %d		\r\n",CAN1_Cache_Size);
		Data_value = CAN1_Cache_Size + CAN1_Location;
//		printf ("Data_value = %d		\r\n",Data_value);
		while(Data_value == CAN1_BUFF_SIZE)
		{
//			printf ("Data_value = %d		\r\n",Data_value);
			Data_value -= CAN1_BUFF_SIZE;
		}
		CAN1_BUFF[Data_value].buff[0]=CAN1_DATA_One.buff[0];
		CAN1_BUFF[Data_value].buff[1]=CAN1_DATA_One.buff[1];
		CAN1_BUFF[Data_value].buff[2]=CAN1_DATA_One.buff[2];
		CAN1_BUFF[Data_value].buff[3]=CAN1_DATA_One.buff[3];
		CAN1_BUFF[Data_value].buff[4]=CAN1_DATA_One.buff[4];
		CAN1_BUFF[Data_value].buff[5]=CAN1_DATA_One.buff[5];
		CAN1_BUFF[Data_value].buff[6]=CAN1_DATA_One.buff[6];
		CAN1_BUFF[Data_value].buff[7]=CAN1_DATA_One.buff[7];
		CAN1_BUFF[Data_value].LEN=CAN1_DATA_One.LEN;
		CAN1_BUFF[Data_value].ID=CAN1_DATA_One.ID;
		CAN1_Cache_Size++;
//		printf ("2 CAN1_Cache_Size = %d		\r\n",CAN1_Cache_Size);
		return 0;
	}
	return 1;
}












