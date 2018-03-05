#include "can1.h"
#include "can2.h"
//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//����,��ʼ��ʧ��; 
/*******************************************************/
CAN1_DATA CAN1_DATA_One;//��Ϊ�������ݵĹ��߱�����
CAN1_DATA CAN1_BUFF[CAN1_BUFF_SIZE];//���ݻ�����
uint16_t CAN1_Location = 0;//��־λ
uint16_t CAN1_Cache_Size = 0;//�ѻ������ݴ�С

/*******************************************************/
u8 CAN1_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode,uint16_t K_value)
{

	GPIO_InitTypeDef 				GPIO_InitStructure; 
	CAN_InitTypeDef        	CAN_InitStructure;
	CAN_FilterInitTypeDef  	CAN_FilterInitStructure;
	#if CAN1_RX0_INT_ENABLE 
	NVIC_InitTypeDef  NVIC_InitStructure;
	#endif
	//ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	                   											 

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	

	//��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12

	//���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1

	//CAN��Ԫ����
	CAN_InitStructure.CAN_TTCM=ENABLE;//��ʱ�䴥��ͨ��ģʽ   
	CAN_InitStructure.CAN_ABOM=DISABLE;	//����Զ����߹���	  
	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
	CAN_InitStructure.CAN_NART=ENABLE;	//��ֹ�����Զ����� 
	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
	CAN_InitStructure.CAN_SJW=tsjw;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 

	//���ù�����
	CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0240;////32λID
	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0xffe0;//32λMASK
	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0006;
	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��



	#if CAN1_RX0_INT_ENABLE

	CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    

	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;     // �����ȼ�Ϊ1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	#endif

	TIM3_Int_Init(K_value);
	return 0;
}   

#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
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

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//����,ʧ��;
u8 CAN1_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	u8 mbox;
	u16 i=0;
	CanTxMsg TxMessage;
	TxMessage.StdId=id;	 	// ��׼��ʶ��Ϊ0
	TxMessage.RTR=0;		 	// ��Ϣ����Ϊ����֡��һ֡8λ
	TxMessage.DLC=8;	 		// ������֡��Ϣ
	for(i=0;i<len;i++)
	TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
	mbox= CAN_Transmit(CAN1, &TxMessage);   
	i=0;
	while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
	if(i>=0XFFF)return 1;
	CAN1_Cache_Size--;
	return 0;		
}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:		0,�����ݱ��յ�;
//					����,���յ����ݳ���;
u8 CAN1_Receive_Msg(u8 *buf)
{		   		   
	u32 i;
	CanRxMsg RxMessage;
	if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
	CAN_Receive(CAN1, CAN_FIFO0, &RxMessage);//��ȡ����	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}


//ͨ�ö�ʱ��3�жϳ�ʼ��
//arr���Զ���װֵ��
//psc��ʱ��Ԥ��Ƶ��
//��ʱ�����ʱ����㷽��:Tout=((arr+1)*(psc+1))/Ft us.
//Ft=��ʱ������Ƶ��,��λ:Mhz
//����ʹ�õ��Ƕ�ʱ��3!
void TIM3_Int_Init(uint16_t k)
{
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3,ENABLE);  ///ʹ��TIM3ʱ��

	TIM_TimeBaseInitStructure.TIM_Period = 125-1; 	//�Զ���װ��ֵ
	TIM_TimeBaseInitStructure.TIM_Prescaler=84 * k;  //��ʱ����Ƶ
	TIM_TimeBaseInitStructure.TIM_CounterMode=TIM_CounterMode_Up; //���ϼ���ģʽ
	TIM_TimeBaseInitStructure.TIM_ClockDivision=TIM_CKD_DIV1; 

	TIM_TimeBaseInit(TIM3,&TIM_TimeBaseInitStructure);//��ʼ��TIM3

	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE); //����ʱ��3�����ж�
	TIM_Cmd(TIM3,ENABLE); //ʹ�ܶ�ʱ��3

	NVIC_InitStructure.NVIC_IRQChannel=TIM3_IRQn; //��ʱ��3�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0x03; //��ռ���ȼ�1
	NVIC_InitStructure.NVIC_IRQChannelSubPriority=0x03; //�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd=ENABLE;
	NVIC_Init(&NVIC_InitStructure);

}

//��ʱ��3�жϷ�����
void TIM3_IRQHandler(void)
{
	if(TIM_GetITStatus(TIM3,TIM_IT_Update)==SET) //����ж�
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
	TIM_ClearITPendingBit(TIM3,TIM_IT_Update);  //����жϱ�־λ
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












