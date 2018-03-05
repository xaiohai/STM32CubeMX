#include "can2.h"

//CAN��ʼ��
//tsjw:����ͬ����Ծʱ�䵥Ԫ.��Χ:CAN_SJW_1tq~ CAN_SJW_4tq
//tbs2:ʱ���2��ʱ�䵥Ԫ.   ��Χ:CAN_BS2_1tq~CAN_BS2_8tq;
//tbs1:ʱ���1��ʱ�䵥Ԫ.   ��Χ:CAN_BS1_1tq ~CAN_BS1_16tq
//brp :�����ʷ�Ƶ��.��Χ:1~1024; tq=(brp)*tpclk1
//������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
//mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
//Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN2_Mode_Init(CAN_SJW_1tq,CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
//������Ϊ:42M/((6+7+1)*6)=500Kbps
//����ֵ:0,��ʼ��OK;
//    ����,��ʼ��ʧ��; 

CAN2_DATA CAN2_DATA_One;
CAN2_DATA CAN2_BUFF[CAN2_BUFF_SIZE];
uint16_t CAN2_Location = 0;
uint16_t CAN2_Cache_Size = 0;

u8 CAN2_Mode_Init(u8 tsjw,u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

		GPIO_InitTypeDef 	   GPIO_InitStructure; 
		CAN_InitTypeDef        CAN_InitStructure;
		CAN_FilterInitTypeDef  CAN_FilterInitStructure;
		#if CAN2_RX0_INT_ENABLE 
		NVIC_InitTypeDef  NVIC_InitStructure;
		#endif
		//ʹ�����ʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��

		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��		
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN2ʱ��	

		//��ʼ��GPIO
		GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5| GPIO_Pin_6;
		GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
		GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
		GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
		GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
		GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PB11,PB12

		//���Ÿ���ӳ������
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF_CAN2); //GPIOB5����ΪCAN2
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_CAN2); //GPIOB6����ΪCAN2

		//CAN��Ԫ����
		CAN_InitStructure.CAN_TTCM=ENABLE;	//��ʱ�䴥��ͨ��ģʽ   
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
		CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN2 

		//���ù�����
		CAN_FilterInitStructure.CAN_FilterNumber=14;	  //������0
		CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
		CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
		CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
		CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
		CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
		CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
		CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
		CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��



		#if CAN2_RX0_INT_ENABLE

		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    

		NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;     // �����ȼ�Ϊ1
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;            // �����ȼ�Ϊ0
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
		NVIC_Init(&NVIC_InitStructure);
		#endif
		return 0;
}   

#if CAN2_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����			    
void CAN2_RX0_IRQHandler(void)
{
	CanRxMsg RxMessage;
	int i=0;
	
	CAN_Receive(CAN2, 0, &RxMessage);
	for(i=0;i<8;i++)
	printf("%d  ",RxMessage.Data[i]);
	printf("CAN2  \r\n");
	
}
#endif

//can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
//len:���ݳ���(���Ϊ8)				     
//msg:����ָ��,���Ϊ8���ֽ�.
//����ֵ:0,�ɹ�;
//����,ʧ��;
u8 CAN2_Send_Msg(u8* msg,u8 len,uint32_t id)
{	
	u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=id;	 // ��׼��ʶ��Ϊ0
  TxMessage.IDE=0;		 // ʹ�ñ�׼��ʶ��
  TxMessage.RTR=0;		 // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;	 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN2, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN2, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
	CAN2_Cache_Size--;
  return 0;		
}
//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
	u32 i;
	CanRxMsg RxMessage;
	if( CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
	CAN_Receive(CAN2, CAN_FIFO0, &RxMessage);//��ȡ����	
	for(i=0;i<RxMessage.DLC;i++)
	buf[i]=RxMessage.Data[i];  
	return RxMessage.DLC;	
}
uint8_t CAN2_Buffer_Data_Entry(void)
{
	uint16_t Data_value = 0;
	if(CAN2_Cache_Size < (CAN2_BUFF_SIZE))
	{
//		printf ("1 CAN2_Cache_Size = %d		\r\n",CAN2_Cache_Size);
		Data_value = CAN2_Cache_Size + CAN2_Location;
//		printf ("Data_value = %d		\r\n",Data_value);
		while(Data_value == CAN2_BUFF_SIZE)
		{
//			printf ("Data_value = %d		\r\n",Data_value);
			Data_value -= CAN2_BUFF_SIZE;
		}
		CAN2_BUFF[Data_value].buff[0]=CAN2_DATA_One.buff[0];
		CAN2_BUFF[Data_value].buff[1]=CAN2_DATA_One.buff[1];
		CAN2_BUFF[Data_value].buff[2]=CAN2_DATA_One.buff[2];
		CAN2_BUFF[Data_value].buff[3]=CAN2_DATA_One.buff[3];
		CAN2_BUFF[Data_value].buff[4]=CAN2_DATA_One.buff[4];
		CAN2_BUFF[Data_value].buff[5]=CAN2_DATA_One.buff[5];
		CAN2_BUFF[Data_value].buff[6]=CAN2_DATA_One.buff[6];
		CAN2_BUFF[Data_value].buff[7]=CAN2_DATA_One.buff[7];
		CAN2_BUFF[Data_value].LEN=CAN2_DATA_One.LEN;
		CAN2_BUFF[Data_value].ID=CAN2_DATA_One.ID;
		CAN2_Cache_Size++;
//		printf ("2 CAN2_Cache_Size = %d		\r\n",CAN2_Cache_Size);
		return 0;
	}
	return 1;
}













