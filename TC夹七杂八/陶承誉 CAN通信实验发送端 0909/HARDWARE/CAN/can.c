/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include <stdio.h>

u8 CAN1_receive_buf[20];
u8 CAN2_receive_buf[20];

u8 CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

		GPIO_InitTypeDef       gpio;
	  CAN_InitTypeDef        can;
  	CAN_FilterInitTypeDef  can_filter;
	
   	NVIC_InitTypeDef       nvic;

    //ʹ�����ʱ��
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	            
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
	  //���Ÿ���ӳ������
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
		    //��ʼ��GPIO
	  gpio.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_Init(GPIOA, &gpio);//��ʼ��PA11,PA12
	
		nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	nvic.NVIC_IRQChannelPreemptionPriority = 3;     // �����ȼ�Ϊ1
  	nvic.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
  	nvic.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&nvic);	
	
		CAN_DeInit(CAN1);
    CAN_StructInit(&can);
		
  	//CAN��Ԫ����
   	can.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	can.CAN_ABOM=ENABLE;	//����Զ����߹���	  
  	can.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	can.CAN_NART=DISABLE;	//��ֹ�����Զ����� 
  	can.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	can.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	can.CAN_Mode= mode;	 //ģʽ���� 
  	can.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	can.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~ CAN_BS1_16tq
  	can.CAN_BS2=tbs2;// Tbs2��ΧCAN_BS2_1tq ~ CAN_BS2_8tq
  	can.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &can);   // ��ʼ��CAN1 
    
		
		//���ù�����
 	  can_filter.CAN_FilterNumber=0;	  //������0
  	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	can_filter.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	can_filter.CAN_FilterIdHigh=0x0000;////32λID
  	can_filter.CAN_FilterIdLow=0x0000;
  	can_filter.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	can_filter.CAN_FilterMaskIdLow=0x0000;
   	can_filter.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	can_filter.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&can_filter);//�˲�����ʼ��
		nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	return 0;
}   

//		CAN_InitTypeDef        can;
//    CAN_FilterInitTypeDef  can_filter;
//    GPIO_InitTypeDef       gpio;
//    NVIC_InitTypeDef       nvic;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

//    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
//    gpio.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_Init(GPIOA, &gpio);
//    
//    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 3;
//    nvic.NVIC_IRQChannelSubPriority = 1;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
//    
//    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 1;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);    
//    
//    CAN_DeInit(CAN1);
//    CAN_StructInit(&can);
//    
//    can.CAN_TTCM = DISABLE;
//    can.CAN_ABOM = ENABLE;
//    can.CAN_AWUM = DISABLE;
//    can.CAN_NART = DISABLE;
//    can.CAN_RFLM = DISABLE;
//    can.CAN_TXFP = DISABLE;
//    can.CAN_Mode = CAN_Mode_Normal;
//    can.CAN_SJW  = CAN_SJW_1tq;
//    can.CAN_BS1 = CAN_BS1_9tq;
//    can.CAN_BS2 = CAN_BS2_4tq;
//    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
//    CAN_Init(CAN1, &can);

//		can_filter.CAN_FilterNumber=0;
//		can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
//		can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
//		can_filter.CAN_FilterIdHigh=0x0000;
//		can_filter.CAN_FilterIdLow=0x0000;
//		can_filter.CAN_FilterMaskIdHigh=0x0000;
//		can_filter.CAN_FilterMaskIdLow=0x0000;
//		can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
//		can_filter.CAN_FilterActivation=ENABLE;
//		CAN_FilterInit(&can_filter);
//    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
////    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
//		return 0;
//}   

 
u8 CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
		GPIO_InitTypeDef       GPIO_InitStructure; 
		CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   	NVIC_InitTypeDef       NVIC_InitStructure;

    //ʹ�����ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//ʹ��PORTAʱ��	            

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//�������
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//����
    GPIO_Init(GPIOB, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12����ΪCAN1
	  
		CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);
		
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=ENABLE ;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=DISABLE ;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2��ΧCAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // ��ʼ��CAN1 
    
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


	    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);


		NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	  
		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.
		return 0;		
}

	void CAN1_TX_IRQHandler(void) //CAN TX
	{
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
	{
		 CAN_ClearITPendingBit(CAN1,CAN_IT_TME);   
	}
	}

void CAN2_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
}


void CAN2_RX0_IRQHandler(void)
{
		u8 i;
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
       CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
       CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
       //������������ݴ���
       CAN2_Receive_Msg(rx_message.Data);
			 for(i=0;i<8;i++)
				CAN2_receive_buf[i]=rx_message.Data[i];	
    }
}


//�жϷ�����	
void CAN1_RX0_IRQHandler(void)
{	 	 
 		u8 i;
    CanRxMsg rx_message;		 
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
    {
//       CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
//       CAN_Receive(CAN1, CAN_FIFO0, &rx_message);  
//       //������������ݴ���
//				for(i=0;i<8;i++)
//				CAN1_receive_buf[i]=rx_message.Data[i];
    }

}



/**
************************************************************************************************************************
* @Name     : CAN1_Send_Msg
* @brief    : can����һ������(�̶���ʽ:IDΪ0X12,��׼֡,����֡)	
* @param    : len     len:���ݳ���(���Ϊ8)				  
* @retval   : 0,�ɹ�; ����,ʧ��;
* @Note     : none
************************************************************************************************************************
**/
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x251;	 // ��׼��ʶ��Ϊ0
  TxMessage.ExtId=0x12;	 // ������չ��ʾ����29λ��
  TxMessage.IDE=0;		  // ʹ����չ��ʶ��
  TxMessage.RTR=0;		  // ��Ϣ����Ϊ����֡��һ֡8λ
  TxMessage.DLC=len;							 // ������֡��Ϣ
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // ��һ֡��Ϣ          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//�ȴ����ͽ���
  if(i>=0XFFF)return 1;
  return 0;		
}


/**
************************************************************************************************************************
* @Name     : Can1_Receive_Msg
* @brief    : This function process the can message representing the encoder data received from the CAN2 bus.
* @param    : buf ���ݻ�����;	 
* @retval   : 0,�����ݱ��յ�;����,���յ����ݳ���;
* @Note     : can�ڽ������ݲ�ѯ
************************************************************************************************************************
**/
u8 CAN1_Receive_Msg(u8 *buf)
{		
		
 	u32 i;
	CanRxMsg rm1;

    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN1, CAN_FIFO0, &rm1);//��ȡ����	
	
		for(i=0;i<rm1.DLC;i++)
    buf[i]=rm1.Data[i]; 
	
	return rm1.Data[0];	
}
/**
************************************************************************************************************************
* @Name     : Can2_Receive_Msg
* @brief    : This function process the can message representing the encoder data received from the CAN2 bus.
* @param    : buf ���ݻ�����;	 
* @retval   : 0,�����ݱ��յ�;����,���յ����ݳ���;
* @Note     : can�ڽ������ݲ�ѯ
************************************************************************************************************************
**/
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg rm;
	rm.StdId=0x205;
	rm.RTR=0;
	rm.IDE=0;
    if(CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//û�н��յ�����,ֱ���˳� 
    CAN_Receive(CAN2, CAN_FIFO0, &rm);//��ȡ����	
    for(i=0;i<rm.DLC;i++)
    buf[i]=rm.Data[i];  
	return rm.Data[0];	
}

/**
************************************************************************************************************************
* @Name     : set_M3508_info
* @brief    : This function sends encoder information for the M3508 via CAN1  
*							can����һ������(�̶���ʽ:IDΪ0X200,��׼֡,����֡)	
* @param    : velocity     the velocity of the M3508 motor.
* @param    : temperature  the temperature of the M3508 motor.
* @retval   : void
* @Note     : none
************************************************************************************************************************
**/
void set_M3508_info()
{
		CanTxMsg tm;					  //CAN�ķ��ͱ��Ľṹ��
		tm.StdId=0x250;	
		tm.ExtId=0;
		tm.IDE=0;					
		tm.RTR=0;
		tm.DLC=8;
		tm.Data[0]=CAN2_receive_buf[2];	  //�߰�λת�ƵͰ�λ
		tm.Data[1]=CAN2_receive_buf[3];	  //ʮ��λ���ݸ�ֵ����λ���ݣ������Ͱ�λ
		tm.Data[2]=CAN2_receive_buf[6];
		tm.Data[3]=0;
		tm.Data[4]=0;
		tm.Data[5]=0;
		tm.Data[6]=0;
		tm.Data[7]=0;
//		tm.Data[0]=(uint8_t)(velocity>>8);//�߰�λת�ƵͰ�λ
//		tm.Data[1]=(uint8_t)(velocity);	  //ʮ��λ���ݸ�ֵ����λ���ݣ������Ͱ�λ
//		tm.Data[2]=(uint8_t)(temperature>>8);
//		tm.Data[3]=(uint8_t)(temperature);
//		tm.Data[4]=0;
//		tm.Data[5]=0;
//		tm.Data[6]=0;
//		tm.Data[7]=0;
		CAN_Transmit(CAN1 ,&tm);
}



void Set_Gimbal_Current1(CAN_TypeDef *CANx, int16_t ch_1_iq, int16_t ch_2_iq, int16_t ch_3_iq, int16_t ch_4_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = (unsigned char)(ch_1_iq >> 8);
    tx_message.Data[1] = (unsigned char)ch_1_iq;
    tx_message.Data[2] = (unsigned char)(ch_2_iq >> 8);
    tx_message.Data[3] = (unsigned char)ch_2_iq;
    tx_message.Data[4] = (unsigned char)(ch_3_iq >> 8);
    tx_message.Data[5] = (unsigned char)ch_3_iq;
    tx_message.Data[6] = (unsigned char)(ch_4_iq >> 8);
    tx_message.Data[7] = (unsigned char)ch_4_iq;
	
    CAN_Transmit(CANx,&tx_message);
}








