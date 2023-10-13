#include "main.h"

/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/

void CAN1_Configuration(void)
{
//    CAN_InitTypeDef        can;
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
////    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
////    nvic.NVIC_IRQChannelPreemptionPriority = 1;
////    nvic.NVIC_IRQChannelSubPriority = 0;
////    nvic.NVIC_IRQChannelCmd = ENABLE;
////    NVIC_Init(&nvic);    
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
  	can.CAN_Mode= CAN_Mode_Normal;	 //ģʽ���� 
  	can.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	can.CAN_BS1=CAN_BS1_9tq; //Tbs1��ΧCAN_BS1_1tq ~ CAN_BS1_16tq
  	can.CAN_BS2=CAN_BS2_4tq;// Tbs2��ΧCAN_BS2_1tq ~ CAN_BS2_8tq
  	can.CAN_Prescaler=3;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
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
//		NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
//    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
//    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
//    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&NVIC_InitStructure);

		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE); 
}

//void CAN1_TX_IRQHandler(void) //CAN TX
//{
//    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
//		{
//				CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
//		}
//}

/*************************************************************************
                          CAN1_RX0_IRQHandler
������CAN1�Ľ����жϺ���
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{   
		CanRxMsg rx_message;	
		if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
		{
//				CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
//				CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
//				CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
//				Can1ReceiveMsgProcess(&rx_message);
		}
}




