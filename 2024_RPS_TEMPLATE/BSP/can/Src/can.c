/* Includes ------------------------------------------------------------------*/
#include "can.h"
/**
  ******************************************************************************
  * @file    can.c
  * @author  TC
  * @version V1.0.0
  * @date    07-September-2023
  * @brief   This file provides firmware functions to manage the following 
  *          functionalities of the Controller area network (CAN) peripheral:
  *           + Initialization and Configuration 
  *           + CAN Frames Transmission
  *           + CAN Frames Reception
  *           + Operation modes switch
  *           + Error management
  *           + Interrupts and flags
  *
@verbatim
 ===============================================================================
                        ##### How to use #####
 ===============================================================================
    [..]
      (#) Enable the CAN controller interface clock using 
          RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE); for CAN1 
          and RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE); for CAN2
      -@- �����ֻʹ��CAN2�����������CAN1ʱ��
       
      (#) CAN pins configuration
        (++) ����CAN gpioʱ�ӣ�ʹ�����¹���:
							RCC_AHB1PeriphClockCmd (RCC_AHB1Periph_GPIOx,ENABLE); 
        (++) ʹ�����¹��ܽ��漰��CAN�������ӵ�AF9
							GPIO_PinAFConfig(GPIOx, GPIO_PinSourcex, GPIO_AF_CANx);
        (++) ͨ�����ý���ЩCAN��������Ϊ���ù���ģʽ
							����GPIO_Init();
							ʹ��CAN_Init()�ͳ�ʼ��������CAN
							CAN_FilterInit()������
			(#)ʹ��CAN_Transmit()�������������CAN֡��

			(#)ʹ��CAN_TransmitStatus()���CAN֡�Ĵ��亯����

			(#)ʹ��CAN_CancelTransmit()ȡ��CAN֡�Ĵ��亯����
				 ʹ��can_receive()��������һ��CAN֡��

		  (#)ʹ��CAN_FIFORelease()�����ͷŽ���fifo��

			(#)���صȴ�����֡�ĸ���
					CAN_MessagePending()������
											 
      (#) Ҫ����CAN�¼�������ʹ���������ַ���֮һ:
        (++) ʹ��CAN_GetFlagStatus()�������CAN��־�� 
        (++) ͨ����ʼ���׶ε�CAN_ITConfig()�������ж������е�CAN_GetITStatus()����ʹ��CAN�ж�������¼��Ƿ�����
             ���һ����־����Ӧ��ʹ��CAN_ClearFlag()������������ڼ���ж��¼�����Ӧ��ʹ��CAN_ClearITPendingBit()�����������  

@endverbatim
           
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2014 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************  
  */
	
 
/* Variables_definination-----------------------------------------------------------------------------------------------*/
u8 CAN1_receive_buf[20];
u8 CAN2_receive_buf[20];
/*----------------------------------------------------------------------------------------------------------------------*/

/**
************************************************************************************************************************
* @Name     : CAN1_Mode_Init
* @brief    : This function initializes the CAN1 struct, the struct of CAN_Filter, and the NVIC configuration
* @param    : tbs2    tbs2:ʱ���2��ʱ�䵥Ԫ.  Range from:CAN_BS2_1tq to CAN_BS2_8tq;
*	@param		:	tbs1		tbs1:ʱ���1��ʱ�䵥Ԫ.  Range from:CAN_BS1_1tq to CAN_BS1_16tq;
*	@param		:	brp			brp :�����ʷ�Ƶ��.       Range from:1 to 1024;                   tq=(brp)*tpclk1
* @param		: mode 		mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
* @retval   : 0,��ʼ��OK;����,��ʼ��ʧ��; 
* @Note     : ������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
*							Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
*							������Ϊ:42M/((6+7+1)*6)=500Kbps
************************************************************************************************************************
**/
u8 CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef       GPIO_InitStructure; 
	  CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN1_RX0_INT_ENABLE|CAN1_TX0_INT_ENABLE
   	NVIC_InitTypeDef       NVIC_InitStructure;
#endif
    //ʹ�����ʱ��
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��PORTAʱ��	            

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//ʹ��CAN1ʱ��	
	
    //��ʼ��GPIO
	  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
    GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��PA11,PA12
	
	  //���Ÿ���ӳ������
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11����ΪCAN1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12����ΪCAN1
	  
		CAN_DeInit(CAN1);
    CAN_StructInit(&CAN_InitStructure);
		
  	//CAN��Ԫ����
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//��ʱ�䴥��ͨ��ģʽ   
  	CAN_InitStructure.CAN_ABOM=ENABLE;	//����Զ����߹���	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//˯��ģʽͨ���������(���CAN->MCR��SLEEPλ)
  	CAN_InitStructure.CAN_NART=DISABLE;	//��ֹ�����Զ����� 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//���Ĳ�����,�µĸ��Ǿɵ�  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//���ȼ��ɱ��ı�ʶ������ 
  	CAN_InitStructure.CAN_Mode= mode;	 //ģʽ���� 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//����ͬ����Ծ���(Tsjw)Ϊtsjw+1��ʱ�䵥λ CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1��ΧCAN_BS1_1tq ~ CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;// Tbs2��ΧCAN_BS2_1tq ~ CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //��Ƶϵ��(Fdiv)Ϊbrp+1	
  	CAN_Init(CAN1, &CAN_InitStructure);   // ��ʼ��CAN1 
    
		
		//���ù�����
 	  CAN_FilterInitStructure.CAN_FilterNumber=0;	  //������0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32λ 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32λID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32λMASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//������0������FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //���������0
  	CAN_FilterInit(&CAN_FilterInitStructure);//�˲�����ʼ��

#if CAN1_RX0_INT_ENABLE
	  CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);//FIFO0��Ϣ�Һ��ж�����.		    
  	NVIC_InitStructure.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);	
#endif
#if CAN1_TX0_INT_ENABLE
		NVIC_InitStructure.NVIC_IRQChannel = CAN1_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif
		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	return 0;
}   
 
u8 CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

  	GPIO_InitTypeDef       GPIO_InitStructure; 
		CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
#if CAN2_RX0_INT_ENABLE|CAN2_TX0_INT_ENABLE
   	NVIC_InitTypeDef       NVIC_InitStructure;
#endif
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

#if CAN2_RX0_INT_ENABLE
	    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // �����ȼ�Ϊ1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;            // �����ȼ�Ϊ0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);
#endif
#if CAN2_TX0_INT_ENABLE
		NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif	  
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

#if CAN2_RX0_INT_ENABLE
void CAN2_RX0_IRQHandler(void)
{
		u8 i;
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
       CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
       CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
       //������������ݴ���
       CAN2_Data_Receive_Process
			 for(i=0;i<8;i++)
				CAN2_receive_buf[i]=rx_message.Data[i];	
    }
}
#endif
#if CAN1_RX0_INT_ENABLE	//ʹ��RX0�ж�
//�жϷ�����	
void CAN1_RX0_IRQHandler(void)
{	 	 
 		u8 i;
    CanRxMsg rx_message;		 
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
    {
       CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
       CAN_Receive(CAN1, CAN_FIFO0, &rx_message);  
       //������������ݴ���
			 CAN1_Data_Receive_Process
				for(i=0;i<8;i++)
				CAN1_receive_buf[i]=rx_message.Data[i];
			}
}


#endif


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








