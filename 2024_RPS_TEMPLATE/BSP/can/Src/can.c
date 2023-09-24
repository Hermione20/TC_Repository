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
#include "main.h"


#if EN_CAN1


/*----CAN1_TX-----PA12----*/
/*----CAN1_RX-----PA11----*/
void CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{   
		GPIO_InitTypeDef       gpio;
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOA, &gpio);
    
    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
//    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 1;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);    
    
    CAN_DeInit(CAN1);
    CAN_StructInit(&can);
    
    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;
    can.CAN_AWUM = DISABLE;
    can.CAN_NART = DISABLE;
    can.CAN_RFLM = DISABLE;
    can.CAN_TXFP = DISABLE;
    can.CAN_Mode = mode;
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = tbs1;
    can.CAN_BS2 = tbs2;
    can.CAN_Prescaler = brp;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN1, &can);

		can_filter.CAN_FilterNumber=0;
		can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
		can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
		can_filter.CAN_FilterIdHigh=0x0000;
		can_filter.CAN_FilterIdLow=0x0000;
		can_filter.CAN_FilterMaskIdHigh=0x0000;
		can_filter.CAN_FilterMaskIdLow=0x0000;
		can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
		can_filter.CAN_FilterActivation=ENABLE;
		CAN_FilterInit(&can_filter);
    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
//    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
}

void CAN1_TX_IRQHandler(void) //CAN TX
{
    if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET) 
		{
				CAN_ClearITPendingBit(CAN1,CAN_IT_TME);
		}
}

/*************************************************************************
                          CAN1_RX0_IRQHandler
������CAN1�Ľ����жϺ���
*************************************************************************/
void CAN1_RX0_IRQHandler(void)
{   
		CanRxMsg rx_message;	
		if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET)
		{
				CAN_ClearITPendingBit(CAN1, CAN_IT_FF0);
				CAN_ClearFlag(CAN1, CAN_FLAG_FF0); 
				CAN_Receive(CAN1, CAN_FIFO0, &rx_message);
				CAN1_Data_Receive_Process
		}
}
#endif

#if EN_CAN2
/*----CAN2_TX-----PB13----*/
/*----CAN2_RX-----PB12----*/

int16_t  pitch_ecd_bias =6000;
int16_t  yaw_ecd_bias  = 5000;

void CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
    CAN_InitTypeDef        can;
    CAN_FilterInitTypeDef  can_filter;
    GPIO_InitTypeDef       gpio;
    NVIC_InitTypeDef       nvic;

    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);

    GPIO_PinAFConfig(GPIOB, GPIO_PinSource12, GPIO_AF_CAN2);
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource13, GPIO_AF_CAN2); 

    gpio.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 ;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    GPIO_Init(GPIOB, &gpio);

    nvic.NVIC_IRQChannel = CAN2_RX0_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);
    
    nvic.NVIC_IRQChannel = CAN2_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 3;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

    CAN_DeInit(CAN2);
    CAN_StructInit(&can);

    can.CAN_TTCM = DISABLE;
    can.CAN_ABOM = ENABLE;    
    can.CAN_AWUM = DISABLE;    
    can.CAN_NART = DISABLE;    
    can.CAN_RFLM = DISABLE;    
    can.CAN_TXFP = DISABLE;     
    can.CAN_Mode = CAN_Mode_Normal; 
    can.CAN_SJW  = CAN_SJW_1tq;
    can.CAN_BS1 = tbs1;
    can.CAN_BS2 = tbs2;
    can.CAN_Prescaler = brp;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
    CAN_Init(CAN2, &can);
    
    can_filter.CAN_FilterNumber=14;
    can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
    can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
    can_filter.CAN_FilterIdHigh=0x0000;
    can_filter.CAN_FilterIdLow=0x0000;
    can_filter.CAN_FilterMaskIdHigh=0x0000;
    can_filter.CAN_FilterMaskIdLow=0x0000;
    can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
    can_filter.CAN_FilterActivation=ENABLE;
    CAN_FilterInit(&can_filter);
    
    CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);
//    CAN_ITConfig(CAN2,CAN_IT_TME,ENABLE);
}

void CAN2_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
//	CAN_ClearITPendingBit(CAN2, CAN_IT_EWG|CAN_IT_EPV|CAN_IT_BOF|CAN_IT_LEC|CAN_IT_ERR);

}

int abbbb;
void CAN2_RX0_IRQHandler(void)
{abbbb++;
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
        CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
        CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
       //������������ݴ���
       
				CAN2_Data_Receive_Process
//			CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);
    }
		
//		CAN_ClearITPendingBit(CAN2, CAN_IT_EWG|CAN_IT_EPV|CAN_IT_BOF|CAN_IT_LEC|CAN_IT_ERR);
}

#endif
