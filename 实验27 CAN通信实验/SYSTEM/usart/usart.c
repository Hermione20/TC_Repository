/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif	  
/**
  ******************************************************************************
  * @file    usart.c
  * @author  TC
  * @version V1.0.0
  * @date    07-September-2023
  * @brief   ���ļ��ṩ�̼�����������ͨ��ͬ���첽���շ�����(USART)�����¹���:
  *           + Initialization and Configuration
  *           + Data transfers
  *           + Multi-Processor Communication
  *           + LIN mode
  *           + Half-duplex mode
  *           + Smartcard mode
  *           + IrDA mode
  *           + DMA transfers management
  *           + Interrupts and flags management 
  *           
  @verbatim       
 ===============================================================================
                        ##### How to use this driver #####
 ===============================================================================
    [..]
      (#)ʹ�����¹��ܿ�������ʱ��
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTx, ENABLE)����USART1��USART6
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USARTx, ENABLE)����USART2, USART3��UART4��UART5��
  
      (#) ����USARTģʽ��ʹ��RCC_AHB1PeriphClockCmd()������I/O������TX, RX, CTS����/��SCLK)��
  
      (#) Peripheral's alternate function: 
        (++) ʹ��GPIO_PinAFConfig()�������������ӵ���������ı��ù���(AF)
        (++) ͨ�����·�ʽ���ñ��ù����е���������:
            GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
        (++) ѡ�����ͣ�����/����������ٶ�ͨ��
            GPIO_PuPd, GPIO_OType and GPIO_Speed members
        (++) Call GPIO_Init() function
          
      (#) ʹ��USART_Init()������̲����ʣ��ֳ���ֹͣλ����żУ�飬Ӳ�������ƺ�ģʽ(������/������)��
  
      (#) ����ͬ��ģʽ������ʱ�Ӳ�ʹ��USART_ClockInit()�����Լ��ԡ���λ�����һλ���б�̡�
  
      (#) �����Ҫʹ���ж�ģʽ����ʹ��USART_ITConfig()��������NVIC����Ӧ���жϡ�
  
      (#) When using the DMA mode 
        (++) ʹ��DMA Init()��������DMA
        (++) ʹ��USART_DMACmd()�������������ͨ������
   
      (#) ʹ��USART_Cmd()��������USART��
   
      (#) ��ʹ��DMAģʽʱ��ʹ��DMA_Cmd()��������DMA��
    
      -@- ����Ķദ������LIN����˫�������ܿ���IrDA�Ӳ���
					��֪����
    
    [..]        
				Ϊ�˴ﵽ���ߵ�ͨ�Ų����ʣ�����ʹ��USART_OverSampling8Cmd()��������8ģʽ�Ĺ�������
				�������Ӧ��������USARTʱ��(RCC_APBxPeriphClockCmd())֮���ڵ���USART_Init()����֮ǰ���á�


            
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

/**
************************************************************************************************************************
* @Name     : fputc/_sys_exit
* @brief    : �������´���,֧��printf����,������Ҫѡ��use MicroLIB
* @param    : ch
* @param    : FILE *f
* @retval   : void
* @Note     : �������´���,֧��printf����,������Ҫѡ��use MicroLIB
************************************************************************************************************************
**/ 
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1   //���ʹ����

/* Variables_definination-----------------------------------------------------------------------------------------------*/

u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���

/*----------------------------------------------------------------------------------------------------------------------*/


/**
************************************************************************************************************************
* @Name     : uart_init
* @brief    : This function process the can message representing the encoder data received from the CAN2 bus.
* @param    : bound     the baud rate setting,bound:������
* @retval   : void
* @Note     : ��ʼ��IO ����1 
************************************************************************************************************************
**/
void uart1_init(u32 bound){
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10����ΪUSART1
	
	//USART1�˿�����
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

   //USART1 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
  USART_Cmd(USART1, ENABLE);  //ʹ�ܴ���1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�

	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���

#endif
}
#if EN_USART1_RX
/**
************************************************************************************************************************
* @Name     : USART1_IRQHandler
* @brief    : ����1�жϷ������
* @param    : none
* @retval   : void
* @Note     : ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	
************************************************************************************************************************
**/
void USART1_IRQHandler(void)                	//����1�жϷ������
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
		{
			if(USART_RX_STA&0x4000)//���յ���0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else USART_RX_STA|=0x8000;	//��������� 
			}
			else //��û�յ�0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 
#endif	
#endif

#if EN_UART4
void uart4_init(u32 bound){
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//ʹ��USART1ʱ��

	//����4��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOA10����ΪUSART1

	//UART4�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOC,&GPIO_InitStructure); //��ʼ��PA9��PA10

	//UART4 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(UART4, &USART_InitStructure); //��ʼ������1
	
#if EN_UART4_DMA_RX
	USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
	
	DMA_InitTypeDef dma;
	DMA_DeInit(DMA1_Stream2);
	DMA_StructInit(&dma);
	dma.DMA_Channel = DMA_Channel_4;
	dma.DMA_PeripheralBaseAddr		= (uint32_t)(&UART4->DR);
	dma.DMA_Memory0BaseAddr   		= (uint32_t)&UART4_DMA_RX_BUF;
	dma.DMA_DIR 					= DMA_DIR_PeripheralToMemory;
	dma.DMA_BufferSize				= UART4_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
	dma.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
	dma.DMA_MemoryInc 				= DMA_MemoryInc_Enable;
	dma.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
	dma.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
	dma.DMA_Mode 					= DMA_Mode_Normal;
	dma.DMA_Priority 				= DMA_Priority_Medium;
	dma.DMA_FIFOMode 				= DMA_FIFOMode_Disable;
	dma.DMA_FIFOThreshold 			= DMA_FIFOThreshold_1QuarterFull;
	dma.DMA_MemoryBurst 			= DMA_MemoryBurst_Single;
	dma.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream2, &dma);
	DMA_Cmd(DMA1_Stream2, ENABLE);
#endif	
#if EN_UART4_DMA_TX
	  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
	  DMA_Cmd(DMA1_Stream4, DISABLE);                           // ��DMAͨ��
	  DMA_DeInit(DMA1_Stream4);
	  while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}
	  dma.DMA_Channel = DMA_Channel_4;
	  dma.DMA_PeripheralBaseAddr	= (uint32_t)(&UART4->DR);
	  dma.DMA_Memory0BaseAddr   	= (uint32_t)&UART4_DMA_TX_BUF[0];
	  dma.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
	  dma.DMA_BufferSize			= 0;//sizeof(USART1_DMA_TX_BUF);
	  dma.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
	  dma.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
	  dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
	  dma.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
	  dma.DMA_Mode 				= DMA_Mode_Normal;
	  dma.DMA_Priority 			= DMA_Priority_Medium;
	  dma.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
	  dma.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
	  dma.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
	  dma.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
	  DMA_Init(DMA1_Stream4,&dma);
#endif
#if EN_UART4_DMA_TX_IQR		  
	nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;   // ����DMAͨ�����ж�����
	nvic.NVIC_IRQChannelPreemptionPriority = 3;     // ���ȼ�����
	nvic.NVIC_IRQChannelSubPriority = 3;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
    DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);
#endif	
#if EN_UART4_RX	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�������ͷǿ��ж�
  //USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);  //��������֡�ж�

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//����4�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
#endif

  USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���1 
}



#if EN_UART4_RX
	void UART4_IRQHandler(void)
	{
	uint8_t length=0;
	if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)    //�����ж�
	{
	  (void)UART4->SR;
	  (void)UART4->DR;
	  DMA_Cmd(DMA1_Stream2, DISABLE);
	  DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
	  length = UART4_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream2);

//	  targetOffsetDataDeal(length, UART4_DMA_RX_BUF );
	  DMA_SetCurrDataCounter(DMA1_Stream2,UART4_RX_BUF_LENGTH);
	  DMA_Cmd(DMA1_Stream2, ENABLE);
	}
}
#endif
#if EN_UART4_DMA_TX_IRQ
	void DMA1_Stream4_IRQHandler(void)
	{
	//�����־
	if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//�ȴ�DMA1_Steam3�������
	{
	  DMA_Cmd(DMA1_Stream4, DISABLE);                      //�ر�DMA����
	  DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//���DMA1_Steam3������ɱ�־
	}
	}
#endif
#endif

	
	