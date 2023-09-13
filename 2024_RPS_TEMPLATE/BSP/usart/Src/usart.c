/* Includes ------------------------------------------------------------------*/
#include "usart.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
	  
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
  *  1.��ע��printf�����ض���Ŀ�����������ȷʹ�����������������ã��������׽�����ѭ���жϣ�
  *  2.ÿ�����ڷ��ͽ��յ�buf��ַ���䶨���Ϸ����ṩ���鳤�ȵĸ��ģ�
  *  3.USART3��ֲ�ڳ����CH100���մ��ڵ����ã��ṩ�˽ӿ���ֲ���ܣ�
  *  4.���ϣ���Ч���ã����Կ�����������ͬ��д����
  *  5.ʹ��˵������ͷ�ļ���ѡ�񴮿ڹ��ܵĿ�������ͬ���������ĳ�ʼ��������ȥ����
  * 						��ͷ�ļ��궨�崦�ṩ�ص�����ͳһ�ӿڣ�����ص����������е����жϴ���
  *							
  *  
  *
  * 
  *  @�������飬���Ҽ��Ʋ�usart.h�������ÿ�����
  *
  ******************************************************************************  
  */ 
/*
����1/3��5V
*/

#if 0
#pragma import(__use_no_semihosting)  
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
	while((USART6->SR&0X40)==0);//ѭ������,ֱ���������   
	USART6 ->DR = (u8) ch;      
	return ch;
}
#endif
#if EN_USART1 
/* Variables_definination-----------------------------------------------------------------------------------------------*/
	#define  UART1_RX_BUF_LENGTH 100
	#define  UART1_TX_BUF_LENGTH 100
	uint8_t _UART1_DMA_TX_BUF[UART1_TX_BUF_LENGTH];

		#if EN_USART1_DMA_SECOND_FIFO
				uint8_t _UART1_DMA_RX_BUF[2][UART1_RX_BUF_LENGTH];
		#else
				uint8_t _UART1_DMA_RX_BUF[UART1_RX_BUF_LENGTH];
		#endif
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
void uart1_init(u32 bound)
{
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	#if EN_USART1_RX|EN_USART1_TX	
		NVIC_InitTypeDef NVIC_InitStructure;
	#endif
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//ʹ��USART1ʱ��
 
	//����1��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource7,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource6,GPIO_AF_USART1); //GPIOA9����ΪUSART1
	
	GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);

   //USART1 ��ʼ������
	USART_DeInit(USART1);
  USART_StructInit(&USART_InitStructure);
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
//		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
#endif

#if EN_USART1_DMA
		DMA_InitTypeDef dma;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	#if EN_USART1_DMA_RX
		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    DMA_DeInit(DMA2_Stream2);
    DMA_StructInit(&dma);
		
		#if EN_USART1_DMA_SECOND_FIFO
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART1_DMA_RX_BUF[0][0];
						dma.DMA_BufferSize        =   sizeof(_UART1_DMA_RX_BUF)/2;
				#else
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART1_DMA_RX_BUF;    //DMA���ջ���ַ
						dma.DMA_BufferSize        =   sizeof(_UART1_DMA_RX_BUF);       //������������
				#endif
				
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);

    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;

    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    dma.DMA_Mode = DMA_Mode_Circular;
    dma.DMA_Priority = DMA_Priority_Medium;
    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
    DMA_Init(DMA2_Stream2, &dma);
	
	//����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
				#if EN_USART1_DMA_SECOND_FIFO
					DMA_DoubleBufferModeConfig(DMA2_Stream2,  (uint32_t)&_UART1_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
				#endif
		DMA_Cmd(DMA2_Stream2, ENABLE);
	#endif
		
			USART_Cmd(USART1, ENABLE);
#endif


}



#if EN_USART1_RX_IRQ
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

#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
	static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		(void)USART1->SR;
		(void)USART1->DR;
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
			this_time_rx_len = UART1_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)UART1_RX_BUF_LENGTH;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA2_Stream2, ENABLE);
     if(this_time_rx_len == 18u)
			{
			 USART1_Data_Receive_Process;
			}
		}
		
		else 
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
			this_time_rx_len = UART1_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)UART1_RX_BUF_LENGTH;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA2_Stream2, ENABLE);
       if(this_time_rx_len == 18u)
			{
			USART1_Data_Receive_Process;
			}
		}
	}       
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 
#endif	
#endif

#if EN_USART2
/* Variables_definination-----------------------------------------------------------------------------------------------*/
	#define  UART2_RX_BUF_LENGTH 100
	#define  UART2_TX_BUF_LENGTH 100
	uint8_t _UART2_DMA_TX_BUF[UART2_TX_BUF_LENGTH];

		#if EN_UART2_DMA_SECOND_FIFO
				uint8_t _UART2_DMA_RX_BUF[2][UART2_RX_BUF_LENGTH];
		#else
				uint8_t _UART2_DMA_RX_BUF[UART2_RX_BUF_LENGTH];
		#endif
/*----------------------------------------------------------------------------------------------------------------------*/

void uart2_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef  GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
		#if EN_UART2_RX
			NVIC_InitTypeDef NVIC_InitStructure;
		#endif
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //ʹ��GPIOAʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2,ENABLE);//ʹ��USART1ʱ��

	//����4��Ӧ���Ÿ���ӳ��
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource2,GPIO_AF_USART2); //GPIOA9����ΪUSART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource3,GPIO_AF_USART2); //GPIOA10����ΪUSART1

	//UART4�˿�����
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3; //GPIOA9��GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//���ù���
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//�ٶ�50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //���츴�����
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //����
	GPIO_Init(GPIOA,&GPIO_InitStructure); //��ʼ��PA9��PA10

	//UART4 ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure); //��ʼ������1
	

		
	#if EN_UART2_DMA	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//ʹ��DMA1ʱ��
		DMA_InitTypeDef dma;
			#if EN_UART2_DMA_RX

				USART_DMACmd(USART2,USART_DMAReq_Rx,ENABLE);
				DMA_DeInit(DMA1_Stream5);
				DMA_StructInit(&dma);
				
					#if EN_UART2_DMA_SECOND_FIFO
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART2_DMA_RX_BUF[0][0];
						dma.DMA_BufferSize        =   sizeof(_UART2_DMA_RX_BUF)/2;
					#else
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART2_DMA_RX_BUF;    //DMA���ջ���ַ
						dma.DMA_BufferSize        =   sizeof(_UART2_DMA_RX_BUF);       //������������
					#endif

				dma.DMA_Channel = DMA_Channel_4;
				dma.DMA_PeripheralBaseAddr		= (uint32_t)(&USART2->DR);

				dma.DMA_DIR 									= DMA_DIR_PeripheralToMemory;

				dma.DMA_PeripheralInc 				= DMA_PeripheralInc_Disable;
				dma.DMA_MemoryInc 						= DMA_MemoryInc_Enable;
				dma.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
				dma.DMA_MemoryDataSize 				= DMA_MemoryDataSize_Byte;
				dma.DMA_Mode 									= DMA_Mode_Normal;
				dma.DMA_Priority 							= DMA_Priority_Medium;
				dma.DMA_FIFOMode 							= DMA_FIFOMode_Disable;
				dma.DMA_FIFOThreshold 				= DMA_FIFOThreshold_1QuarterFull;
				dma.DMA_MemoryBurst 					= DMA_MemoryBurst_Single;
				dma.DMA_PeripheralBurst 			= DMA_PeripheralBurst_Single;
				DMA_Init(DMA1_Stream5, &dma);
				DMA_Cmd(DMA1_Stream5, ENABLE);
				
//����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
				#if EN_UART2_DMA_SECOND_FIFO
					DMA_DoubleBufferModeConfig(DMA1_Stream5,  (uint32_t)&_UART2_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
				#endif
				
			#endif	
			#if EN_UART2_DMA_TX
					
					USART_DMACmd(USART2, USART_DMAReq_Tx, ENABLE);
					DMA_Cmd(DMA1_Stream6, DISABLE);                           // ��DMAͨ��
					DMA_DeInit(DMA1_Stream6);
					while(DMA_GetCmdStatus(DMA1_Stream6) != DISABLE) {}
					dma.DMA_Channel = DMA_Channel_4;
					dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART2->DR);
					dma.DMA_Memory0BaseAddr   	= (uint32_t)&_UART2_DMA_TX_BUF[0];
					dma.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
					dma.DMA_BufferSize			= 0;//sizeof(_USART1_DMA_TX_BUF);
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
					DMA_Init(DMA1_Stream6,&dma);				
					DMA_Cmd(DMA1_Stream6, ENABLE); 
						
					DMA_ITConfig(DMA1_Stream6,DMA_IT_TC,ENABLE);
						
					NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream6_IRQn;   // ����DMAͨ�����ж�����
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // ���ȼ�����
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);
						
			#endif
	#endif


	

			#if EN_UART2_RX	
			//USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ͷǿ��ж�
				USART_ITConfig(USART2,USART_IT_IDLE,ENABLE);  //��������֡�ж�

				NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;//����4�ж�ͨ��
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
				NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
				NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
			#endif

			USART_Cmd(USART2, ENABLE);  //ʹ�ܴ���1 		
}


   
		#if EN_UART2_RX
			uint8_t leng=0;
			void USART2_IRQHandler(void)
			{
			
			if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)    //�����ж�
			{	
				(void)USART2->SR;
				(void)USART2->DR;
				DMA_Cmd(DMA1_Stream5, DISABLE);
				DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
				leng = UART2_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream5);

				USART2_Data_Receive_Process;
				DMA_SetCurrDataCounter(DMA1_Stream5,UART2_RX_BUF_LENGTH);
				DMA_Cmd(DMA1_Stream5, ENABLE);
			}
		}
		#endif
		
		#if EN_UART2_DMA_TX_IRQ
		/**
		************************************************************************************************************************
		* @Name     : UART2_MYDMA_Enable
		* @brief    : ר���ڴ���2�ķ���һ��DMA���ݴ��亯���������÷�����ɱ�־λ�������
		* @param    : ndtr  ���ݴ�����,��֡����
		* @retval   : void
		* @Note     : ����������_UART2_DMA_TX_BUF[100]����,����DMA1_Stream4_IRQHandlerȥ�����־λ
		************************************************************************************************************************
		**/	
		void UART2_MYDMA_Enable(u16 ndtr)
		{	
			while (DMA_GetCmdStatus(DMA1_Stream6) != DISABLE){}	//ȷ��DMA���Ա�����  
				
			DMA_SetCurrDataCounter(DMA1_Stream6,ndtr);          //���ݴ�����  
		 
			DMA_Cmd(DMA1_Stream6, ENABLE);                      //����DMA���� 	
		}	 

		void DMA1_Stream6_IRQHandler(void)
		{
			//�����־
			if(DMA_GetFlagStatus(DMA1_Stream6,DMA_FLAG_TCIF6)!=RESET)//�ȴ�DMA1_Steam3�������
			{
				DMA_Cmd(DMA1_Stream6, DISABLE);                      //�ر�DMA����
				DMA_ClearFlag(DMA1_Stream6,DMA_FLAG_TCIF6);//���DMA1_Steam3������ɱ�־
			}
		}
		#endif
#endif

#if EN_USART3
/* Variables_definination-----------------------------------------------------------------------------------------------*/
   #define USART3_DMA_RX_BUF_LEN 100
	 #define USART3_DMA_TX_BUF_LEN 100
	 uint8_t  _USART3_DMA_TX_BUF[USART3_DMA_TX_BUF_LEN];
		 #if EN_UART5_DMA_SECOND_FIFO
				uint8_t _USART3_DMA_RX_BUF[2][USART3_DMA_RX_BUF_LEN];
		 #else
				uint8_t _USART3_DMA_RX_BUF[USART3_DMA_RX_BUF_LEN];
		 #endif
/*----------------------------------------------------------------------------------------------------------------------*/

void uart3_init(u32 bound)//921600
	{
	USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;

	 #if EN_USART3_RX|EN_USART3_TX
			NVIC_InitTypeDef NVIC_InitStructure;
	 #endif
	/* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USART_CH100_TX_GPIO_CLK | USART_CH100_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USART_CH100_CLK_INIT(USART_CH100_CLK, ENABLE);
  
  /* USART_CH100 GPIO configuration -----------------------------------------------*/ 
  /* Connect USART pins to AF7 */
  GPIO_PinAFConfig(USART_CH100_TX_GPIO_PORT, USART_CH100_TX_SOURCE, USART_CH100_TX_AF);
  GPIO_PinAFConfig(USART_CH100_RX_GPIO_PORT, USART_CH100_RX_SOURCE, USART_CH100_RX_AF);
  
  /* Configure USART Tx and Rx as alternate function push-pull */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  
  GPIO_InitStructure.GPIO_Pin = USART_CH100_TX_PIN;
  GPIO_Init(USART_CH100_TX_GPIO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = USART_CH100_RX_PIN;
  GPIO_Init(USART_CH100_RX_GPIO_PORT, &GPIO_InitStructure);
 
   /* Enable the USART OverSampling by 8 */
  USART_InitStructure.USART_BaudRate = bound;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART_CH100, &USART_InitStructure);
	
		#if EN_USART3_RX	

		NVIC_InitStructure.NVIC_IRQChannel = USART_CH100_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//
    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;//
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ
		NVIC_Init(&NVIC_InitStructure);
		
			/*ʹ�ܿ���֡�ж�*/
   USART_ITConfig(USART_CH100,USART_IT_IDLE,ENABLE);
// USART_ITConfig(USART_CH100,USART_IT_RXNE,ENABLE);
	 USART_ClearFlag(USART_CH100,USART_FLAG_TC|USART_FLAG_IDLE);
		#endif
		
  /* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
	#if EN_UART3_DMA
		
				/* Enable the DMA clock */
			RCC_AHB1PeriphClockCmd(USART_CH100_DMAx_CLK, ENABLE);
			DMA_InitTypeDef  DMA_InitStructure;
			
			#if EN_UART3_DMA_RX

						#if EN_USART3_DMA_SECOND_FIFO
							DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_USART3_DMA_RX_BUF[0][0];
							DMA_InitStructure.DMA_BufferSize        =   sizeof(_USART3_DMA_RX_BUF)/2;
						#else
							DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_USART3_DMA_RX_BUF;    //DMA���ջ���ַ
							DMA_InitStructure.DMA_BufferSize        =   sizeof(_USART3_DMA_RX_BUF);       //������������
						#endif	
				/* Configure DMA Initialization Structure */;
				DMA_InitStructure.DMA_FIFOMode 						 = DMA_FIFOMode_Disable ;
				DMA_InitStructure.DMA_FIFOThreshold 		 	 = DMA_FIFOThreshold_1QuarterFull ;
				DMA_InitStructure.DMA_MemoryBurst 				 = DMA_MemoryBurst_Single ;
				DMA_InitStructure.DMA_MemoryDataSize 			 = DMA_MemoryDataSize_Byte;
				DMA_InitStructure.DMA_MemoryInc 					 = DMA_MemoryInc_Enable;
				DMA_InitStructure.DMA_Mode								 = DMA_Mode_Circular;
				DMA_InitStructure.DMA_PeripheralBaseAddr 	 =(uint32_t) (&(USART_CH100->DR)) ;
				DMA_InitStructure.DMA_PeripheralBurst 		 = DMA_PeripheralBurst_Single;
				DMA_InitStructure.DMA_PeripheralDataSize   = DMA_PeripheralDataSize_Byte;
				DMA_InitStructure.DMA_PeripheralInc 			 = DMA_PeripheralInc_Disable;
				DMA_InitStructure.DMA_Priority 						 = DMA_Priority_High;
				/* Configure RX DMA */
				DMA_InitStructure.DMA_Channel 						 = USART_CH100_RX_DMA_CHANNEL ;
				DMA_InitStructure.DMA_DIR 							 	 = DMA_DIR_PeripheralToMemory ;
				DMA_Init(USART_CH100_RX_DMA_STREAM,&DMA_InitStructure);
				 /* Enable DMA USART RX Stream */
				DMA_Cmd(USART_CH100_RX_DMA_STREAM,ENABLE);
				/* Enable USART DMA RX Requsts */
				USART_DMACmd(USART_CH100, USART_DMAReq_Rx, ENABLE);
				
		//����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
						#if EN_USART3_DMA_SECOND_FIFO
							DMA_DoubleBufferModeConfig(USART_CH100_RX_DMA_STREAM,  (uint32_t)&_USART3_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
							DMA_DoubleBufferModeCmd(USART_CH100_RX_DMA_STREAM, ENABLE);
						#endif
			#endif
			
			#if EN_USART3_DMA_TX
			
				//����3���÷���DMA
				USART_DMACmd(USART_CH100, USART_DMAReq_Tx, ENABLE);   //����USART��DMA�ӿڣ�DMA1��������7��ͨ��4
				DMA_Cmd(USART_CH100_TX_DMA_STREAM, DISABLE);                 // ��DMAͨ��
				DMA_DeInit(USART_CH100_TX_DMA_STREAM);

				while(DMA_GetCmdStatus(USART_CH100_TX_DMA_STREAM) != DISABLE) {}
				DMA_InitStructure.DMA_Channel 						= USART_CH100_TX_DMA_CHANNEL;
				DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)(&USART_CH100->DR);
				DMA_InitStructure.DMA_Memory0BaseAddr   	= (uint32_t)&_USART3_DMA_TX_BUF[0];
				DMA_InitStructure.DMA_DIR 			    			= DMA_DIR_MemoryToPeripheral;
				DMA_InitStructure.DMA_BufferSize					= sizeof(_USART3_DMA_TX_BUF);   //�������ݷ��ڸ�������
				DMA_InitStructure.DMA_PeripheralInc 			= DMA_PeripheralInc_Disable;
				DMA_InitStructure.DMA_MemoryInc 					= DMA_MemoryInc_Enable;
				DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
				DMA_InitStructure.DMA_MemoryDataSize 			= DMA_MemoryDataSize_Byte;
				DMA_InitStructure.DMA_Mode 								= DMA_Mode_Normal;
				DMA_InitStructure.DMA_Priority 						= DMA_Priority_Low ;
				DMA_InitStructure.DMA_FIFOMode 						= DMA_FIFOMode_Disable;
				DMA_InitStructure.DMA_FIFOThreshold 			= DMA_FIFOThreshold_Full;
				DMA_InitStructure.DMA_MemoryBurst 				= DMA_MemoryBurst_Single;
				DMA_InitStructure.DMA_PeripheralBurst 		= DMA_PeripheralBurst_Single;
				DMA_Init(USART_CH100_TX_DMA_STREAM,&DMA_InitStructure);
				DMA_Cmd(USART_CH100_TX_DMA_STREAM, ENABLE);                        //ʹ��DMAͨ������

				NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream3_IRQn;     // DMA�����ж�
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // ���ȼ�����
				NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
				NVIC_Init(&NVIC_InitStructure);
			#endif

	#endif

  /* Enable USART */
	
  USART_Cmd(USART_CH100, ENABLE);
}
	
			#if EN_UART3_RX
			uint8_t lengt=0;
			void USART3_IRQHandler(void)
			{
			
			if(USART_GetITStatus(USART_CH100, USART_IT_IDLE) != RESET)    //�����ж�
			{	
				(void)USART_CH100->SR;
				(void)USART_CH100->DR;
				DMA_Cmd(USART_CH100_TX_DMA_STREAM, DISABLE);
				DMA_ClearFlag(USART_CH100_TX_DMA_STREAM, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
				lengt = UART3_RX_BUF_LENGTH - DMA_GetCurrDataCounter(USART_CH100_TX_DMA_STREAM);

				USART3_Data_Receive_Process;
				DMA_SetCurrDataCounter(USART_CH100_TX_DMA_STREAM,UART2_RX_BUF_LENGTH);
				DMA_Cmd(USART_CH100_TX_DMA_STREAM, ENABLE);
			}
		}
		#endif
		#if EN_UART3_DMA_TX
			/**
			************************************************************************************************************************
			* @Name     : UART3_MYDMA_Enable
			* @brief    : ר���ڴ���3�ķ���һ��DMA���ݴ��亯���������÷�����ɱ�־λ�������
			* @param    : ndtr  ���ݴ�����,��֡����
			* @retval   : void
			* @Note     : ����������_UART3_DMA_TX_BUF[100]����
			************************************************************************************************************************
			**/	
		void UART3_MYDMA_Enable(u16 ndtr)
		{	
			DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA2_Steam7������ɱ�־
			
			DMA_Cmd(DMA1_Stream3, DISABLE);                      //�ر�DMA���� 
			
			while (DMA_GetCmdStatus(DMA1_Stream3) != DISABLE){}	//ȷ��DMA���Ա�����  
				
			DMA_SetCurrDataCounter(DMA1_Stream3,ndtr);          //���ݴ�����  
		 
			DMA_Cmd(DMA1_Stream3, ENABLE);                      //����DMA���� 
				
		}	  
			#endif

#endif
#if EN_UART4
/* Variables_definination-----------------------------------------------------------------------------------------------*/
	#define  UART4_RX_BUF_LENGTH 100
	#define  UART4_TX_BUF_LENGTH 100
	uint8_t _UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];

		#if EN_UART4_DMA_SECOND_FIFO
				uint8_t _UART4_DMA_RX_BUF[2][UART4_RX_BUF_LENGTH];
		#else
				uint8_t _UART4_DMA_RX_BUF[UART4_RX_BUF_LENGTH];
		#endif
/*----------------------------------------------------------------------------------------------------------------------*/

void uart4_init(u32 bound)
{
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
		#if EN_UART4_RX
			NVIC_InitTypeDef NVIC_InitStructure;
		#endif
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
	

		
	#if EN_UART4_DMA	
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);//ʹ��DMA1ʱ��
		DMA_InitTypeDef dma;
			#if EN_UART4_DMA_RX

				USART_DMACmd(UART4,USART_DMAReq_Rx,ENABLE);
				DMA_DeInit(DMA1_Stream2);
				DMA_StructInit(&dma);
				
					#if EN_UART4_DMA_SECOND_FIFO
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART4_DMA_RX_BUF[0][0];
						dma.DMA_BufferSize        =   sizeof(_UART4_DMA_RX_BUF)/2;
					#else
						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_UART4_DMA_RX_BUF;    //DMA���ջ���ַ
						dma.DMA_BufferSize        =   sizeof(_UART4_DMA_RX_BUF);       //������������
					#endif

				dma.DMA_Channel = DMA_Channel_4;
				dma.DMA_PeripheralBaseAddr		= (uint32_t)(&UART4->DR);

				dma.DMA_DIR 									= DMA_DIR_PeripheralToMemory;

				dma.DMA_PeripheralInc 				= DMA_PeripheralInc_Disable;
				dma.DMA_MemoryInc 						= DMA_MemoryInc_Enable;
				dma.DMA_PeripheralDataSize 		= DMA_PeripheralDataSize_Byte;
				dma.DMA_MemoryDataSize 				= DMA_MemoryDataSize_Byte;
				dma.DMA_Mode 									= DMA_Mode_Normal;
				dma.DMA_Priority 							= DMA_Priority_Medium;
				dma.DMA_FIFOMode 							= DMA_FIFOMode_Disable;
				dma.DMA_FIFOThreshold 				= DMA_FIFOThreshold_1QuarterFull;
				dma.DMA_MemoryBurst 					= DMA_MemoryBurst_Single;
				dma.DMA_PeripheralBurst 			= DMA_PeripheralBurst_Single;
				DMA_Init(DMA1_Stream2, &dma);
				DMA_Cmd(DMA1_Stream2, ENABLE);
				
//����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
				#if EN_UART4_DMA_SECOND_FIFO
					DMA_DoubleBufferModeConfig(DMA1_Stream2,  (uint32_t)&_UART4_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA1_Stream2, ENABLE);
				#endif
				
			#endif	
			#if EN_UART4_DMA_TX
					
					USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);
					DMA_Cmd(DMA1_Stream4, DISABLE);                           // ��DMAͨ��
					DMA_DeInit(DMA1_Stream4);
					while(DMA_GetCmdStatus(DMA1_Stream4) != DISABLE) {}
					dma.DMA_Channel = DMA_Channel_4;
					dma.DMA_PeripheralBaseAddr	= (uint32_t)(&UART4->DR);
					dma.DMA_Memory0BaseAddr   	= (uint32_t)&_UART4_DMA_TX_BUF[0];
					dma.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
					dma.DMA_BufferSize			= 0;//sizeof(_USART1_DMA_TX_BUF);
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
						
					DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);
						
					NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream4_IRQn;   // ����DMAͨ�����ж�����
					NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // ���ȼ�����
					NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
					NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
					NVIC_Init(&NVIC_InitStructure);
						
			#endif
	#endif


	

			#if EN_UART4_RX	
			//USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//�������ͷǿ��ж�
				USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);  //��������֡�ж�

				NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//����4�ж�ͨ��
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//��ռ���ȼ�3
				NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//�����ȼ�3
				NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
				NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
			#endif

		
			USART_Cmd(UART4, ENABLE);  //ʹ�ܴ���1 
			
			
}


   
		#if EN_UART4_RX
			uint8_t length=0;
			void UART4_IRQHandler(void)
			{
			
			if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)    //�����ж�
			{	
				(void)UART4->SR;
				(void)UART4->DR;
				DMA_Cmd(DMA1_Stream2, DISABLE);
				DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
				length = UART4_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream2);
				
				USART4_Data_Receive_Process;
				DMA_SetCurrDataCounter(DMA1_Stream2,UART4_RX_BUF_LENGTH);
				DMA_Cmd(DMA1_Stream2, ENABLE);
			}
		}
		#endif
		
		#if EN_UART4_DMA_TX_IRQ
		/**
		************************************************************************************************************************
		* @Name     : UART5_MYDMA_Enable
		* @brief    : ר���ڴ���5�ķ���һ��DMA���ݴ��亯���������÷�����ɱ�־λ�������
		* @param    : ndtr  ���ݴ�����,��֡����
		* @retval   : void
		* @Note     : ����������_UART5_DMA_TX_BUF[100]����,����DMA1_Stream4_IRQHandlerȥ�����־λ
		************************************************************************************************************************
		**/	
		void UART4_MYDMA_Enable(u16 ndtr)
		{	
			while (DMA_GetCmdStatus(DMA1_Stream4) != DISABLE){}	//ȷ��DMA���Ա�����  
				
			DMA_SetCurrDataCounter(DMA1_Stream4,ndtr);          //���ݴ�����  
		 
			DMA_Cmd(DMA1_Stream4, ENABLE);                      //����DMA���� 	
		}	 

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
	
#if EN_UART5

/* Variables_definination-----------------------------------------------------------------------------------------------*/
   #define UART5_DMA_RX_BUF_LEN 100
	 #define UART5_DMA_TX_BUF_LEN 100
	 uint8_t  _UART5_DMA_TX_BUF[UART5_DMA_TX_BUF_LEN];
	 #if EN_UART5_DMA_SECOND_FIFO
			uint8_t _UART5_DMA_RX_BUF[2][UART5_DMA_RX_BUF_LEN];
	 #else
			uint8_t _UART5_DMA_RX_BUF[UART5_DMA_RX_BUF_LEN];
   #endif
/*----------------------------------------------------------------------------------------------------------------------*/	

   void uart5_init(u32 bound)
	 {
			GPIO_InitTypeDef GPIO_InitStructure;
			USART_InitTypeDef USART_InitStructure;
			#if EN_UART5_RX|EN_UART5_TX
				NVIC_InitTypeDef NVIC_InitStructure;
			#endif


			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
			RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);

			//IO��ʼ��
			GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_12;
			GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
			GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
			GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_2;
			GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
			GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
			GPIO_Init(GPIOD, &GPIO_InitStructure);
			GPIO_PinAFConfig(GPIOC,GPIO_PinSource12, GPIO_AF_UART5);
			GPIO_PinAFConfig(GPIOD,GPIO_PinSource2, GPIO_AF_UART5);
			//����5��ʼ��
			USART_InitStructure.USART_BaudRate              =   bound;
			USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx;
			USART_InitStructure.USART_Parity                =   USART_Parity_No;
			USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
			USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
			USART_Init(UART5, &USART_InitStructure);

	#if EN_UART5_DMA
					DMA_InitTypeDef DMA_uart5;
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
			
			#if EN_UART5_DMA_RX
			//����5���ý���DMA
				USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);    //����USART��DMA�ӿڣ�DMA1��������0��ͨ��4
				DMA_StructInit(&DMA_uart5);              //DMA������������ֵ

					#if EN_UART5_DMA_SECOND_FIFO 
						DMA_uart5.DMA_Memory0BaseAddr   =   (uint32_t)&_UART5_DMA_RX_BUF[0][0];
						DMA_uart5.DMA_BufferSize        =   sizeof(_UART5_DMA_RX_BUF)/2;
					#else
						DMA_uart5.DMA_Memory0BaseAddr   =   (uint32_t)&_UART5_DMA_RX_BUF;    //DMA���ջ���ַ
						DMA_uart5.DMA_BufferSize        =   sizeof(_UART5_DMA_RX_BUF);       //������������
					#endif
				DMA_uart5.DMA_Channel           =   DMA_Channel_4;
				DMA_uart5.DMA_PeripheralBaseAddr=   (uint32_t)(&UART5->DR);       //�������ַ
				DMA_uart5.DMA_DIR               =   DMA_DIR_PeripheralToMemory;   //���赽�洢��
				DMA_uart5.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;    //�����ַ������
				DMA_uart5.DMA_MemoryInc         =   DMA_MemoryInc_Enable;         //�洢����ַ����
				DMA_uart5.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;      //�洢�����ݿ��
				DMA_uart5.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;  //�������ݿ��
				DMA_uart5.DMA_Mode              =   DMA_Mode_Normal;              //�Ƿ�ѭ��ģʽ
				DMA_uart5.DMA_Priority          =   DMA_Priority_Medium  ;        //���ȼ��е�
				DMA_uart5.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
				DMA_uart5.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull;
				DMA_uart5.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;       //�洢��ͻ�������δ���
				DMA_uart5.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;   //����ͻ�������δ���
				DMA_Init(DMA1_Stream0, &DMA_uart5);
				DMA_Cmd(DMA1_Stream0, ENABLE);                        //ʹ��DMAͨ������


  //����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
				#if EN_UART5_DMA_SECOND_FIFO
					DMA_DoubleBufferModeConfig(DMA1_Stream0,  (uint32_t)&_UART5_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA1_Stream0, ENABLE);
				#endif
			#endif
			#if EN_UART5_DMA_TX

			//����5���÷���DMA
			USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);   //����USART��DMA�ӿڣ�DMA1��������7��ͨ��4
			DMA_Cmd(DMA1_Stream7, DISABLE);                 // ��DMAͨ��
			DMA_DeInit(DMA1_Stream7);

			while(DMA_GetCmdStatus(DMA1_Stream7) != DISABLE) {}
			DMA_uart5.DMA_Channel = DMA_Channel_4;
			DMA_uart5.DMA_PeripheralBaseAddr	= (uint32_t)(&UART5->DR);
			DMA_uart5.DMA_Memory0BaseAddr   	= (uint32_t)&_UART5_DMA_TX_BUF[0];
			DMA_uart5.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
			DMA_uart5.DMA_BufferSize			= sizeof(_UART5_DMA_TX_BUF);   //�������ݷ��ڸ�������
			DMA_uart5.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
			DMA_uart5.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
			DMA_uart5.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
			DMA_uart5.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
			DMA_uart5.DMA_Mode 				= DMA_Mode_Normal;
			DMA_uart5.DMA_Priority 			= DMA_Priority_Low ;
			DMA_uart5.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
			DMA_uart5.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
			DMA_uart5.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
			DMA_uart5.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
			DMA_Init(DMA1_Stream7,&DMA_uart5);
			DMA_Cmd(DMA1_Stream7, ENABLE);                        //ʹ��DMAͨ������
				
			NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;     // DMA�����ж�
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // ���ȼ�����
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

		#endif
#endif
			#if EN_UART5_RX

				NVIC_InitStructure.NVIC_IRQChannel						=	UART5_IRQn;          //����5�����ж�
				NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;
				NVIC_Init(&NVIC_InitStructure);
				
				USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);//�����ж�
			//USART_ITConfig(UART5, USART_IT_TXNE, ENABLE);//���շǿ��ж�
				USART_Cmd(UART5, ENABLE);
			#endif

				USART_Cmd(UART5, ENABLE);
 }
			#if EN_UART5_DMA_TX
			/**
			************************************************************************************************************************
			* @Name     : UART5_MYDMA_Enable
			* @brief    : ר���ڴ���5�ķ���һ��DMA���ݴ��亯���������÷�����ɱ�־λ�������
			* @param    : ndtr  ���ݴ�����,��֡����
			* @retval   : void
			* @Note     : ����������_UART5_DMA_TX_BUF[100]����
			************************************************************************************************************************
			**/	
		void UART5_MYDMA_Enable(u16 ndtr)
		{	
			DMA_ClearFlag(DMA1_Stream7,DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־
			
			DMA_Cmd(DMA1_Stream7, DISABLE);                      //�ر�DMA���� 
			
			while (DMA_GetCmdStatus(DMA1_Stream7) != DISABLE){}	//ȷ��DMA���Ա�����  
				
			DMA_SetCurrDataCounter(DMA1_Stream7,ndtr);          //���ݴ�����  
		 
			DMA_Cmd(DMA1_Stream7, ENABLE);                      //����DMA���� 
				
		}	  
			#endif

			#if EN_UART5_RX_IRQ

			void UART5_IRQHandler(void)
			{
				static uint32_t this_time_rx_len = 0;
				if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
					{
						(void)UART5->SR;
						(void)UART5->DR;

			#if EN_UART5_DMA_SECOND_FIFO 
						if(DMA_GetCurrentMemoryTarget(DMA1_Stream0) == 0)
							{
								DMA_Cmd(DMA1_Stream0, DISABLE);
								DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
								this_time_rx_len = UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

								DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);
								DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[1][0],DMA_Memory_1);
								DMA_Cmd(DMA1_Stream0, ENABLE);

								if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
									USART5_Data_Receive_Process;
							}
						//Target is Memory1
						else
							{
								DMA_Cmd(DMA1_Stream0, DISABLE);
								DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);

								this_time_rx_len =UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

								DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);
								DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[0][0],DMA_Memory_0);
								DMA_Cmd(DMA1_Stream0, ENABLE);
								
								if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
									USART5_Data_Receive_Process;
							}
				#else
						DMA_Cmd(DMA1_Stream0, DISABLE);                          //�رմ���5��DMA����ͨ��
						DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
						this_time_rx_len = UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0); //��ȡDMA_GetCurrDataCounterʣ��������

						DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);      //���õ�ǰDMAʣ��������
						DMA_Cmd(DMA1_Stream0, ENABLE);                                       //��������5��DMA����ͨ��

						if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
								 USART5_Data_Receive_Process //_UART5_DMA_RX_BUF��ΪDMA���ջ�����
			//		
			#endif
				}
			}
			#endif

#endif
#if EN_UART6
/* Variables_definination-----------------------------------------------------------------------------------------------*/
   #define UART6_DMA_RX_BUF_LEN 100
	 #define UART6_DMA_TX_BUF_LEN 100
	 uint8_t  _UART6_DMA_TX_BUF[UART6_DMA_TX_BUF_LEN];
	 #if EN_UART6_DMA_SECOND_FIFO
			uint8_t _UART6_DMA_RX_BUF[2][UART6_DMA_RX_BUF_LEN];
	 #else
			uint8_t _UART6_DMA_RX_BUF[UART6_DMA_RX_BUF_LEN];
   #endif
/*----------------------------------------------------------------------------------------------------------------------*/	

   void uart6_init(u32 bound)
	 {
			GPIO_InitTypeDef GPIO_InitStructure;
			USART_InitTypeDef USART_InitStructure;
			#if EN_UART6_RX|EN_UART6_TX
				NVIC_InitTypeDef NVIC_InitStructure;
			#endif


			RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
			RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE);

			//IO��ʼ��
			GPIO_InitStructure.GPIO_Mode   =   GPIO_Mode_AF;
			GPIO_InitStructure.GPIO_OType  =   GPIO_OType_PP;
			GPIO_InitStructure.GPIO_Pin    =   GPIO_Pin_6|GPIO_Pin_7;
			GPIO_InitStructure.GPIO_PuPd   =   GPIO_PuPd_UP;
			GPIO_InitStructure.GPIO_Speed  =   GPIO_Speed_100MHz;
			GPIO_Init(GPIOC, &GPIO_InitStructure);
		
			GPIO_PinAFConfig(GPIOC,GPIO_PinSource6, GPIO_AF_USART6);
			GPIO_PinAFConfig(GPIOC,GPIO_PinSource7, GPIO_AF_USART6);
		
		//����5��ʼ��
			USART_DeInit(USART6);
			USART_InitStructure.USART_BaudRate              =   bound;
			USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
			USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx;
			USART_InitStructure.USART_Parity                =   USART_Parity_No;
			USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
			USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
			USART_Init(USART6, &USART_InitStructure);

	#if EN_UART6_DMA
					DMA_InitTypeDef DMA_uart6;
					RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
			
			#if EN_UART6_DMA_RX
			//����5���ý���DMA
				USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);    //����USART��DMA�ӿڣ�DMA1��������0��ͨ��4
				DMA_StructInit(&DMA_uart6);              //DMA������������ֵ

					#if EN_UART6_DMA_SECOND_FIFO 
						DMA_uart6.DMA_Memory0BaseAddr   =   (uint32_t)&_UART6_DMA_RX_BUF[0][0];
						DMA_uart6.DMA_BufferSize        =   sizeof(_UART6_DMA_RX_BUF)/2;
					#else
						DMA_uart6.DMA_Memory0BaseAddr   =   (uint32_t)&_UART6_DMA_RX_BUF;    //DMA���ջ���ַ
						DMA_uart6.DMA_BufferSize        =   sizeof(_UART6_DMA_RX_BUF);       //������������
					#endif
				DMA_uart6.DMA_Channel           =   DMA_Channel_5;
				DMA_uart6.DMA_PeripheralBaseAddr=   (uint32_t)(&USART6->DR);       //�������ַ
				DMA_uart6.DMA_DIR               =   DMA_DIR_PeripheralToMemory;   //���赽�洢��
				DMA_uart6.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;    //�����ַ������
				DMA_uart6.DMA_MemoryInc         =   DMA_MemoryInc_Enable;         //�洢����ַ����
				DMA_uart6.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;      //�洢�����ݿ��
				DMA_uart6.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;  //�������ݿ��
				DMA_uart6.DMA_Mode              =   DMA_Mode_Normal;              //�Ƿ�ѭ��ģʽ
				DMA_uart6.DMA_Priority          =   DMA_Priority_Medium  ;        //���ȼ��е�
				DMA_uart6.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
				DMA_uart6.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull;
				DMA_uart6.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;       //�洢��ͻ�������δ���
				DMA_uart6.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;   //����ͻ�������δ���
				DMA_Init(DMA2_Stream1, &DMA_uart6);
				DMA_Cmd(DMA2_Stream1, ENABLE);                        //ʹ��DMAͨ������


  //����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
				#if EN_UART6_DMA_SECOND_FIFO
					DMA_DoubleBufferModeConfig(DMA2_Stream1,  (uint32_t)&_UART6_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA2_Stream1, ENABLE);
				#endif
			#endif
			#if EN_UART6_DMA_TX

			//����5���÷���DMA
			USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);   //����USART��DMA�ӿڣ�DMA1��������7��ͨ��4
			DMA_Cmd(DMA2_Stream7, DISABLE);                 // ��DMAͨ��
			DMA_DeInit(DMA2_Stream7);

			while(DMA_GetCmdStatus(DMA2_Stream7) != DISABLE) {}
			DMA_uart6.DMA_Channel = DMA_Channel_5;
			DMA_uart6.DMA_PeripheralBaseAddr	= (uint32_t)(&USART6->DR);
			DMA_uart6.DMA_Memory0BaseAddr   	= (uint32_t)&_UART6_DMA_TX_BUF[0];
			DMA_uart6.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
			DMA_uart6.DMA_BufferSize			= sizeof(_UART6_DMA_TX_BUF);   //�������ݷ��ڸ�������
			DMA_uart6.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
			DMA_uart6.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
			DMA_uart6.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
			DMA_uart6.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
			DMA_uart6.DMA_Mode 				= DMA_Mode_Normal;
			DMA_uart6.DMA_Priority 			= DMA_Priority_Low ;
			DMA_uart6.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
			DMA_uart6.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
			DMA_uart6.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
			DMA_uart6.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
			DMA_Init(DMA2_Stream7,&DMA_uart6);
			DMA_Cmd(DMA2_Stream7, ENABLE);                        //ʹ��DMAͨ������
				
			NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;     // DMA�����ж�
			NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // ���ȼ�����
			NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
			NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
			NVIC_Init(&NVIC_InitStructure);

		#endif
#endif
			#if EN_UART6_RX

				NVIC_InitStructure.NVIC_IRQChannel						=	USART6_IRQn;          //����5�����ж�
				NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
				NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
				NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;
				NVIC_Init(&NVIC_InitStructure);
				
				USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);//�����ж�
			//USART_ITConfig(USART6, USART_IT_TXNE, ENABLE);//���շǿ��ж�
				USART_Cmd(USART6, ENABLE);
			#endif

				USART_Cmd(USART6, ENABLE);
 }
			#if EN_UART6_DMA_TX
			/**
			************************************************************************************************************************
			* @Name     : UART5_MYDMA_Enable
			* @brief    : ר���ڴ���5�ķ���һ��DMA���ݴ��亯���������÷�����ɱ�־λ�������
			* @param    : ndtr  ���ݴ�����,��֡����
			* @retval   : void
			* @Note     : ����������_UART5_DMA_TX_BUF[100]����
			************************************************************************************************************************
			**/	
		void UART6_MYDMA_Enable(u16 ndtr)
		{	
			DMA_ClearFlag(DMA2_Stream7,DMA_FLAG_TCIF7);//���DMA2_Steam7������ɱ�־
			
			DMA_Cmd(DMA2_Stream7, DISABLE);                      //�ر�DMA���� 
			
			while (DMA_GetCmdStatus(DMA2_Stream7) != DISABLE){}	//ȷ��DMA���Ա�����  
				
			DMA_SetCurrDataCounter(DMA2_Stream7,ndtr);          //���ݴ�����  
		 
			DMA_Cmd(DMA2_Stream7, ENABLE);                      //����DMA���� 
				
		}	  
			#endif

			#if EN_UART6_RX_IRQ

			void UART6_IRQHandler(void)
			{
				static uint32_t this_time_rx_len = 0;
				if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
					{
						(void)USART6->SR;
						(void)USART6->DR;

			#if EN_UART6_DMA_SECOND_FIFO 
						if(DMA_GetCurrentMemoryTarget(DMA1_Stream0) == 0)
							{
								DMA_Cmd(DMA1_Stream0, DISABLE);
								DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
								this_time_rx_len = UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

								DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);
								DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[1][0],DMA_Memory_1);
								DMA_Cmd(DMA1_Stream0, ENABLE);

								if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
									USART6_Data_Receive_Process;
							}
						//Target is Memory1
						else
							{
								DMA_Cmd(DMA1_Stream0, DISABLE);
								DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);

								this_time_rx_len =UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

								DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);
								DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[0][0],DMA_Memory_0);
								DMA_Cmd(DMA1_Stream0, ENABLE);
								
								if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
									USART6_Data_Receive_Process;
							}
				#else
						DMA_Cmd(DMA1_Stream0, DISABLE);                          //�رմ���5��DMA����ͨ��
						DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
						this_time_rx_len = UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0); //��ȡDMA_GetCurrDataCounterʣ��������

						DMA_SetCurrDataCounter(DMA1_Stream0, UART5_DMA_RX_BUF_LEN);      //���õ�ǰDMAʣ��������
						DMA_Cmd(DMA1_Stream0, ENABLE);                                       //��������5��DMA����ͨ��

						if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
							USART6_Data_Receive_Process;  //_UART5_DMA_RX_BUF��ΪDMA���ջ�����
			//		
			#endif
				}
			}
			#endif
#endif
