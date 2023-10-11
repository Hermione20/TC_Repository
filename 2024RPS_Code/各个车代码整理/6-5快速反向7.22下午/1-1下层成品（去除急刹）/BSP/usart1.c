
#include "main.h"

/*-----USART1_RX-----PB7----*/ 

/*
*********************************************************************************************************
*                                               VARIABLES
*********************************************************************************************************
*/
static uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];
static uint8_t _USART1_RX_BUF[BSP_USART1_RX_BUF_SIZE_IN_FRAMES * BSP_USART1_DMA_RX_BUF_LEN];
static FIFO_t  _USART1_RX_FIFO;

//void USART1_Configuration(uint32_t baud_rate)
//{
//	GPIO_InitTypeDef gpio;
//	USART_InitTypeDef usart;
//	NVIC_InitTypeDef nvic;
//	DMA_InitTypeDef dma;
//    
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
//	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
//    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
//	
//    GPIO_StructInit(&gpio);
//    gpio.GPIO_Pin = GPIO_Pin_7;
//    gpio.GPIO_Mode = GPIO_Mode_AF;
//    gpio.GPIO_Speed = GPIO_Speed_2MHz;
//    gpio.GPIO_PuPd = GPIO_PuPd_UP;
//    GPIO_Init(GPIOB, &gpio);
//    
//    USART_DeInit(USART1);
//    USART_StructInit(&usart);
//    usart.USART_BaudRate = baud_rate;
//    usart.USART_WordLength = USART_WordLength_8b;
//    usart.USART_StopBits = USART_StopBits_1;
//    usart.USART_Parity = USART_Parity_Even;
//    usart.USART_Mode = USART_Mode_Rx;
//    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_Init(USART1, &usart);
////    USART1_FIFO_Init();
//    
//    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
//    
//    DMA_DeInit(DMA2_Stream2);
//    DMA_StructInit(&dma);
//    dma.DMA_Channel = DMA_Channel_4;
//    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
//    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART1_DMA_RX_BUF[0][0];
//    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
//    dma.DMA_BufferSize = sizeof(_USART1_DMA_RX_BUF)/2;
//    dma.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
//    dma.DMA_MemoryInc = DMA_MemoryInc_Enable;
//    dma.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//    dma.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
//    dma.DMA_Mode = DMA_Mode_Circular;
//    dma.DMA_Priority = DMA_Priority_Medium;
//    dma.DMA_FIFOMode = DMA_FIFOMode_Disable;
//    dma.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
//    dma.DMA_MemoryBurst = DMA_MemoryBurst_Single;
//    dma.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
//    DMA_Init(DMA2_Stream2, &dma);
//    
//    DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)&_USART1_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
//    DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
//    DMA_Cmd(DMA2_Stream2, ENABLE);
//    
//	nvic.NVIC_IRQChannel = USART1_IRQn;                          
//	nvic.NVIC_IRQChannelPreemptionPriority = 0;   //pre-emption priority 
//	nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
//	nvic.NVIC_IRQChannelCmd = ENABLE;			
//	NVIC_Init(&nvic);	

//	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
//	USART_Cmd(USART1, ENABLE);

//}

void USART1_Configuration(uint32_t baud_rate)
{
   //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	
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
	USART_InitStructure.USART_BaudRate = baud_rate;//����������
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	

	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//��������ж�
//		USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	//Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//����1�ж�ͨ��
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=0;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =0;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���



		DMA_InitTypeDef dma;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);

		USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);

    DMA_DeInit(DMA2_Stream2);
    DMA_StructInit(&dma);
		

						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_USART1_DMA_RX_BUF[0][0];
						dma.DMA_BufferSize        =   sizeof(_USART1_DMA_RX_BUF)/2;

						dma.DMA_Memory0BaseAddr   =   (uint32_t)&_USART1_DMA_RX_BUF;    //DMA���ջ���ַ
						dma.DMA_BufferSize        =   sizeof(_USART1_DMA_RX_BUF);       //������������

				
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

					DMA_DoubleBufferModeConfig(DMA2_Stream2,  (uint32_t)&_USART1_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
					DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);

		DMA_Cmd(DMA2_Stream2, ENABLE);
}
//���ڽ����жϷ�����
void USART1_IRQHandler(void)
{
	static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
	{
		(void)USART1->SR;
		(void)USART1->DR;
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);	
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA2_Stream2, ENABLE);
     if(this_time_rx_len == RC_FRAME_LENGTH)
			{
//				RemoteDataPrcess(_USART1_DMA_RX_BUF[0]);
//        LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_RC));
			}
		}
		
		else 
		{
			DMA_Cmd(DMA2_Stream2, DISABLE);
			DMA_ClearFlag(DMA2_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
			this_time_rx_len = BSP_USART1_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream2);
			DMA2_Stream2->NDTR = (uint16_t)BSP_USART1_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream2->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA2_Stream2, ENABLE);
       if(this_time_rx_len == RC_FRAME_LENGTH)
			{
//				RemoteDataPrcess(_USART1_DMA_RX_BUF[1]);
//                LostCounterFeed(GetLostCounter(LOST_COUNTER_INDEX_RC));
			}
		}
	}       
}

static void USART1_FIFO_Init(void)
{
    FIFO_Init(&_USART1_RX_FIFO, (void *)_USART1_RX_BUF, RC_FRAME_LENGTH, BSP_USART1_RX_BUF_SIZE_IN_FRAMES); 
}

void *USART1_GetRxBuf(void)
{
    return (void *)&_USART1_RX_FIFO;
}



