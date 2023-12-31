#include "usart.h"

/*-----USART1_RX-----PB7----*/ 

/*
*********************************************************************************************************
*                                               VARIABLES
*********************************************************************************************************
*/
static uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];
static uint8_t _USART1_RX_BUF[BSP_USART1_DMA_RX_BUF_LEN];

#define BSP_USART2_DMA_RX_BUF_LEN 100
static uint8_t _USART2_DMA_RX_BUF[2][BSP_USART2_DMA_RX_BUF_LEN];
static uint8_t _USART2_RX_BUF[BSP_USART2_DMA_RX_BUF_LEN];

#define CH100_RX_BUFF_SIZE 100
uint8_t ch100_Rx_Buffer[CH100_RX_BUFF_SIZE];
__align(4) id0x91_t dat; /* struct must be 4 byte aligned */

uint8_t UART4_DMA_RX_BUF[UART4_RX_BUF_LENGTH];
uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];

static uint8_t USART5_DMA_TX_BUF[100];
static uint8_t judge_rxdata_buf[100];
static uint8_t _UART5_RX_BUF[100];

uint8_t judge_dma_rxbuff[2][100];
uint8_t  tx_buf[66];
uint8_t  pdata[32];
uint8_t  ddata[66];
uint8_t client_graphic_busy = 0;
u32 bullet_num=0;

#if EN_UART5_DMA_SECOND_FIFO == 1	
uint8_t _UART5_DMA_RX_BUF[2][BSP_UART5_DMA_RX_BUF_LEN];
#else
uint8_t _UART5_DMA_RX_BUF[100];
#endif

#define BSP_USART6_DMA_RX_BUF_LEN 100
static uint8_t _USART6_DMA_RX_BUF[2][BSP_USART6_DMA_RX_BUF_LEN];
static uint8_t _USART6_RX_BUF[BSP_USART6_DMA_RX_BUF_LEN];

#if EN_USART1
void usart1_init(uint32_t baud_rate)
{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;
		NVIC_InitTypeDef nvic;
		DMA_InitTypeDef dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); 
    GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_7;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOB, &gpio);
    
    USART_DeInit(USART1);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_Even;
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART1, &usart);
//    USART1_FIFO_Init();
    
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    
    DMA_DeInit(DMA2_Stream2);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART1->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART1_DMA_RX_BUF[0][0];
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(_USART1_DMA_RX_BUF)/2;
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
    
    DMA_DoubleBufferModeConfig(DMA2_Stream2, (uint32_t)&_USART1_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA2_Stream2, ENABLE);
    DMA_Cmd(DMA2_Stream2, ENABLE);
    
	nvic.NVIC_IRQChannel = USART1_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 0;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	

	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	USART_Cmd(USART1, ENABLE);

}

//串口接收中断服务函数
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
					USART1_Data_Receive_Process
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
					USART1_Data_Receive_Process
			}
		}
	}       
}
#endif




#if EN_USART2
void usart2_init(uint32_t baud_rate)
{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;
		NVIC_InitTypeDef nvic;
		DMA_InitTypeDef dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART1);
	
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_2;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOA, &gpio);
    
    USART_DeInit(USART2);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_Even;
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART2, &usart);
//    USART1_FIFO_Init();
    
    USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
    
    DMA_DeInit(DMA1_Stream5);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART2->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART2_DMA_RX_BUF[0][0];
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(_USART2_DMA_RX_BUF)/2;
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
    DMA_Init(DMA1_Stream5, &dma);
    
    DMA_DoubleBufferModeConfig(DMA1_Stream5, (uint32_t)&_USART2_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
    DMA_Cmd(DMA1_Stream5, ENABLE);
    
	nvic.NVIC_IRQChannel = USART2_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 0;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	

	USART_ITConfig(USART2, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	USART_Cmd(USART2, ENABLE);

}

//串口接收中断服务函数
void USART2_IRQHandler(void)
{
	static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART2, USART_IT_IDLE) != RESET)
	{
		(void)USART2->SR;
		(void)USART2->DR;
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream2) == 0)
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);
			DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);	
			this_time_rx_len = BSP_USART2_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)BSP_USART2_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA1_Stream5, ENABLE);
     if(this_time_rx_len == RC_FRAME_LENGTH)
			{
//				RemoteDataPrcess(_USART1_DMA_RX_BUF[0]);
					USART2_Data_Receive_Process
			}
		}
		
		else 
		{
			DMA_Cmd(DMA1_Stream5, DISABLE);
			DMA_ClearFlag(DMA1_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
			this_time_rx_len = BSP_USART2_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream5);
			DMA1_Stream5->NDTR = (uint16_t)BSP_USART2_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
			DMA1_Stream5->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA1_Stream5, ENABLE);
       if(this_time_rx_len == RC_FRAME_LENGTH)
			{
//				RemoteDataPrcess(_USART1_DMA_RX_BUF[1]);
					USART2_Data_Receive_Process
			}
		}
	}       
}

#endif



#if EN_USART3
void usart3_init(u32 bound)
{
  USART_InitTypeDef USART_InitStructure;
  GPIO_InitTypeDef GPIO_InitStructure;
	DMA_InitTypeDef  DMA_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

  /* Peripheral Clock Enable -------------------------------------------------*/
  /* Enable GPIO clock */
  RCC_AHB1PeriphClockCmd(USART_CH100_TX_GPIO_CLK | USART_CH100_RX_GPIO_CLK, ENABLE);
  
  /* Enable USART clock */
  USART_CH100_CLK_INIT(USART_CH100_CLK, ENABLE);
  
  /* Enable the DMA clock */
  RCC_AHB1PeriphClockCmd(USART_CH100_DMAx_CLK, ENABLE);
  
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
 
 
		NVIC_InitStructure.NVIC_IRQChannel = USART_CH100_IRQn;

    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2;//


    NVIC_InitStructure.NVIC_IRQChannelSubPriority =1;//

    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;//IRQ


	NVIC_Init(&NVIC_InitStructure);

  /* Enable the USART OverSampling by 8 */
  USART_InitStructure.USART_BaudRate = bound;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  /* When using Parity the word length must be configured to 9 bits */
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  USART_Init(USART_CH100, &USART_InitStructure);

	/*使能空闲帧中断*/
   USART_ITConfig(USART_CH100,USART_IT_IDLE,ENABLE);
	 USART_ClearFlag(USART_CH100,USART_FLAG_TC|USART_FLAG_IDLE);
  /* Configure DMA controller to manage USART TX and RX DMA request ----------*/ 
   
  /* Configure DMA Initialization Structure */
  DMA_InitStructure.DMA_BufferSize = CH100_RX_BUFF_SIZE ;
  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable ;
  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull ;
  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single ;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&(USART_CH100->DR)) ;
  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_Priority = DMA_Priority_High;
  /* Configure RX DMA */
  DMA_InitStructure.DMA_Channel = USART_CH100_RX_DMA_CHANNEL ;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory ;
  DMA_InitStructure.DMA_Memory0BaseAddr =(uint32_t)ch100_Rx_Buffer ; 
  DMA_Init(USART_CH100_RX_DMA_STREAM,&DMA_InitStructure);
   /* Enable DMA USART RX Stream */
  DMA_Cmd(USART_CH100_RX_DMA_STREAM,ENABLE);
  /* Enable USART DMA RX Requsts */
  USART_DMACmd(USART_CH100, USART_DMAReq_Rx, ENABLE);
  /* Enable USART */
  USART_Cmd(USART_CH100, ENABLE);
}


void USART_CH100_IRQHandler(void)
{
	if(USART_GetITStatus(USART_CH100, USART_IT_IDLE)!= RESET)//
	{
		USART_ReceiveData(USART_CH100); //一定要读一次，不然可能会丢第一个字节，原因未知
		USART_ClearITPendingBit(USART_CH100,USART_IT_IDLE);//清除中断标志位
		DMA_Cmd(USART_CH100_RX_DMA_STREAM,DISABLE);  
		USART_DMACmd(USART_CH100, USART_DMAReq_Rx, DISABLE);
		memcpy(&dat, &ch100_Rx_Buffer[6], sizeof(id0x91_t));
//		CH100_getDATA();
		USART3_Data_Receive_Process
		USART_DMACmd(USART_CH100, USART_DMAReq_Rx, ENABLE);
		DMA_Cmd(USART_CH100_RX_DMA_STREAM,ENABLE);//重新置位后，地址指针变成0
	}
}
#endif







#if EN_UART5
void uart5_init(u32 bound)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART5,ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  //IO初始化
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
  //串口5初始化
  USART_InitStructure.USART_BaudRate              =   bound;
  USART_InitStructure.USART_HardwareFlowControl   =   USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode                  =   USART_Mode_Tx|USART_Mode_Rx;
  USART_InitStructure.USART_Parity                =   USART_Parity_No;
  USART_InitStructure.USART_StopBits              =   USART_StopBits_1;
  USART_InitStructure.USART_WordLength            =   USART_WordLength_8b;
  USART_Init(UART5, &USART_InitStructure);
  USART_Cmd(UART5, ENABLE);
  //串口5配置接收DMA
  USART_DMACmd(UART5, USART_DMAReq_Rx, ENABLE);    //启用USART的DMA接口，DMA1、数据流0、通道4
  DMA_StructInit(&DMA_InitStructure);              //DMA各个参数赋初值

#if EN_UART5_DMA_SECOND_FIFO == 1
  DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_UART5_DMA_RX_BUF[0][0];
  DMA_InitStructure.DMA_BufferSize        =   sizeof(_UART5_DMA_RX_BUF)/2;
#else
  DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)&_UART5_DMA_RX_BUF;    //DMA接收基地址
  DMA_InitStructure.DMA_BufferSize        =   sizeof(_UART5_DMA_RX_BUF);       //传输数据数量
#endif
  DMA_InitStructure.DMA_Channel           =   DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&UART5->DR);       //外设基地址
  DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;   //外设到存储器
  DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;    //外设地址不递增
  DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;         //存储器地址递增
  DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_Byte;      //存储器数据宽度
  DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_Byte;  //外设数据宽度
  DMA_InitStructure.DMA_Mode              =   DMA_Mode_Normal;              //是否开循环模式
  DMA_InitStructure.DMA_Priority          =   DMA_Priority_Medium  ;        //优先级中等
  DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_1QuarterFull;
  DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;       //存储器突发，单次传输
  DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;   //外设突发，单次传输
  DMA_Init(DMA1_Stream0, &DMA_InitStructure);
  DMA_Cmd(DMA1_Stream0, ENABLE);                        //使能DMA通道开启


  //配置Memory1,Memory0是第一个使用的Memory
#if EN_UART5_DMA_SECOND_FIFO == 1
  DMA_DoubleBufferModeConfig(DMA1_Stream0,  (uint32_t)&_UART5_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
  DMA_DoubleBufferModeCmd(DMA1_Stream0, ENABLE);
#endif
  NVIC_InitStructure.NVIC_IRQChannel						=	UART5_IRQn;          //串口5接收中断
  NVIC_InitStructure.NVIC_IRQChannelCmd					=	ENABLE;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority	=	1;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority			=	2;
  NVIC_Init(&NVIC_InitStructure);
  USART_ITConfig(UART5, USART_IT_IDLE, ENABLE);
  USART_Cmd(UART5, ENABLE);


  //串口5配置发送DMA
  USART_DMACmd(UART5, USART_DMAReq_Tx, ENABLE);   //启用USART的DMA接口，DMA1、数据流7、通道4
  DMA_Cmd(DMA1_Stream7, DISABLE);                 // 关DMA通道
  DMA_DeInit(DMA1_Stream7);

  while(DMA_GetCmdStatus(DMA1_Stream7) != DISABLE) {}
  DMA_InitStructure.DMA_Channel = DMA_Channel_4;
  DMA_InitStructure.DMA_PeripheralBaseAddr	= (uint32_t)(&UART5->DR);
  DMA_InitStructure.DMA_Memory0BaseAddr   	= (uint32_t)&tx_buf[0];
  DMA_InitStructure.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
	DMA_InitStructure.DMA_BufferSize			= sizeof(tx_buf);   //发送数据放在该数组中
  DMA_InitStructure.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
  DMA_InitStructure.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
  DMA_InitStructure.DMA_Mode 				= DMA_Mode_Normal;
  DMA_InitStructure.DMA_Priority 			= DMA_Priority_Low ;
  DMA_InitStructure.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
  DMA_InitStructure.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
  DMA_InitStructure.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
  DMA_InitStructure.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream7,&DMA_InitStructure);
  DMA_Cmd(DMA1_Stream7, ENABLE);                        //使能DMA通道开启

  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Stream7_IRQn;     // DMA发送中断
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;   // 优先级设置
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
}
void MYDMA_Enable(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)
{
 
	DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输 
	
	while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}	//确保DMA可以被设置  
		
	DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量  
 
	DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输 
}	  


/**
  * @brief  裁判系统空闲中断(UART4)
  * @param  void
  * @retval void
  * @note   DJI很毒,以50HZ发送Info包，当有实时数据包产生时，在最近的一帧Info前无间隔加上实时数据包
  */

void UART5_IRQHandler(void)
{
  static uint32_t this_time_rx_len = 0;
  if(USART_GetITStatus(UART5, USART_IT_IDLE) != RESET)
    {
      (void)UART5->SR;
      (void)UART5->DR;

#if EN_UART5_DMA_SECOND_FIFO == 1
      if(DMA_GetCurrentMemoryTarget(DMA1_Stream0) == 0)
        {
          DMA_Cmd(DMA1_Stream0, DISABLE);
          DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
          this_time_rx_len = BSP_UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

          DMA_SetCurrDataCounter(DMA1_Stream0, BSP_UART5_DMA_RX_BUF_LEN);
          DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[1][0],DMA_Memory_1);
          DMA_Cmd(DMA1_Stream0, ENABLE);

//          if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
//            judgement_data_handle(_UART5_DMA_RX_BUF[0],this_time_rx_len);
					UART5_Data_Receive_Process
        }
      //Target is Memory1
      else
        {
          DMA_Cmd(DMA1_Stream0, DISABLE);
          DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);

          this_time_rx_len =BSP_UART5_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA1_Stream0);

          DMA_SetCurrDataCounter(DMA1_Stream0, BSP_UART5_DMA_RX_BUF_LEN);
          DMA_MemoryTargetConfig (DMA1_Stream0,(uint32_t)&_UART5_DMA_RX_BUF[0][0],DMA_Memory_0);
//				DMA1_Stream0->NDTR = (uint16_t)BSP_UART5_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
//				DMA1_Stream0->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
          DMA_Cmd(DMA1_Stream0, ENABLE);
					UART5_Data_Receive_Process
//				FIFO_S_Puts(&_UART5_RX_FIFO,(void *)&_UART5_DMA_RX_BUF[1][0],this_time_rx_len);
//				unpack_fifo_data(&judge_unpack_obj,DN_REG_ID);
//          if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
//            judgement_data_handle(_UART5_DMA_RX_BUF[1],this_time_rx_len);
        }
#else
      DMA_Cmd(DMA1_Stream0, DISABLE);                          //关闭串口5的DMA接收通道
      DMA_ClearFlag(DMA1_Stream0, DMA_FLAG_TCIF0 | DMA_FLAG_HTIF0);
      this_time_rx_len = 100 - DMA_GetCurrDataCounter(DMA1_Stream0); //获取DMA_GetCurrDataCounter剩余数据量

      DMA_SetCurrDataCounter(DMA1_Stream0, 100);      //设置当前DMA剩余数据量
      DMA_Cmd(DMA1_Stream0, ENABLE);                                       //开启串口5的DMA接收通道
			
				UART5_Data_Receive_Process
//      if(this_time_rx_len > (HEADER_LEN + CMD_LEN + CRC_LEN))
//      judgement_data_handle(_UART5_DMA_RX_BUF,this_time_rx_len);  //_UART5_DMA_RX_BUF作为DMA接收缓存区
#endif

    }
	}

#endif
	





#if EN_UART4
	/*-----UART4_TX-----PC10-----*/
/*-----UART4_RX-----PC11-----*/
	void uart4_init(u32 bound)
{
  USART_InitTypeDef uart4;
  GPIO_InitTypeDef  gpio;
  NVIC_InitTypeDef  nvic;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4);
  GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);

  gpio.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11;
  gpio.GPIO_Mode = GPIO_Mode_AF;
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_100MHz;
  gpio.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOC,&gpio);

  uart4.USART_BaudRate = bound;          // speed 10byte/ms
  uart4.USART_WordLength = USART_WordLength_8b;
  uart4.USART_StopBits = USART_StopBits_1;
  uart4.USART_Parity = USART_Parity_No;
  uart4.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
  uart4.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(UART4,&uart4);

  USART_DMACmd(UART4, USART_DMAReq_Rx, ENABLE);


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

  nvic.NVIC_IRQChannel = UART4_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 3;
  nvic.NVIC_IRQChannelSubPriority =3;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);

  USART_DMACmd(UART4, USART_DMAReq_Tx, ENABLE);

  DMA_Cmd(DMA1_Stream4, DISABLE);                           // 关DMA通道
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

//	DMA_Cmd(DMA1_Stream3, DISABLE);                           // 关DMA通道
  nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;   // 发送DMA通道的中断配置
  nvic.NVIC_IRQChannelPreemptionPriority = 3;     // 优先级设置
  nvic.NVIC_IRQChannelSubPriority = 3;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);


  USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);
  USART_Cmd(UART4,ENABLE);

}

void UART4_IRQHandler(void)
{
  uint8_t length=0;
  if(USART_GetITStatus(UART4, USART_IT_IDLE) != RESET)    //接收中断
    {
      (void)UART4->SR;
      (void)UART4->DR;
      DMA_Cmd(DMA1_Stream2, DISABLE);
      DMA_ClearFlag(DMA1_Stream2, DMA_FLAG_TCIF2 | DMA_FLAG_HTIF2);
      length = UART4_RX_BUF_LENGTH - DMA_GetCurrDataCounter(DMA1_Stream2);

//      targetOffsetDataDeal(length, UART4_DMA_RX_BUF );
			UART4_Data_Receive_Process
      DMA_SetCurrDataCounter(DMA1_Stream2,UART4_RX_BUF_LENGTH);
      DMA_Cmd(DMA1_Stream2, ENABLE);

    }
}


void Uart4DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
//    DMA_Cmd(DMA_Streamx, DISABLE);                      //关闭DMA传输
//    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //确保DMA可以被设置
  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //数据传输量
  DMA_Cmd(DMA_Streamx, ENABLE);                      //开启DMA传输
}



void DMA1_Stream4_IRQHandler(void)
{
  //清除标志
  if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Steam3传输完成
    {
      DMA_Cmd(DMA1_Stream4, DISABLE);                      //关闭DMA传输
      DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//清除DMA1_Steam3传输完成标志
    }
}


//发送单字节
void Uart4SendByteInfoProc(u8 nSendInfo)
{
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = UART4_DMA_TX_BUF;
  *pBuf++ = nSendInfo;

  Uart4DmaSendDataProc(DMA1_Stream4,1); //开始一次DMA传输！

}

//发送多字节

void Uart4SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;
  //指向发送缓冲区
  pBuf = UART4_DMA_TX_BUF;
  for (i=0; i<nSendCount; i++)
    {
      *(pBuf+i) = pSendInfo[i];
    }

  //DMA发送方式

  Uart4DmaSendDataProc(DMA1_Stream4,nSendCount); //开始一次DMA传输！
}

#endif

			


#if EN_USART6
void usart6_init(uint32_t baud_rate)
{
		GPIO_InitTypeDef gpio;
		USART_InitTypeDef usart;
		NVIC_InitTypeDef nvic;
		DMA_InitTypeDef dma;
    
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC, ENABLE); 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE); 
    GPIO_PinAFConfig(GPIOA, GPIO_PinSource2, GPIO_AF_USART1);
	
    GPIO_StructInit(&gpio);
    gpio.GPIO_Pin = GPIO_Pin_6;
    gpio.GPIO_Mode = GPIO_Mode_AF;
    gpio.GPIO_Speed = GPIO_Speed_2MHz;
    gpio.GPIO_PuPd = GPIO_PuPd_UP;
    GPIO_Init(GPIOC, &gpio);
    
    USART_DeInit(USART6);
    USART_StructInit(&usart);
    usart.USART_BaudRate = baud_rate;
    usart.USART_WordLength = USART_WordLength_8b;
    usart.USART_StopBits = USART_StopBits_1;
    usart.USART_Parity = USART_Parity_Even;
    usart.USART_Mode = USART_Mode_Rx;
    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
    USART_Init(USART6, &usart);
//    USART1_FIFO_Init();
    
    USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
    
    DMA_DeInit(DMA2_Stream1);
    DMA_StructInit(&dma);
    dma.DMA_Channel = DMA_Channel_4;
    dma.DMA_PeripheralBaseAddr = (uint32_t)(&USART6->DR);
    dma.DMA_Memory0BaseAddr = (uint32_t)&_USART6_DMA_RX_BUF[0][0];
    dma.DMA_DIR = DMA_DIR_PeripheralToMemory;
    dma.DMA_BufferSize = sizeof(_USART6_DMA_RX_BUF)/2;
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
    DMA_Init(DMA2_Stream1, &dma);
    
    DMA_DoubleBufferModeConfig(DMA2_Stream1, (uint32_t)&_USART6_DMA_RX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
    DMA_DoubleBufferModeCmd(DMA1_Stream5, ENABLE);
    DMA_Cmd(DMA2_Stream1, ENABLE);
    
	nvic.NVIC_IRQChannel = USART6_IRQn;                          
	nvic.NVIC_IRQChannelPreemptionPriority = 0;   //pre-emption priority 
	nvic.NVIC_IRQChannelSubPriority = 0;		    //subpriority 
	nvic.NVIC_IRQChannelCmd = ENABLE;			
	NVIC_Init(&nvic);	

	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
	USART_Cmd(USART6, ENABLE);

}

//串口接收中断服务函数
void USART6_IRQHandler(void)
{
	static uint32_t this_time_rx_len = 0;
	if(USART_GetITStatus(USART6, USART_IT_IDLE) != RESET)
	{
		(void)USART6->SR;
		(void)USART6->DR;
		if(DMA_GetCurrentMemoryTarget(DMA2_Stream1) == 0)
		{
			DMA_Cmd(DMA2_Stream1, DISABLE);
			DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);	
			this_time_rx_len = BSP_USART2_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
			DMA2_Stream1->NDTR = (uint16_t)BSP_USART6_DMA_RX_BUF_LEN;     //relocate the dma memory pointer to the beginning position
			DMA2_Stream1->CR |= (uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 1
			DMA_Cmd(DMA2_Stream1, ENABLE);
     if(this_time_rx_len == RC_FRAME_LENGTH)
			{
//				RemoteDataPrcess(_USART1_DMA_RX_BUF[0]);
					USART6_Data_Receive_Process
			}
		}
		
		else 
		{
			DMA_Cmd(DMA2_Stream1, DISABLE);
			DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1 | DMA_FLAG_HTIF1);
			this_time_rx_len = BSP_USART2_DMA_RX_BUF_LEN - DMA_GetCurrDataCounter(DMA2_Stream1);
			DMA2_Stream1->NDTR = (uint16_t)BSP_USART6_DMA_RX_BUF_LEN;      //relocate the dma memory pointer to the beginning position
			DMA2_Stream1->CR &= ~(uint32_t)(DMA_SxCR_CT);                  //enable the current selected memory is Memory 0
			DMA_Cmd(DMA2_Stream1, ENABLE);
       if(this_time_rx_len == RC_FRAME_LENGTH)
			{
//				RemoteDataPrcess(_USART1_DMA_RX_BUF[1]);
					USART6_Data_Receive_Process
			}
		}
	}       
}

#endif