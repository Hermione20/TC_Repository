/* Includes ------------------------------------------------------------------*/
#include "sys.h"
#include "usart.h"	
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif	  
/**
  ******************************************************************************
  * @file    usart.c
  * @author  TC
  * @version V1.0.0
  * @date    07-September-2023
  * @brief   该文件提供固件功能来管理通用同步异步接收发送器(USART)的以下功能:
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
      (#)使用以下功能开启外设时钟
					RCC_APB2PeriphClockCmd(RCC_APB2Periph_USARTx, ENABLE)用于USART1和USART6
					RCC_APB1PeriphClockCmd(RCC_APB1Periph_USARTx, ENABLE)用于USART2, USART3，UART4或UART5。
  
      (#) 根据USART模式，使用RCC_AHB1PeriphClockCmd()函数。I/O可以是TX, RX, CTS，或/和SCLK)。
  
      (#) Peripheral's alternate function: 
        (++) 使用GPIO_PinAFConfig()函数将引脚连接到所需外设的备用功能(AF)
        (++) 通过以下方式配置备用功能中的所需引脚:
            GPIO_InitStruct->GPIO_Mode = GPIO_Mode_AF
        (++) 选择类型，上拉/下拉和输出速度通过
            GPIO_PuPd, GPIO_OType and GPIO_Speed members
        (++) Call GPIO_Init() function
          
      (#) 使用USART_Init()函数编程波特率，字长，停止位，奇偶校验，硬件流控制和模式(接收器/发射器)。
  
      (#) 对于同步模式，启用时钟并使用USART_ClockInit()函数对极性、相位和最后一位进行编程。
  
      (#) 如果需要使用中断模式，请使用USART_ITConfig()函数启用NVIC和相应的中断。
  
      (#) When using the DMA mode 
        (++) 使用DMA Init()函数配置DMA
        (++) 使用USART_DMACmd()函数激活所需的通道请求
   
      (#) 使用USART_Cmd()函数启用USART。
   
      (#) 当使用DMA模式时，使用DMA_Cmd()函数启用DMA。
    
      -@- 请参阅多处理器，LIN，半双工，智能卡，IrDA子部分
					欲知详情
    
    [..]        
				为了达到更高的通信波特率，可以使用USART_OverSampling8Cmd()函数启用8模式的过采样。
				这个函数应该在启用USART时钟(RCC_APBxPeriphClockCmd())之后，在调用USART_Init()函数之前调用。


            
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
* @brief    : 加入以下代码,支持printf函数,而不需要选择use MicroLIB
* @param    : ch
* @param    : FILE *f
* @retval   : void
* @Note     : 加入以下代码,支持printf函数,而不需要选择use MicroLIB
************************************************************************************************************************
**/ 
#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 
}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{ 	
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
	USART1->DR = (u8) ch;      
	return ch;
}
#endif
 
#if EN_USART1   //如果使能了

/* Variables_definination-----------------------------------------------------------------------------------------------*/

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记

/*----------------------------------------------------------------------------------------------------------------------*/


/**
************************************************************************************************************************
* @Name     : uart_init
* @brief    : This function process the can message representing the encoder data received from the CAN2 bus.
* @param    : bound     the baud rate setting,bound:波特率
* @retval   : void
* @Note     : 初始化IO 串口1 
************************************************************************************************************************
**/
void uart1_init(u32 bound){
   //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE); //使能GPIOA时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);//使能USART1时钟
 
	//串口1对应引脚复用映射
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource9,GPIO_AF_USART1); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource10,GPIO_AF_USART1); //GPIOA10复用为USART1
	
	//USART1端口配置
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_10; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOA,&GPIO_InitStructure); //初始化PA9，PA10

   //USART1 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
  USART_Init(USART1, &USART_InitStructure); //初始化串口1
	
  USART_Cmd(USART1, ENABLE);  //使能串口1 
	
	//USART_ClearFlag(USART1, USART_FLAG_TC);
	
#if EN_USART1_RX	
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启相关中断

	//Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;//串口1中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器

#endif
}
#if EN_USART1_RX
/**
************************************************************************************************************************
* @Name     : USART1_IRQHandler
* @brief    : 串口1中断服务程序
* @param    : none
* @retval   : void
* @Note     : 注意,读取USARTx->SR能避免莫名其妙的错误   	
************************************************************************************************************************
**/
void USART1_IRQHandler(void)                	//串口1中断服务程序
{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
	{
		Res =USART_ReceiveData(USART1);//(USART1->DR);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
		{
			if(USART_RX_STA&0x4000)//接收到了0x0d
			{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else USART_RX_STA|=0x8000;	//接收完成了 
			}
			else //还没收到0X0D
			{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
				{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//接收数据错误,重新开始接收	  
				}		 
			}
		}   		 
  } 
#if SYSTEM_SUPPORT_OS 	//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntExit();  											 
#endif
} 
#endif	
#endif

#if EN_UART4
void uart4_init(u32 bound){
	//GPIO端口设置
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE); //使能GPIOA时钟
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);//使能USART1时钟

	//串口4对应引脚复用映射
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource10,GPIO_AF_UART4); //GPIOA9复用为USART1
	GPIO_PinAFConfig(GPIOC,GPIO_PinSource11,GPIO_AF_UART4); //GPIOA10复用为USART1

	//UART4端口配置
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9与GPIOA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;	//速度50MHz
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP; //推挽复用输出
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP; //上拉
	GPIO_Init(GPIOC,&GPIO_InitStructure); //初始化PA9，PA10

	//UART4 初始化设置
	USART_InitStructure.USART_BaudRate = bound;//波特率设置
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式
	USART_Init(UART4, &USART_InitStructure); //初始化串口1
	
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
#endif
#if EN_UART4_DMA_TX_IQR		  
	nvic.NVIC_IRQChannel = DMA1_Stream4_IRQn;   // 发送DMA通道的中断配置
	nvic.NVIC_IRQChannelPreemptionPriority = 3;     // 优先级设置
	nvic.NVIC_IRQChannelSubPriority = 3;
	nvic.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&nvic);
    DMA_ITConfig(DMA1_Stream4,DMA_IT_TC,ENABLE);
#endif	
#if EN_UART4_RX	
	USART_ITConfig(UART4, USART_IT_RXNE, ENABLE);//开启发送非空中断
  //USART_ITConfig(UART4,USART_IT_IDLE,ENABLE);  //开启空闲帧中断

	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;//串口4中断通道
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority =3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
#endif

  USART_Cmd(UART4, ENABLE);  //使能串口1 
}



#if EN_UART4_RX
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

//	  targetOffsetDataDeal(length, UART4_DMA_RX_BUF );
	  DMA_SetCurrDataCounter(DMA1_Stream2,UART4_RX_BUF_LENGTH);
	  DMA_Cmd(DMA1_Stream2, ENABLE);
	}
}
#endif
#if EN_UART4_DMA_TX_IRQ
	void DMA1_Stream4_IRQHandler(void)
	{
	//清除标志
	if(DMA_GetFlagStatus(DMA1_Stream4,DMA_FLAG_TCIF4)!=RESET)//等待DMA1_Steam3传输完成
	{
	  DMA_Cmd(DMA1_Stream4, DISABLE);                      //关闭DMA传输
	  DMA_ClearFlag(DMA1_Stream4,DMA_FLAG_TCIF4);//清除DMA1_Steam3传输完成标志
	}
	}
#endif
#endif

	
	