#include "sys.h"
#include "usart.h"	
#include "led.h"
#include "delay.h"
#include "dma.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//如果使用ucos,则包括下面的头文件即可.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos 使用	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//STM32F103ZE核心板
//串口1初始化		
////////////////////////////////////////////////////////////////////////////////// 	  


//////////////////////////////////////////////////////////////////
//加入以下代码,支持printf函数,而不需要选择use MicroLIB	  
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

/*使用microLib的方法*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //如果使能了接收
//串口1中断服务程序
//注意,读取USARTx->SR能避免莫名其妙的错误   	

u8 DMA_RX_BUF[DMA_REC_LEN]; 			  //DMA的接收缓冲，最大DMA_REC_LEN个字节.

u8 USART_RX_BUF[USART_REC_LEN];     //接收缓冲,最大USART_REC_LEN个字节.
//接收状态
//bit15，	接收完成标志
//bit14，	接收到0x0d
//bit13~0，	接收到的有效字节数目
u16 USART_RX_STA=0;       //接收状态标记	  
 

//标志LED状态的核心变量
//bit7~4, 四个LED的状态
//bit3~0, LED的闪烁次数
u8 USART_LED_STA=0;   		//LED状态标记



u8 temp_rx_buf;
uint8_t i,m,j,times;
int led_mode;
uint8_t flash_times=0;
uint8_t LED_count=0;
uint8_t frame_is_ok=1;
uint16_t length=0;

/****LED控制函数参数初始化****/
void LED_controller_init()
{
	 i=0;
	 j=0;
	 m=0;
	 temp_rx_buf=0;
	 times=0;
	 led_mode=1;
	 flash_times=0;
	 LED_count=0;
	 frame_is_ok=1;
	 length=0;
}

/****LED控制开关****/
void LED_controller()
{		
	if(led_mode==1)  //LED的非闪烁模式
	{	
		LED1_OFF;LED2_OFF;LED3_OFF;LED4_OFF;
		if(((USART_LED_STA&0xf0)>>4)&0x01)
		{LED1_ON;}
		else
		{LED1_OFF;}
		if((((USART_LED_STA&0xf0)>>4)&0x02)>>1)
		{LED2_ON;}
		else
		{LED2_OFF;}
		if((((USART_LED_STA&0xf0)>>4)&0x04)>>2)
		{LED3_ON;}
		else
		{LED3_OFF;}
		if((((USART_LED_STA&0xf0)>>4)&0x08)>>3)
		{LED4_ON;}
		else
		{LED4_OFF;}
	}
	else if(led_mode==0) //LED闪烁模式
	{
		
	if(((USART_LED_STA&0xf0)>>4)==1)
		{
			LED1_FLASH(flash_times);
			USART_LED_STA=0;
		}
		else
		{LED1_OFF;}
		if(((USART_LED_STA&0xf0)>>5)==1)
		{
			LED2_FLASH(flash_times);
			USART_LED_STA=0;
		}
		else
		{LED2_OFF;}
		if(((USART_LED_STA&0xf0)>>6)==1)
		{
			LED3_FLASH(flash_times);
			USART_LED_STA=0;
		}
		else
		{LED3_OFF;}
		if(((USART_LED_STA&0xf0)>>7)==1)
		{
			LED4_FLASH(flash_times);
			USART_LED_STA=0;
		}
		else
		{LED4_OFF;}
	}
		
}


/****LED的状态观测，根据指令改变状态****/
void LED_state_observer()
{
	
	if(USART_RX_STA&0x8000)
		{
			frame_is_ok =1;
			length=USART_RX_STA&0x3fff;
			
/***************************			
			LEDn Flash n功能判断		
*****************************/			
				
			if(length>0\
					&&USART_RX_BUF[0]=='L'\
					&&USART_RX_BUF[1]=='E'\
					&&USART_RX_BUF[2]=='D'\
					&&USART_RX_BUF[4]==0x20\
					&&USART_RX_BUF[5]=='F'\
					&&USART_RX_BUF[6]=='l'\
					&&USART_RX_BUF[7]=='a'\
					&&USART_RX_BUF[8]=='s'\
					&&USART_RX_BUF[9]=='h'\
					&&USART_RX_BUF[10]==0x20
				 )
				{
					led_mode=0;
					switch (USART_RX_BUF[3])
					{
						case '1':
							USART_LED_STA&=0x00;
							USART_LED_STA|=0x10;
							USART_LED_STA|=(USART_RX_BUF[11]&=0x0f);
							flash_times   =(USART_RX_BUF[11]&=0x0f);
						break;
						case '2':
							USART_LED_STA&=0x00;
							USART_LED_STA|=0x20;
							USART_LED_STA|=(USART_RX_BUF[11]&=0x0f);
							flash_times   =(USART_RX_BUF[11]&=0x0f);
						break;
						case '3':
							USART_LED_STA&=0x00;
							USART_LED_STA|=0x40;
							USART_LED_STA|=(USART_RX_BUF[11]&=0x0f);
							flash_times   =(USART_RX_BUF[11]&=0x0f);
						break;
						case '4':
							USART_LED_STA&=0x00;
							USART_LED_STA|=0x80;
							USART_LED_STA|=(USART_RX_BUF[11]&=0x0f);
							flash_times   =(USART_RX_BUF[11]&=0x0f);
						break;
						default:
							frame_is_ok =0;
							break;

					}							
					USART_RX_STA=0;				
				}
/***************************			
		LEDn Turn Off功能判断		
*****************************/	
				
				else if(length>0\
					&&USART_RX_BUF[0]=='L'\
					&&USART_RX_BUF[1]=='E'\
					&&USART_RX_BUF[2]=='D'\
					&&USART_RX_BUF[4]==0x20\
					&&USART_RX_BUF[5]=='T'\
					&&USART_RX_BUF[6]=='u'\
					&&USART_RX_BUF[7]=='r'\
					&&USART_RX_BUF[8]=='n'\
					&&USART_RX_BUF[9]==0x20\
					&&USART_RX_BUF[10]=='O'\
					&&USART_RX_BUF[11]=='f'\
					&&USART_RX_BUF[12]=='f'\
				 )
				{
					led_mode=1;
					switch (USART_RX_BUF[3])
					{
						case '1':
							USART_LED_STA&=~(1<<4);
						break;
						case '2':
							USART_LED_STA&=~(1<<5);
						break;
						case '3':
							USART_LED_STA&=~(1<<6);
						break;
						case '4':
							USART_LED_STA&=~(1<<7);
						break;
						default:
							frame_is_ok =0;
							break;
					}
						USART_RX_STA=0;
				}
				
/***************************			
	  LEDn Turn On功能判断				
*****************************/				
				else if(length>0\
					&&USART_RX_BUF[0]=='L'\
					&&USART_RX_BUF[1]=='E'\
					&&USART_RX_BUF[2]=='D'\
					&&USART_RX_BUF[4]==0x20\
					&&USART_RX_BUF[5]=='T'\
					&&USART_RX_BUF[6]=='u'\
					&&USART_RX_BUF[7]=='r'\
					&&USART_RX_BUF[8]=='n'\
					&&USART_RX_BUF[9]==0x20\
					&&USART_RX_BUF[10]=='O'\
					&&USART_RX_BUF[11]=='n'\
				  )
				{
					led_mode=1;
					switch (USART_RX_BUF[3])
					{
						case '1':
							USART_LED_STA|=(1<<4);
						break;
						case '2':
							USART_LED_STA|=(1<<5);
						break;
						case '3':
							USART_LED_STA|=(1<<6);
						break;
						case '4':
							USART_LED_STA|=(1<<7);
						break;
						default:
							frame_is_ok =0;
							break;
					}
        		USART_RX_STA=0;
				}
				
				
/***************************			
	  LEDstring功能判断			
*****************************/				
				else if(length>0\
					&&USART_RX_BUF[0]=='L'\
					&&USART_RX_BUF[1]=='E'\
					&&USART_RX_BUF[2]=='D'\
					&&USART_RX_BUF[4]!=0x20
				 )
				{
					led_mode=1;
					for(LED_count=3;LED_count<length;LED_count++)
					{								
							USART_LED_STA=0;
							temp_rx_buf=USART_RX_BUF[LED_count]-0x30;
							if(temp_rx_buf>9&&temp_rx_buf<=96)temp_rx_buf=(USART_RX_BUF[LED_count]-0x07);
							else if(temp_rx_buf>96)temp_rx_buf-=39;
							USART_LED_STA=(temp_rx_buf<<4);
							delay_ms(200);
							LED_controller();
							delay_ms(200);	
		
					
					}
					USART_RX_STA=0;
				}		
				else
				{				
				frame_is_ok =0;
				printf("命令有误\r\n");
				}
				
			}
		
			else
		{
			times++;
			if(times%500==0)
			{
				printf("\r\n");
			}
			if(times%2000==0)
			{	
				printf("命令LEDn Flash n或LEDn Turn Off或LEDn Turn On或LEDstring,以回车键结束\r\n");  
				delay_ms(100);  
			}
		}
		USART_RX_STA=0;
} 



void uart_init(u32 bound){
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//使能USART1，GPIOA时钟
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.9
   
  //USART1_RX	  GPIOA.10初始化
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIOA.10  

  //Usart1 NVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//抢占优先级3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//子优先级3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQ通道使能
	NVIC_Init(&NVIC_InitStructure);	//根据指定的参数初始化VIC寄存器
  
   //USART 初始化设置

	USART_InitStructure.USART_BaudRate = bound;//串口波特率
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//字长为8位数据格式
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//一个停止位
	USART_InitStructure.USART_Parity = USART_Parity_No;//无奇偶校验位
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//无硬件数据流控制
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//收发模式

  USART_Init(USART1, &USART_InitStructure); //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//开启串口接受中断
//	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//开启串口空闲中断
  USART_Cmd(USART1, ENABLE);                    //使能串口1 

}

void USART1_IRQHandler(void)                	//串口1中断服务程序
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//如果SYSTEM_SUPPORT_OS为真，则需要支持OS.
	OSIntEnter();    
#endif
		
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //接收中断(接收到的数据必须是0x0d 0x0a结尾)
		{
			
		MYDMA_Enable(DMA2_Channel4);
		Res =USART_ReceiveData(USART1);	//读取接收到的数据
		
		if((USART_RX_STA&0x8000)==0)//接收未完成
			{
			if(USART_RX_STA&0x4000)//接收到了0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//接收错误,重新开始
				else 
				{
					USART_RX_STA|=0x8000;	//接收完成了
				}
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

