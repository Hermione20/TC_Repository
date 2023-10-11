#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 

//STM32F103ZE核心板
//串口1初始化		   

//********************************************************************************
#define DMA_REC_LEN         50		//定义最大接收字节数 50
#define USART_REC_LEN  			50  	//定义最大接收字节数 50
#define EN_USART1_RX 			1		//使能（1）/禁止（0）串口1接收
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //接收缓冲,最大USART_REC_LEN个字节.末字节为换行符
extern u8 DMA_RX_BUF[DMA_REC_LEN];
extern u16 USART_RX_STA;         		//接收状态标记	
//如果想串口中断接收，请不要注释以下宏定义
void uart_init(u32 bound);
void LED_controller(void);
void LED_state_observer(void);
void LED_controller_init(void);
typedef enum 
{
	FLASH_MODE=0,
	STRING_MODE=1

}LED_MODE;

#endif


