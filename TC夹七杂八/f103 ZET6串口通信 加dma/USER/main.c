#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "dma.h"

 
/************************************************
 //STM32F103ZE核心板
 
 串口实验 

************************************************/
uint32_t a,b;

 int main(void)
 {		

	delay_init();	    	 //延时函数初始化	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //设置NVIC中断分组2:2位抢占优先级，2位响应优先级
	uart_init(115200);	 //串口初始化为115200
 	LED_Init();			     //LED端口初始化
	KEY_Init();          //初始化与按键连接的硬件接口
	LED_controller_init();//初始化与LED有关参数
  MYDMA_Config(DMA1_Channel5,(uint32_t)(&(USART1->DR)),(uint32_t)DMA_RX_BUF,DMA_REC_LEN);

	 while(1)
	{		
		LED_controller();		 //LED控制
		delay_ms(100);
		LED_state_observer();//LED状态观测
	}	 
 }

