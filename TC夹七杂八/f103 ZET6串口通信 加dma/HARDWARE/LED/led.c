#include "led.h"
#include "delay.h"
//////////////////////////////////////////////////////////////////////////////////	 
//STM32F103ZE核心板
//LED驱动代码	   
						  
////////////////////////////////////////////////////////////////////////////////// 	   

//初始化PB5和PE5为输出口.并使能这两个口的时钟		    
//LED IO初始化
void LED_Init(void)
{
 
 GPIO_InitTypeDef  GPIO_InitStructure;
 	
 RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB|RCC_APB2Periph_GPIOE|RCC_APB2Periph_GPIOF, ENABLE);	 //使能PB,PE端口时钟
	
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;				 //LED0-->PB.5 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
 GPIO_SetBits(GPIOB,GPIO_Pin_9);						 //PB.5 输出高
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;				 //LED0-->PB.5 端口配置
 GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //推挽输出
 GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO口速度为50MHz
 GPIO_Init(GPIOB, &GPIO_InitStructure);					 //根据设定参数初始化GPIOB.5
 GPIO_SetBits(GPIOB,GPIO_Pin_8);						 //PB.5 输出高	

 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_Init(GPIOF, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
 GPIO_SetBits(GPIOF,GPIO_Pin_6); 						 //PE.5 输出高 
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_Init(GPIOF, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
 GPIO_SetBits(GPIOF,GPIO_Pin_8); 						 //PE.5 输出高 
	
 GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	    		 //LED1-->PE.5 端口配置, 推挽输出
 GPIO_Init(GPIOF, &GPIO_InitStructure);	  				 //推挽输出 ，IO口速度为50MHz
 GPIO_SetBits(GPIOF,GPIO_Pin_9); 						 //PE.5 输出高 
}


/****LED1/2/3/4的闪烁控制****/
void LED1_FLASH(uint8_t flash_times)
{
	static int s;
	for(s=flash_times;s>0;s--)
	{	
		LED1_ON;
		delay_ms(100);
		LED1_OFF;
		delay_ms(100);
	}
}

void LED2_FLASH(uint8_t flash_times)
{
	static int s;
	for(s=flash_times;s>0;s--)
	{	
	LED2_ON;
	delay_ms(100);
	LED2_OFF;
	delay_ms(100);
	}
}

void LED3_FLASH(uint8_t flash_times)
{
	static int s;
	for(s=flash_times;s>0;s--)
	{
	LED3_ON;
	delay_ms(100);
	LED3_OFF;
	delay_ms(100);
	}
}

void LED4_FLASH(uint8_t flash_times)
{
	static int s;
	for(s=flash_times;s>0;s--)
	{
	LED4_ON;
	delay_ms(100);
	LED4_OFF;
	delay_ms(100);
	}
}

