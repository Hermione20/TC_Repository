#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "can.h"
//ALIENTEK 探索者STM32F407开发板 实验27
//CAN通信实验-库函数版本


int main(void)
{ 

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	delay_init(168);    //初始化延时函数
	uart_init(115200);	//初始化串口波特率为115200
	LED_Init();					//初始化LED 
	KEY_Init(); 				//按键初始化  
	CAN1_Mode_Init(CAN_BS2_6tq,CAN_BS1_7tq,30,CAN_Mode_LoopBack);//CAN初始化环回模式,波特率100Kbps   42M/（6+7+1）/30==100k

 									  
while(1)
	{
		set_M3508_info( 200, 35);
	}
	
}
