#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "can.h"
//ALIENTEK 探索者STM32F407开发板 实验27
//CAN通信实验-库函数版本
	int a=0;

int main(void)
{ 

	u32 i=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	uart1_init(115200);	//初始化串口波特率为115200
	LED_Init();					//初始化LED 
	KEY_Init(); 				//按键初始化 
	CAN1_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CAN初始化环回模式,波特率100Kbps   42M/（6+7+1）/30==100kps
	CAN2_Mode_Init(CAN_BS2_6tq,CAN_BS1_7tq,3,CAN_Mode_Normal);//CAN初始化环回模式,波特率100Kbps   42M/（6+7+1）/30==1Mps
//	for(i=0;i<840000;i++)
//	set_M3508_info();				  
while(1)
	{
		i++;
		if(i%84000==0)
		{
			set_M3508_info();
//			Set_Gimbal_Current1(CAN1,2000,0,0,0);
//			a++;
		}	
		
	}
	
}
