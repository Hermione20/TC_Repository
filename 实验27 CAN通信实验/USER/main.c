#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "led.h"
#include "key.h"
#include "can.h"
//ALIENTEK ̽����STM32F407������ ʵ��27
//CANͨ��ʵ��-�⺯���汾


int main(void)
{ 

	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	delay_init(168);    //��ʼ����ʱ����
	uart_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED 
	KEY_Init(); 				//������ʼ��  
	CAN1_Mode_Init(CAN_BS2_6tq,CAN_BS1_7tq,30,CAN_Mode_LoopBack);//CAN��ʼ������ģʽ,������100Kbps   42M/��6+7+1��/30==100k

 									  
while(1)
	{
		set_M3508_info( 200, 35);
	}
	
}
