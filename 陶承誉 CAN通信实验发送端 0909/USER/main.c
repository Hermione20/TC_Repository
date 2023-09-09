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

	u32 i=0;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	uart1_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	uart4_init(115200);	//��ʼ�����ڲ�����Ϊ115200
	LED_Init();					//��ʼ��LED 
	KEY_Init(); 				//������ʼ�� 
	CAN1_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,30,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������100Kbps   42M/��6+7+1��/30==100kps
	CAN2_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������100Kbps   42M/��6+7+1��/30==1Mps
		  
while(1)
	{
		i++;
		if(i%(84000)==0)
		{
			set_M3508_info();//CAN1�����ն˷�����Ϣ
		}	
		
	}
	
}
