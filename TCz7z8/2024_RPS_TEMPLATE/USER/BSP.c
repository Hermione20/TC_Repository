#include "BSP.h"

void BSP_Init(void)
{
	
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
#if EN_USART1
	usart1_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_USART2
	usart2_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_USART3
	usart3_init(921600);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_UART4
	uart4_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_UART5
	uart5_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_UART6
	usart6_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
	

		CAN1_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������100Kbps   42M/��6+7+1��/30==1Mps

		CAN2_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������100Kbps   42M/��6+7+1��/30==1Mps

}








