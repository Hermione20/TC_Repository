#include "BSP.h"

void BSP_Init(void)
{
	
NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//����ϵͳ�ж����ȼ�����2
	
#if EN_USART1
	uart1_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_USART2
	uart2_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_USART3
	uart3_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_UART4
	uart4_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_UART5
	uart5_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
#if EN_UART6
	uart6_init(115200);	//��ʼ�����ڲ�����Ϊ115200
#endif
	
#if CAN1_RX0_INT_ENABLE	| CAN1_TX0_INT_ENABLE
		CAN1_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������100Kbps   42M/��6+7+1��/30==1Mps
#endif

#if CAN2_TX0_INT_ENABLE | CAN2_RX0_INT_ENABLE
		CAN2_Mode_Init(CAN_BS2_4tq,CAN_BS1_9tq,3,CAN_Mode_Normal);//CAN��ʼ������ģʽ,������100Kbps   42M/��6+7+1��/30==1Mps
#endif
}








