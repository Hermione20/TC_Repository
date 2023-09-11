#include "BSP.h"

void BSP_Init(void)
{
	
		NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);//设置系统中断优先级分组2
	
#if EN_USART1
	uart1_init(115200);	//初始化串口波特率为115200
#endif
#if EN_USART2
	uart2_init(115200);	//初始化串口波特率为115200
#endif
#if EN_USART3
	uart3_init(115200);	//初始化串口波特率为115200
#endif
#if EN_UART4
	uart4_init(115200);	//初始化串口波特率为115200
#endif
#if EN_UART5
	uart5_init(115200);	//初始化串口波特率为115200
#endif
#if EN_UART6
	uart6_init(115200);	//初始化串口波特率为115200
#endif
	
	
}








