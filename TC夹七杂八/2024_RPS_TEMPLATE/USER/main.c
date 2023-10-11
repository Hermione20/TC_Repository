#include "main.h"



int main()
{
		
   BSP_Init();

	while(1)
	{
		
//			printf("Success!");
		USART_SendData(USART3,0X10);
			delay_ms(10);
	}
}
