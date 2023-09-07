#include "led.h"
#include "delay.h"
#include "key.h"
#include "sys.h"
#include "usart.h"
#include "dma.h"

 
/************************************************
 //STM32F103ZE���İ�
 
 ����ʵ�� 

************************************************/
uint32_t a,b;

 int main(void)
 {		

	delay_init();	    	 //��ʱ������ʼ��	  
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2); //����NVIC�жϷ���2:2λ��ռ���ȼ���2λ��Ӧ���ȼ�
	uart_init(115200);	 //���ڳ�ʼ��Ϊ115200
 	LED_Init();			     //LED�˿ڳ�ʼ��
	KEY_Init();          //��ʼ���밴�����ӵ�Ӳ���ӿ�
	LED_controller_init();//��ʼ����LED�йز���
  MYDMA_Config(DMA1_Channel5,(uint32_t)(&(USART1->DR)),(uint32_t)DMA_RX_BUF,DMA_REC_LEN);

	 while(1)
	{		
		LED_controller();		 //LED����
		delay_ms(100);
		LED_state_observer();//LED״̬�۲�
	}	 
 }

