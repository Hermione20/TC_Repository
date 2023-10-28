#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "sys.h" 
//////////////////////////////////////////////////////////////////////////////////	 

//STM32F103ZE���İ�
//����1��ʼ��		   

//********************************************************************************
#define DMA_REC_LEN         50		//�����������ֽ��� 50
#define USART_REC_LEN  			50  	//�����������ֽ��� 50
#define EN_USART1_RX 			1		//ʹ�ܣ�1��/��ֹ��0������1����
	  	
extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з�
extern u8 DMA_RX_BUF[DMA_REC_LEN];
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart_init(u32 bound);
void LED_controller(void);
void LED_state_observer(void);
void LED_controller_init(void);
typedef enum 
{
	FLASH_MODE=0,
	STRING_MODE=1

}LED_MODE;

#endif


