#ifndef __LED_H
#define __LED_H
#include "sys.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//LED��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/2
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	


//LED�˿ڶ���
#define LED0_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_1)
#define LED1_ON  GPIO_ResetBits(GPIOC,GPIO_Pin_2)
#define LED0_OFF GPIO_SetBits(GPIOC,GPIO_Pin_1)
#define LED1_OFF GPIO_SetBits(GPIOC,GPIO_Pin_2)	 

void LED_Init(void);//��ʼ��		 				    
#endif
