#ifndef __LED_H
#define __LED_H	 
#include "sys.h"
//////////////////////////////////////////////////////////////////////////////////	 
//STM32F103ZE���İ�
//LED��������	   
							  
////////////////////////////////////////////////////////////////////////////////// 
#define LED2_OFF 	GPIO_ResetBits(GPIOF,GPIO_Pin_6);
#define LED1_OFF   GPIO_ResetBits(GPIOB,GPIO_Pin_9);	

#define LED3_OFF 	GPIO_ResetBits(GPIOF,GPIO_Pin_8);
#define LED4_OFF	GPIO_ResetBits(GPIOF,GPIO_Pin_9);

#define LED2_ON 	GPIO_SetBits(GPIOF,GPIO_Pin_6);
#define LED1_ON  GPIO_SetBits(GPIOB,GPIO_Pin_9);	 

#define LED3_ON	GPIO_SetBits(GPIOF,GPIO_Pin_8);
#define LED4_ON	GPIO_SetBits(GPIOF,GPIO_Pin_9);

#define BEEP_ON   GPIO_SetBits(GPIOB,GPIO_Pin_8);
#define BEEP_OFF  GPIO_ResetBits(GPIOB,GPIO_Pin_8);

void LED_Init(void);//��ʼ��
void LED1_FLASH(uint8_t flash_times);
void LED2_FLASH(uint8_t flash_times);
void LED3_FLASH(uint8_t flash_times);
void LED4_FLASH(uint8_t flash_times);
		 				    
#endif
