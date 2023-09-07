#include "sys.h"
#include "usart.h"	
#include "led.h"
#include "delay.h"
#include "dma.h"
////////////////////////////////////////////////////////////////////////////////// 	 
//���ʹ��ucos,����������ͷ�ļ�����.
#if SYSTEM_SUPPORT_OS
#include "includes.h"					//ucos ʹ��	  
#endif
//////////////////////////////////////////////////////////////////////////////////	 
//STM32F103ZE���İ�
//����1��ʼ��		
////////////////////////////////////////////////////////////////////////////////// 	  


//////////////////////////////////////////////////////////////////
//�������´���,֧��printf����,������Ҫѡ��use MicroLIB	  
#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
void _sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*ʹ��microLib�ķ���*/
 /* 
int fputc(int ch, FILE *f)
{
	USART_SendData(USART1, (uint8_t) ch);

	while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET) {}	
   
    return ch;
}
int GetKey (void)  { 

    while (!(USART1->SR & USART_FLAG_RXNE));

    return ((int)(USART1->DR & 0x1FF));
}
*/
 
#if EN_USART1_RX   //���ʹ���˽���
//����1�жϷ������
//ע��,��ȡUSARTx->SR�ܱ���Ī������Ĵ���   	

u8 DMA_RX_BUF[DMA_REC_LEN]; 			  //DMA�Ľ��ջ��壬���DMA_REC_LEN���ֽ�.

u8 USART_RX_BUF[USART_REC_LEN];     //���ջ���,���USART_REC_LEN���ֽ�.
//����״̬
//bit15��	������ɱ�־
//bit14��	���յ�0x0d
//bit13~0��	���յ�����Ч�ֽ���Ŀ
u16 USART_RX_STA=0;       //����״̬���	  
 

//��־LED״̬�ĺ��ı���
//bit7~4, �ĸ�LED��״̬
//bit3~0, LED����˸����
u8 USART_LED_STA=0;   		//LED״̬���



u8 temp_rx_buf;
uint8_t i,m,j,times;
int led_mode;
uint8_t flash_times=0;
uint8_t LED_count=0;
uint8_t frame_is_ok=1;
uint16_t length=0;

/****LED���ƺ���������ʼ��****/
void LED_controller_init()
{
	 i=0;
	 j=0;
	 m=0;
	 temp_rx_buf=0;
	 times=0;
	 led_mode=1;
	 flash_times=0;
	 LED_count=0;
	 frame_is_ok=1;
	 length=0;
}

/****LED���ƿ���****/
void LED_controller()
{		
	if(led_mode==1)  //LED�ķ���˸ģʽ
	{	
		LED1_OFF;LED2_OFF;LED3_OFF;LED4_OFF;
		if(((USART_LED_STA&0xf0)>>4)&0x01)
		{LED1_ON;}
		else
		{LED1_OFF;}
		if((((USART_LED_STA&0xf0)>>4)&0x02)>>1)
		{LED2_ON;}
		else
		{LED2_OFF;}
		if((((USART_LED_STA&0xf0)>>4)&0x04)>>2)
		{LED3_ON;}
		else
		{LED3_OFF;}
		if((((USART_LED_STA&0xf0)>>4)&0x08)>>3)
		{LED4_ON;}
		else
		{LED4_OFF;}
	}
	else if(led_mode==0) //LED��˸ģʽ
	{
		
	if(((USART_LED_STA&0xf0)>>4)==1)
		{
			LED1_FLASH(flash_times);
			USART_LED_STA=0;
		}
		else
		{LED1_OFF;}
		if(((USART_LED_STA&0xf0)>>5)==1)
		{
			LED2_FLASH(flash_times);
			USART_LED_STA=0;
		}
		else
		{LED2_OFF;}
		if(((USART_LED_STA&0xf0)>>6)==1)
		{
			LED3_FLASH(flash_times);
			USART_LED_STA=0;
		}
		else
		{LED3_OFF;}
		if(((USART_LED_STA&0xf0)>>7)==1)
		{
			LED4_FLASH(flash_times);
			USART_LED_STA=0;
		}
		else
		{LED4_OFF;}
	}
		
}


/****LED��״̬�۲⣬����ָ��ı�״̬****/
void LED_state_observer()
{
	
	if(USART_RX_STA&0x8000)
		{
			frame_is_ok =1;
			length=USART_RX_STA&0x3fff;
			
/***************************			
			LEDn Flash n�����ж�		
*****************************/			
				
			if(length>0\
					&&USART_RX_BUF[0]=='L'\
					&&USART_RX_BUF[1]=='E'\
					&&USART_RX_BUF[2]=='D'\
					&&USART_RX_BUF[4]==0x20\
					&&USART_RX_BUF[5]=='F'\
					&&USART_RX_BUF[6]=='l'\
					&&USART_RX_BUF[7]=='a'\
					&&USART_RX_BUF[8]=='s'\
					&&USART_RX_BUF[9]=='h'\
					&&USART_RX_BUF[10]==0x20
				 )
				{
					led_mode=0;
					switch (USART_RX_BUF[3])
					{
						case '1':
							USART_LED_STA&=0x00;
							USART_LED_STA|=0x10;
							USART_LED_STA|=(USART_RX_BUF[11]&=0x0f);
							flash_times   =(USART_RX_BUF[11]&=0x0f);
						break;
						case '2':
							USART_LED_STA&=0x00;
							USART_LED_STA|=0x20;
							USART_LED_STA|=(USART_RX_BUF[11]&=0x0f);
							flash_times   =(USART_RX_BUF[11]&=0x0f);
						break;
						case '3':
							USART_LED_STA&=0x00;
							USART_LED_STA|=0x40;
							USART_LED_STA|=(USART_RX_BUF[11]&=0x0f);
							flash_times   =(USART_RX_BUF[11]&=0x0f);
						break;
						case '4':
							USART_LED_STA&=0x00;
							USART_LED_STA|=0x80;
							USART_LED_STA|=(USART_RX_BUF[11]&=0x0f);
							flash_times   =(USART_RX_BUF[11]&=0x0f);
						break;
						default:
							frame_is_ok =0;
							break;

					}							
					USART_RX_STA=0;				
				}
/***************************			
		LEDn Turn Off�����ж�		
*****************************/	
				
				else if(length>0\
					&&USART_RX_BUF[0]=='L'\
					&&USART_RX_BUF[1]=='E'\
					&&USART_RX_BUF[2]=='D'\
					&&USART_RX_BUF[4]==0x20\
					&&USART_RX_BUF[5]=='T'\
					&&USART_RX_BUF[6]=='u'\
					&&USART_RX_BUF[7]=='r'\
					&&USART_RX_BUF[8]=='n'\
					&&USART_RX_BUF[9]==0x20\
					&&USART_RX_BUF[10]=='O'\
					&&USART_RX_BUF[11]=='f'\
					&&USART_RX_BUF[12]=='f'\
				 )
				{
					led_mode=1;
					switch (USART_RX_BUF[3])
					{
						case '1':
							USART_LED_STA&=~(1<<4);
						break;
						case '2':
							USART_LED_STA&=~(1<<5);
						break;
						case '3':
							USART_LED_STA&=~(1<<6);
						break;
						case '4':
							USART_LED_STA&=~(1<<7);
						break;
						default:
							frame_is_ok =0;
							break;
					}
						USART_RX_STA=0;
				}
				
/***************************			
	  LEDn Turn On�����ж�				
*****************************/				
				else if(length>0\
					&&USART_RX_BUF[0]=='L'\
					&&USART_RX_BUF[1]=='E'\
					&&USART_RX_BUF[2]=='D'\
					&&USART_RX_BUF[4]==0x20\
					&&USART_RX_BUF[5]=='T'\
					&&USART_RX_BUF[6]=='u'\
					&&USART_RX_BUF[7]=='r'\
					&&USART_RX_BUF[8]=='n'\
					&&USART_RX_BUF[9]==0x20\
					&&USART_RX_BUF[10]=='O'\
					&&USART_RX_BUF[11]=='n'\
				  )
				{
					led_mode=1;
					switch (USART_RX_BUF[3])
					{
						case '1':
							USART_LED_STA|=(1<<4);
						break;
						case '2':
							USART_LED_STA|=(1<<5);
						break;
						case '3':
							USART_LED_STA|=(1<<6);
						break;
						case '4':
							USART_LED_STA|=(1<<7);
						break;
						default:
							frame_is_ok =0;
							break;
					}
        		USART_RX_STA=0;
				}
				
				
/***************************			
	  LEDstring�����ж�			
*****************************/				
				else if(length>0\
					&&USART_RX_BUF[0]=='L'\
					&&USART_RX_BUF[1]=='E'\
					&&USART_RX_BUF[2]=='D'\
					&&USART_RX_BUF[4]!=0x20
				 )
				{
					led_mode=1;
					for(LED_count=3;LED_count<length;LED_count++)
					{								
							USART_LED_STA=0;
							temp_rx_buf=USART_RX_BUF[LED_count]-0x30;
							if(temp_rx_buf>9&&temp_rx_buf<=96)temp_rx_buf=(USART_RX_BUF[LED_count]-0x07);
							else if(temp_rx_buf>96)temp_rx_buf-=39;
							USART_LED_STA=(temp_rx_buf<<4);
							delay_ms(200);
							LED_controller();
							delay_ms(200);	
		
					
					}
					USART_RX_STA=0;
				}		
				else
				{				
				frame_is_ok =0;
				printf("��������\r\n");
				}
				
			}
		
			else
		{
			times++;
			if(times%500==0)
			{
				printf("\r\n");
			}
			if(times%2000==0)
			{	
				printf("����LEDn Flash n��LEDn Turn Off��LEDn Turn On��LEDstring,�Իس�������\r\n");  
				delay_ms(100);  
			}
		}
		USART_RX_STA=0;
} 



void uart_init(u32 bound){
  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	 
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1|RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��USART1��GPIOAʱ��
  
	//USART1_TX   GPIOA.9
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA.9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.9
   
  //USART1_RX	  GPIOA.10��ʼ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;//��������
  GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��GPIOA.10  

  //Usart1 NVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=3 ;//��ռ���ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;		//�����ȼ�3
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
  
   //USART ��ʼ������

	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ

  USART_Init(USART1, &USART_InitStructure); //��ʼ������1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
//	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);//�������ڿ����ж�
  USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1 

}

void USART1_IRQHandler(void)                	//����1�жϷ������
	{
	u8 Res;
#if SYSTEM_SUPPORT_OS 		//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntEnter();    
#endif
		
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)  //�����ж�(���յ������ݱ�����0x0d 0x0a��β)
		{
			
		MYDMA_Enable(DMA2_Channel4);
		Res =USART_ReceiveData(USART1);	//��ȡ���յ�������
		
		if((USART_RX_STA&0x8000)==0)//����δ���
			{
			if(USART_RX_STA&0x4000)//���յ���0x0d
				{
				if(Res!=0x0a)USART_RX_STA=0;//���մ���,���¿�ʼ
				else 
				{
					USART_RX_STA|=0x8000;	//���������
				}
				}
		 else //��û�յ�0X0D
				{	
				if(Res==0x0d)USART_RX_STA|=0x4000;
				else
					{
					USART_RX_BUF[USART_RX_STA&0X3FFF]=Res ;
					USART_RX_STA++;
					if(USART_RX_STA>(USART_REC_LEN-1))USART_RX_STA=0;//�������ݴ���,���¿�ʼ����	  
					}		 
				}
			}   		 
     }

		
		 
#if SYSTEM_SUPPORT_OS 	//���SYSTEM_SUPPORT_OSΪ�棬����Ҫ֧��OS.
	OSIntExit();  											 
#endif
} 
#endif	

