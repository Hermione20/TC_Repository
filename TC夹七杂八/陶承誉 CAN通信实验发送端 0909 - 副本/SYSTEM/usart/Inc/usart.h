#ifndef __USART_H
#define __USART_H
#include "stdio.h"	
#include "stm32f4xx_conf.h"
#include "stm32f4xx_dma.h"
#include "sys.h" 



#define USART_REC_LEN  			200  	//�����������ֽ��� 200

#define EN_USART1 								1
#define EN_USART1_RX 							1		//ʹ�ܣ�1��/��ֹ��0������1����
#define EN_USART1_TX							1
#define EN_USART1_DMA							1
#define EN_USART1_DMA_RX					1
#define EN_USART1_DMA_TX					1
#define EN_USART1_DMA_SECOND_FIFO 1
#define EN_USART1_RX_IRQ 					1

#define EN_USART2       					1//0
#define EN_UART2_RX       				1
#define EN_UART2_TX								1
#define EN_UART2_DMA							1
#define EN_UART2_DMA_TX      		  1
#define EN_UART2_DMA_RX      		  1
#define EN_UART2_DMA_SECOND_FIFO  1
#define EN_UART2_DMA_TX_IRQ			  1

#define EN_USART3									1
#define EN_USART3_RX							1
#define EN_USART3_DMA							1
#define EN_USART3_DMA_RX					1
#define EN_USART3_DMA_TX					1
#define EN_USART3_DMA_SECOND_FIFO 1


#define EN_UART4									1
#define EN_UART4_RX       				1
#define EN_UART4_TX								1
#define EN_UART4_DMA							1
#define EN_UART4_DMA_TX      		  1
#define EN_UART4_DMA_RX      			1
#define EN_UART4_DMA_SECOND_FIFO  1
#define EN_UART4_DMA_TX_IRQ				1


#define EN_UART5									1
#define EN_UART5_RX								1
#define EN_UART5_TX								1
#define EN_UART5_DMA          		1
#define EN_UART5_DMA_TX						1
#define EN_UART5_DMA_RX       		1
#define EN_UART5_DMA_SECOND_FIFO  1
#define EN_UART5_RX_IQR       		1

#define EN_UART6									1//0
#define EN_UART6_RX								1
#define EN_UART6_TX								1
#define EN_UART6_DMA          		1
#define EN_UART6_DMA_TX						1
#define EN_UART6_DMA_RX       		1
#define EN_UART6_DMA_SECOND_FIFO  1
#define EN_UART6_RX_IQR       		1   

 /* Definition for USART_CH100 resources ******************************************/
  #define USART_CH100                           USART3
  #define USART_CH100_CLK                       RCC_APB1Periph_USART3
  #define USART_CH100_CLK_INIT                  RCC_APB1PeriphClockCmd
  #define USART_CH100_IRQn                      USART3_IRQn
  #define USART_CH100_IRQHandler                USART3_IRQHandler

  #define USART_CH100_TX_PIN                    GPIO_Pin_10                
  #define USART_CH100_TX_GPIO_PORT              GPIOB                      
  #define USART_CH100_TX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART_CH100_TX_SOURCE                 GPIO_PinSource10
  #define USART_CH100_TX_AF                     GPIO_AF_USART3

  #define USART_CH100_RX_PIN                    GPIO_Pin_11              
  #define USART_CH100_RX_GPIO_PORT              GPIOB                   
  #define USART_CH100_RX_GPIO_CLK               RCC_AHB1Periph_GPIOB
  #define USART_CH100_RX_SOURCE                 GPIO_PinSource11
  #define USART_CH100_RX_AF                     GPIO_AF_USART3

  /* Definition for DMAx resources ********************************************/
  #define USART_CH100_DR_ADDRESS                ((uint32_t)USART3 + 0x04) 

  #define USART_CH100_DMA                       DMA1
  #define USART_CH100_DMAx_CLK                  RCC_AHB1Periph_DMA1
     
  #define USART_CH100_TX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH100_TX_DMA_STREAM             DMA1_Stream3
  #define USART_CH100_TX_DMA_FLAG_FEIF          DMA_FLAG_FEIF3
  #define USART_CH100_TX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF3
  #define USART_CH100_TX_DMA_FLAG_TEIF          DMA_FLAG_TEIF3
  #define USART_CH100_TX_DMA_FLAG_HTIF          DMA_FLAG_HTIF3
  #define USART_CH100_TX_DMA_FLAG_TCIF          DMA_FLAG_TCIF3
              
  #define USART_CH100_RX_DMA_CHANNEL            DMA_Channel_4
  #define USART_CH100_RX_DMA_STREAM             DMA1_Stream1
  #define USART_CH100_RX_DMA_FLAG_FEIF          DMA_FLAG_FEIF1
  #define USART_CH100_RX_DMA_FLAG_DMEIF         DMA_FLAG_DMEIF1
  #define USART_CH100_RX_DMA_FLAG_TEIF          DMA_FLAG_TEIF1
  #define USART_CH100_RX_DMA_FLAG_HTIF          DMA_FLAG_HTIF1
  #define USART_CH100_RX_DMA_FLAG_TCIF          DMA_FLAG_TCIF1


#define USART1_Data_Receive_Process				do{}while(0)   		
#define USART2_Data_Receive_Process				do{}while(0) 
#define USART3_Data_Receive_Process				do{}while(0) 
#define USART4_Data_Receive_Process				do{}while(0) 
#define USART5_Data_Receive_Process				do{}while(0) 
#define USART6_Data_Receive_Process				do{}while(0) 


extern u8  USART_RX_BUF[USART_REC_LEN]; //���ջ���,���USART_REC_LEN���ֽ�.ĩ�ֽ�Ϊ���з� 
extern u16 USART_RX_STA;         		//����״̬���	
//����봮���жϽ��գ��벻Ҫע�����º궨��
void uart1_init(u32 bound);
void uart2_init(u32 bound);
void uart3_init(u32 bound);
void uart4_init(u32 bound);
void uart5_init(u32 bound);
void uart6_init(u32 bound);

void UART1_MYDMA_Enable(u16 ndtr);
void UART2_MYDMA_Enable(u16 ndtr);
void UART3_MYDMA_Enable(u16 ndtr);
void UART4_MYDMA_Enable(u16 ndtr);
void UART5_MYDMA_Enable(u16 ndtr);
void UART6_MYDMA_Enable(u16 ndtr);
#endif


