#ifndef __USART_H__
#define __USART_H__
#include "public.h"


#define EN_USART1 									    1//Ĭ��˫����

#define EN_USART2       						  	1//Ĭ��˫����

#define EN_USART3												1//Ĭ�ϵ�����

#define EN_UART4												1//Ĭ�ϵ�����

#define EN_UART5												1//��ѡ����
#define EN_UART5_DMA_SECOND_FIFO 				1

#define EN_USART6												1//Ĭ�ϵ�����

/*
*********************************************************************************************************
*                                     �жϵ��ú����ӿ�
*********************************************************************************************************
*/
#define USART1_Data_Receive_Process_0				do{RemoteDataPrcess(_USART1_DMA_RX_BUF[0],this_time_rx_len1);infantry_mode_switch_task();}while(0);																																															
#define USART1_Data_Receive_Process_1				do{RemoteDataPrcess(_USART1_DMA_RX_BUF[1],this_time_rx_len1);infantry_mode_switch_task();}while(0);

#define USART2_Data_Receive_Process_0				do{}while(0);
#define USART2_Data_Receive_Process_1				do{}while(0);
	
#define USART3_Data_Receive_Process					do{CH100_getDATA(_USART3_RX_BUF,&gimbal_gyro);}while(0);
#define UART4_Data_Receive_Process					do{vision_process_general_message(_UART4_DMA_RX_BUF,length);}while(0);
	
#define UART5_Data_Receive_Process_0				do{judgement_data_handle(_UART5_DMA_RX_BUF[0],this_time_rx_len5);}while(0);
#define UART5_Data_Receive_Process_1				do{judgement_data_handle(_UART5_DMA_RX_BUF[1],this_time_rx_len5);}while(0);

#define USART6_Data_Receive_Process					do{CH100_getDATA(_USART6_DMA_RX_BUF,&gimbal_gyro);}while(0);
/*
*********************************************************************************************************
*                                          MACROS
*********************************************************************************************************
*///������ 
//	static uint8_t _USART1_DMA_RX_BUF[2][BSP_USART1_DMA_RX_BUF_LEN];//˫���������
//	static uint8_t _USART2_DMA_RX_BUF[2][BSP_USART2_DMA_RX_BUF_LEN];//˫���������
//	static uint8_t _USART3_RX_BUF[BSP_USART2_DMA_RX_BUF_LEN];//ch100�����������
//	static uint8_t _UART4_DMA_RX_BUF[UART4_RX_BUF_LENGTH];	
//	static uint8_t _USART6_DMA_RX_BUF[USART6_RX_BUF_LENGTH];
//	static uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];
//	static uint8_t UART5_DMA_TX_BUF[UART5_TX_BUF_LENGTH];
#define UART4_RX_BUF_LENGTH       100

#define BSP_USART1_DMA_RX_BUF_LEN 64u 
#define BSP_USART2_DMA_RX_BUF_LEN 100
#define BSP_UART5_DMA_RX_BUF_LEN  512
#define BSP_USART6_RX_BUF_LENGTH  100


#define UART4_TX_BUF_LENGTH       100
#define UART5_TX_BUF_LENGTH       150

#define RC_FRAME_LENGTH           18u

#define BSP_USART1_RX_BUF_SIZE_IN_FRAMES       (BSP_USART1_RX_BUF_SIZE / RC_FRAME_LENGTH)
#define BSP_UART5_RX_BUF_SIZE_IN_FRAMES 			 (BSP_UART5_DMA_RX_BUF_LEN / RC_FRAME_LENGTH)  
/*
*********************************************************************************************************
*                                             FUNCTION PROTOTYPES
*********************************************************************************************************
*/
#if EN_USART3												
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

//  #define USART_CH100_DMA_TX_IRQn               DMA2_Stream6_IRQn
//  #define USART_CH100_DMA_RX_IRQn               DMA2_Stream1_IRQn
//  #define USART_CH100_DMA_TX_IRQHandler         DMA2_Stream6_IRQHandler
//  #define USART_CH100_DMA_RX_IRQHandler         DMA2_Stream1_IRQHandler
#endif


void usart1_init(uint32_t baud_rate);
void usart2_init(u32 bound);
void usart3_init(u32 bound);
void uart4_init (u32 bound);
void uart5_init (u32 bound);
void usart6_init(void);

void Uart4DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr);
void Uart4SendByteInfoProc(u8 nSendInfo);
void Uart4SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);

extern uint8_t UART4_DMA_TX_BUF[UART4_TX_BUF_LENGTH];

#endif
