#ifndef __DEISTANCE_MEASURE
#define __DEISTANCE_MEASURE
#include "stm32f4xx.h"

//�������߼����ж����ȼ���õ��ڴ˴���
//���Ŷ���
/*******************************************************/
//#define USE_MEASURE_DISTANCE//ʹ�ò��ģ�鵱ʹ�ô�ģ��ʱ�����Դ��ڲ���ʹ��
#define DEBUG_USART                             USART3
#define DEBUG_USART_CLK                         RCC_APB1Periph_USART3
#define DEBUG_USART_BAUDRATE                    115200 //���ڲ����ʲ�����̫�߻�ʱ�ź�ʧ��

#define DEBUG_USART_RX_GPIO_PORT                GPIOB
#define DEBUG_USART_RX_GPIO_CLK                 RCC_AHB1Periph_GPIOB
#define DEBUG_USART_RX_PIN                      GPIO_Pin_11
#define DEBUG_USART_RX_AF                       GPIO_AF_USART3
#define DEBUG_USART_RX_SOURCE                   GPIO_PinSource11

#define DEBUG_USART_TX_GPIO_PORT                GPIOB
#define DEBUG_USART_TX_GPIO_CLK                 RCC_AHB1Periph_GPIOB
#define DEBUG_USART_TX_PIN                      GPIO_Pin_10
#define DEBUG_USART_TX_AF                       GPIO_AF_USART3
#define DEBUG_USART_TX_SOURCE                   GPIO_PinSource10

#define DEBUG_USART_IRQHandler                  USART3_IRQHandler
#define DEBUG_USART_IRQ                 				USART3_IRQn

//DMA
#define DEBUG_USART_DR_BASE               (&USART3->DR)		
#define RECEIVEBUFF_SIZE                  8				//���ջ����С
#define DEBUG_USART_DMA_CLK               RCC_AHB1Periph_DMA1	
#define DEBUG_USART_DMA_CHANNEL           DMA_Channel_4
#define DEBUG_USART_DMA_STREAM            DMA1_Stream1

#define AVERAGE_FILTER_N  10
#define FREAM_DEFAULT {0x55,0,{0,0,0,0},0,0xAA}
#define MAX_MEASURE  4000              //����������40m

#define total_crc_byte   5
#define frame_head       0x55
#define frame_tail       0xaa //��β�����֡β
#define single_frame_tail  0xff  //���β�����֡β

#define total_crc_byte   5
#define frame_head       0x55
#define frame_tail       0xaa //��β�����֡β
#define single_frame_tail  0xff  //���β�����֡β

#define FREAM_DEFAULT {0x55,0,{0,0,0,0},0,0xAA}

typedef enum{
GET_INFORMATION=0x01,  //��ȡ�豸��Ϣ
SET_MEASURE_FREQUENCY=0x03,//���ò���Ƶ��
SET_MEASURE_FORMAT=0x04,   //�������ݸ�ʽ
SET_MEASURE_MODE=0x0D,      //���ò���ģʽ
START_MEASURE=0x05,     //��������
STOP_MEASURE=0x06,       //ֹͣ����
CONSERVE_SET=0x08,       //��������
SET_BOUNT_RATE=0x12,     //���ò�����
}KEY;


typedef struct{
uint8_t head;
uint8_t key;
uint8_t value[4];
uint8_t CRC8;
uint8_t tail;
}measure_frame_t;


//���β�����crcУ��
typedef struct
{
uint8_t head;
uint8_t key;
uint8_t value[4];
	uint8_t CRC8;
uint8_t tail;
}measure_single_frame_t;

typedef enum{
single_measure=1,
continue_measure=0,	
}measure_mode_e;


void Frame_Make(KEY key,uint8_t* value,uint8_t value_len);
void Frame_Send(void);
void MEASURE_DISTANCE_USART_Config(void);
void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch);
void MEASURE_DISTANCE_USART_Config(void);
void Frame_Make(KEY key,uint8_t* value,uint8_t value_len);


void stop_measure(void);//ÿ���л�����ģʽʱҪ�Ƚ���ֹͣģʽ
void set_continue_measure_mode(void);
void set_single_measure_mode(void);
void start_measure(void);

uint32_t get_the_distance(void);
uint8_t check_the_recive(void);
uint8_t crc_high_first(uint8_t *ptr,uint8_t len);
uint8_t check_the_single_recive(void);
uint32_t from_frame_to_the_distance_mm(void *frame);
extern measure_frame_t measure_frame_sent;
extern measure_frame_t measure_frame_receive;
extern uint8_t frame_value[4];
extern uint32_t distance_last;
extern uint32_t Measure_filter_distance;

extern void MEASURE_DISTANCE_USART_DMA_Config(void);
void Distance_handle(void) ;

#endif
