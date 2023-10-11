#include "usart3.h"

u8 USART3_DMA_RX_BUF[USART3_RX_BUF_LENGTH] = {0};
u8 USART3_DMA_TX_BUF[USART3_TX_BUF_LENGTH] = {1,2,3,4,5};

//int fputc(int ch, FILE *f)
//{
//
//    USART_SendData(USART3, (uint8_t)ch);
//	   while (USART_GetFlagStatus(USART3,USART_FLAG_TC) == RESET);
//    return ch;
//}

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
  while((USART3->SR&0X40)==0);//ѭ������,ֱ���������
  USART3->DR = (u8) ch;
  return ch;
}
#endif
//void USART3_Configuration_Send()
//{
//    USART_InitTypeDef usart;
//    GPIO_InitTypeDef  gpio;
//    NVIC_InitTypeDef  nvic;
//		DMA_InitTypeDef dma;

//		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
////    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
//		RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
//
//		//����1��Ӧ���Ÿ���ӳ��
//		GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA9����ΪUSART1
//		GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA10����ΪUSART1
//
//		//USART1�˿�����
//		gpio.GPIO_Pin 	= GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9��GPIOA10
//		gpio.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
//		gpio.GPIO_OType = GPIO_OType_PP;
//		gpio.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
//		gpio.GPIO_PuPd 	= GPIO_PuPd_NOPULL; //����
//		GPIO_Init(GPIOB,&gpio); //��ʼ��PA9��PA10

//    USART_DeInit(USART3);
//    USART_StructInit(&usart);
//    usart.USART_BaudRate = 115200;
//    usart.USART_WordLength = USART_WordLength_8b;
//    usart.USART_StopBits = USART_StopBits_1;
//    usart.USART_Parity = USART_Parity_No;
//    usart.USART_Mode = USART_Mode_Tx;
//    usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//    USART_Init(USART3, &usart);
//
//    USART_Cmd(USART3,ENABLE);
//}

void USART3_Configuration_Send()
{
  USART_InitTypeDef usart;
  GPIO_InitTypeDef  gpio;
  NVIC_InitTypeDef  nvic;
  DMA_InitTypeDef dma;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  //����1��Ӧ���Ÿ���ӳ��
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA9����ΪUSART1
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA10����ΪUSART1

  //USART1�˿�����
  gpio.GPIO_Pin 	= GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9��GPIOA10
  gpio.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
  gpio.GPIO_PuPd 	= GPIO_PuPd_NOPULL; //����
  GPIO_Init(GPIOB,&gpio); //��ʼ��PA9��PA10

  USART_DeInit(USART3);
  USART_StructInit(&usart);
  usart.USART_BaudRate = 115200;
  usart.USART_WordLength = USART_WordLength_8b;
  usart.USART_StopBits = USART_StopBits_1;
  usart.USART_Parity = USART_Parity_No;
  usart.USART_Mode = USART_Mode_Tx;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART3, &usart);

  USART_Cmd(USART3,ENABLE);

  USART_DMACmd(USART3, USART_DMAReq_Tx, ENABLE);

  DMA_Cmd(DMA1_Stream3, DISABLE);                           // ��DMAͨ��
  DMA_DeInit(DMA1_Stream3);
  while(DMA_GetCmdStatus(DMA1_Stream3) != DISABLE) {}
  dma.DMA_Channel = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART3->DR);
  dma.DMA_Memory0BaseAddr   	= (uint32_t)&USART3_DMA_TX_BUF[0];
  dma.DMA_DIR 			    = DMA_DIR_MemoryToPeripheral;
  dma.DMA_BufferSize			= 0;//sizeof(USART1_DMA_TX_BUF);
  dma.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
  dma.DMA_Mode 				= DMA_Mode_Normal;
  dma.DMA_Priority 			= DMA_Priority_Medium;
  dma.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
  dma.DMA_FIFOThreshold 		= DMA_FIFOThreshold_Full;
  dma.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
  dma.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream3,&dma);

//	DMA_Cmd(DMA1_Stream3, DISABLE);                           // ��DMAͨ��
  nvic.NVIC_IRQChannel = DMA1_Stream3_IRQn;   // ����DMAͨ�����ж�����
  nvic.NVIC_IRQChannelPreemptionPriority = 2;     // ���ȼ�����
  nvic.NVIC_IRQChannelSubPriority = 1;
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  DMA_ITConfig(DMA1_Stream3,DMA_IT_TC,ENABLE);


}

void Usart3DmaSendDataProc(DMA_Stream_TypeDef *DMA_Streamx,u16 ndtr)

{
//    DMA_Cmd(DMA_Streamx, DISABLE);                      //�ر�DMA����
//    while (DMA_GetCmdStatus(DMA_Streamx) != DISABLE){}  //ȷ��DMA���Ա�����
  DMA_SetCurrDataCounter(DMA_Streamx,ndtr);          //���ݴ�����
  DMA_Cmd(DMA_Streamx, ENABLE);                      //����DMA����
}



void DMA1_Stream3_IRQHandler(void)
{
  //�����־
  if(DMA_GetFlagStatus(DMA1_Stream3,DMA_FLAG_TCIF3)!=RESET)//�ȴ�DMA1_Steam3�������
    {
      DMA_Cmd(DMA1_Stream3, DISABLE);                      //�ر�DMA����
      DMA_ClearFlag(DMA1_Stream3,DMA_FLAG_TCIF3);//���DMA1_Steam3������ɱ�־
    }

}


//���͵��ֽ�
void Usart3SendByteInfoProc(u8 nSendInfo)
{
  u8 *pBuf = NULL;
  //ָ���ͻ�����
  pBuf = USART3_DMA_TX_BUF;
  *pBuf++ = nSendInfo;

  Usart3DmaSendDataProc(DMA1_Stream3,1); //��ʼһ��DMA���䣡

}

//���Ͷ��ֽ�

void Usart3SendBytesInfoProc(u8* pSendInfo, u16 nSendCount)
{
  u16 i = 0;
  u8 *pBuf = NULL;
  //ָ���ͻ�����
  pBuf = USART3_DMA_TX_BUF;
  for (i=0; i<nSendCount; i++)
    {
      *(pBuf+i) = pSendInfo[i];
    }

  //DMA���ͷ�ʽ

  Usart3DmaSendDataProc(DMA1_Stream3,nSendCount); //��ʼһ��DMA���䣡

}

void USART3_Configuration_For_Hi220()
{
  USART_InitTypeDef usart;
  GPIO_InitTypeDef  gpio;
  NVIC_InitTypeDef  nvic;
  DMA_InitTypeDef dma;

  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);

  //����1��Ӧ���Ÿ���ӳ��
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource10,GPIO_AF_USART3); //GPIOA9����ΪUSART1
  GPIO_PinAFConfig(GPIOB,GPIO_PinSource11,GPIO_AF_USART3); //GPIOA10����ΪUSART1

  //USART1�˿�����
  gpio.GPIO_Pin 	= GPIO_Pin_10 | GPIO_Pin_11; //GPIOA9��GPIOA10
  gpio.GPIO_Mode 	= GPIO_Mode_AF;//���ù���
  gpio.GPIO_OType = GPIO_OType_PP;
  gpio.GPIO_Speed = GPIO_Speed_50MHz;	//�ٶ�50MHz
  gpio.GPIO_PuPd 	= GPIO_PuPd_NOPULL; //����
  GPIO_Init(GPIOB,&gpio); //��ʼ��PA9��PA10

  USART_DeInit(USART3);
//    USART_StructInit(&usart);
  usart.USART_BaudRate = 115200;
  usart.USART_WordLength = USART_WordLength_8b;
  usart.USART_StopBits = USART_StopBits_1;
  usart.USART_Parity = USART_Parity_No;
  usart.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
  usart.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
  USART_Init(USART3, &usart);


  USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);

  DMA_DeInit(DMA1_Stream1);
//    DMA_StructInit(&dma);
  dma.DMA_Channel = DMA_Channel_4;
  dma.DMA_PeripheralBaseAddr	= (uint32_t)(&USART3->DR);
  dma.DMA_Memory0BaseAddr   	= (uint32_t)&USART3_DMA_RX_BUF[0];
  dma.DMA_DIR 			    = DMA_DIR_PeripheralToMemory;
  dma.DMA_BufferSize			= USART3_RX_BUF_LENGTH;//sizeof(USART1_DMA_RX_BUF);
  dma.DMA_PeripheralInc 		= DMA_PeripheralInc_Disable;
  dma.DMA_MemoryInc 			= DMA_MemoryInc_Enable;
  dma.DMA_PeripheralDataSize 	= DMA_PeripheralDataSize_Byte;
  dma.DMA_MemoryDataSize 		= DMA_MemoryDataSize_Byte;
  dma.DMA_Mode 				= DMA_Mode_Normal;
  dma.DMA_Priority 			= DMA_Priority_Medium;
  dma.DMA_FIFOMode 			= DMA_FIFOMode_Disable;
  dma.DMA_FIFOThreshold 		= DMA_FIFOThreshold_1QuarterFull;
  dma.DMA_MemoryBurst 		= DMA_MemoryBurst_Single;
  dma.DMA_PeripheralBurst 	= DMA_PeripheralBurst_Single;
  DMA_Init(DMA1_Stream1, &dma);
  DMA_Cmd(DMA1_Stream1, ENABLE);

  //����Memory1,Memory0�ǵ�һ��ʹ�õ�Memory
  //ʹ��˫������ģʽʱ�����Զ�ʹ��ѭ��ģʽ��DMA_SxCR �е� CIRC λ��״̬�ǡ���
  //�ء���������ÿ���������ʱ�����洢��ָ�롣
//    DMA_DoubleBufferModeConfig(DMA2_Stream6, (uint32_t)&USART6_DMA_TX_BUF[1][0], DMA_Memory_0);   //first used memory configuration
//    DMA_DoubleBufferModeCmd(DMA2_Stream6, ENABLE);

  nvic.NVIC_IRQChannel = USART3_IRQn;
  nvic.NVIC_IRQChannelPreemptionPriority = 3;   //pre-emption priority
  nvic.NVIC_IRQChannelSubPriority = 3;		    //subpriority
  nvic.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&nvic);
  USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);        //usart rx idle interrupt  enabled
//		USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);        //usart rx idle interrupt  enabled
//		nvic.NVIC_IRQChannel = DMA2_Stream2_IRQn ;
//    nvic.NVIC_IRQChannelPreemptionPriority =3;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
//		DMA_ITConfig(DMA2_Stream2,DMA_IT_TC,ENABLE);


  USART_Cmd(USART3,ENABLE);
}
