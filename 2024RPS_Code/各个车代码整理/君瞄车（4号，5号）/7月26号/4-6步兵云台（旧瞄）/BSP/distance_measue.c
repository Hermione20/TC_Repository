
//#include "main.h"

// /**
//  * @brief  ����Ƕ�������жϿ�����NVIC
//  * @param  ��
//  * @retval ��
//  */
//	
//uint8_t measure_recive_buffer[RECEIVEBUFF_SIZE]={0};
//uint8_t measure_recive_count=0;
//measure_frame_t measure_frame_sent=FREAM_DEFAULT;
//measure_frame_t measure_frame_receive=FREAM_DEFAULT;
//measure_single_frame_t measure_single_frame_receive;//���β������ݽ���֡

//uint8_t frame_value[4]={0,0,0,0};   //һ֡���ݵ�value
//measure_mode_e measure_mode=continue_measure;  //����ģʽ״̬

//uint8_t Usart3_rec_len;
///**
//  * @brief  DEBUG_USART GPIO ����,����ģʽ���á�115200 8-N-1 ���жϽ���ģʽ
//  * @param  ��
//  * @retval ��
//  */
//void MEASURE_DISTANCE_USART_Config(void)
//{
//  GPIO_InitTypeDef GPIO_InitStructure;
//  USART_InitTypeDef USART_InitStructure;
//  NVIC_InitTypeDef NVIC_InitStruct;	
//	
//  RCC_AHB1PeriphClockCmd( DEBUG_USART_RX_GPIO_CLK|DEBUG_USART_TX_GPIO_CLK, ENABLE);

//  /* Enable UART clock */
//  RCC_APB1PeriphClockCmd(DEBUG_USART_CLK, ENABLE);
//  
//  /* Connect PXx to USARTx_Tx*/
//  GPIO_PinAFConfig(DEBUG_USART_RX_GPIO_PORT,DEBUG_USART_RX_SOURCE, DEBUG_USART_RX_AF);

//  /* Connect PXx to USARTx_Rx*/
//  GPIO_PinAFConfig(DEBUG_USART_TX_GPIO_PORT,DEBUG_USART_TX_SOURCE,DEBUG_USART_TX_AF);
//	
//  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
//  NVIC_InitStruct.NVIC_IRQChannel = DEBUG_USART_IRQ;
//  NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
//  NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 0;
//  NVIC_InitStruct.NVIC_IRQChannelSubPriority = 3;
//  NVIC_Init(&NVIC_InitStruct);

//  /* Configure USART Tx as alternate function  */
//  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
//  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;

//  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_TX_PIN  ;
//  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//  GPIO_Init(DEBUG_USART_TX_GPIO_PORT, &GPIO_InitStructure);

//  /* Configure USART Rx as alternate function  */
//  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
//  GPIO_InitStructure.GPIO_Pin = DEBUG_USART_RX_PIN;
//  GPIO_Init(DEBUG_USART_RX_GPIO_PORT, &GPIO_InitStructure);
//			
//  /* USART mode config */
//  USART_InitStructure.USART_BaudRate = DEBUG_USART_BAUDRATE;
//  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
//  USART_InitStructure.USART_StopBits = USART_StopBits_1;
//  USART_InitStructure.USART_Parity = USART_Parity_No ;
//  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
//  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
//  USART_Init(DEBUG_USART, &USART_InitStructure); 
//  
//  USART_ITConfig(USART3,USART_IT_IDLE,ENABLE);
//  
//  USART_Cmd(USART3, ENABLE);
//}


//void MEASURE_DISTANCE_USART_DMA_Config(void)
//{
//  DMA_InitTypeDef DMA_InitStructure;
////  NVIC_InitTypeDef  nvic;
//  /*����DMAʱ��*/
//  RCC_AHB1PeriphClockCmd(DEBUG_USART_DMA_CLK, ENABLE);
//  
//  /* ��λ��ʼ��DMA������ */
//  DMA_DeInit(DEBUG_USART_DMA_STREAM);

//  /* ȷ��DMA��������λ��� */
//  while (DMA_GetCmdStatus(DEBUG_USART_DMA_STREAM) != DISABLE)  {
//  }

//	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
//  /*usart1 rx��Ӧdma1��ͨ��4��������1*/	
//  DMA_InitStructure.DMA_Channel = DMA_Channel_4;  
//  /*����DMAԴ���������ݼĴ�����ַ*/
//  DMA_InitStructure.DMA_PeripheralBaseAddr =(uint32_t) (&USART3->DR);	 
//  /*�ڴ��ַ(Ҫ����ı�����ָ��)*/
//  DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)measure_recive_buffer;
//  /*���򣺴����赽�ڴ�*/		
//  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;	
//  /*�����СDMA_BufferSize=RECEIVEBUFF_SIZE*/	
//  DMA_InitStructure.DMA_BufferSize = RECEIVEBUFF_SIZE;
//  /*�����ַ����*/	    
//  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable; 
//  /*�ڴ��ַ����*/
//  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;	
//  /*�������ݵ�λ*/	
//  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
//  /*�ڴ����ݵ�λ 8bit*/
//  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;	
//  /*DMAģʽ������ѭ��*/
//  DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;	 
//  /*���ȼ�����*/	
//  DMA_InitStructure.DMA_Priority = DMA_Priority_Medium;      
//  /*����FIFO*/
//  DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;        
//  DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;    
//  /*�洢��ͻ������ 16������*/
//  DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;    
//  /*����ͻ������ 1������*/
//  DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;    
//  /*����DMA2��������2*/		   
//  DMA_Init(DMA1_Stream1, &DMA_InitStructure);
//  
////  nvic.NVIC_IRQChannel = DMA1_Stream1_IRQn;   // ����DMAͨ�����ж�����
////  nvic.NVIC_IRQChannelPreemptionPriority = 2;     // ���ȼ�����
////  nvic.NVIC_IRQChannelSubPriority = 1;
////  nvic.NVIC_IRQChannelCmd = ENABLE;
////  NVIC_Init(&nvic);
////	DMA_ITConfig(DMA1_Stream1,DMA_IT_TC,ENABLE);
//	
//  /*ʹ��DMA*/
//  DMA_Cmd(DMA1_Stream1, ENABLE);
//  
//  /* �ȴ�DMA��������Ч*/
//  while(DMA_GetCmdStatus(DEBUG_USART_DMA_STREAM) != ENABLE)
//  {
//  }   
//}


///*****************  ����һ���ַ� **********************/
//void Usart_SendByte( USART_TypeDef * pUSARTx, uint8_t ch)
//{
//	/* ����һ���ֽ����ݵ�USART */
//	USART_SendData(pUSARTx,ch);
//	/* �ȴ��������ݼĴ���Ϊ�� */
//	while (USART_GetFlagStatus(pUSARTx, USART_FLAG_TXE) == RESET);	
//}



////��������֡
//void Frame_Make(KEY key,uint8_t* value,uint8_t value_len)
//{ 
//	int i=0;
//	measure_frame_sent.key=key;
//	for(i=0;i<value_len;i++)
//	{
//	   measure_frame_sent.value[i]=value[i];
//	}
//	measure_frame_sent.CRC8=crc_high_first(&measure_frame_sent.key,total_crc_byte);
//}


////����֡
//void Frame_Send()
//{
//	uint8_t *prt_to_frame=(uint8_t*)(&measure_frame_sent);
//	int index;
//	for(index=0;index<sizeof(measure_frame_sent);index++)
//	{
//	Usart_SendByte(DEBUG_USART,*(prt_to_frame));
//		prt_to_frame++;
//	}
//}


//////���պ���
////void DEBUG_USART_IRQHandler(void)
////{
////	if(USART_GetITStatus(DEBUG_USART,USART_IT_RXNE)!=RESET)
////	{		
////		measure_recive_buffer[measure_recive_count] = USART_ReceiveData(DEBUG_USART);
////		measure_recive_count++;
////	}	
////	if(measure_recive_buffer[0]!=frame_head)
////	{
////			measure_recive_count=0;
////	}
////	if(measure_recive_count>(sizeof(measure_recive_buffer)-1))
////	{
////		measure_frame_receive=*(measure_frame_t*)(measure_recive_buffer);
////		measure_recive_count=0;
////	}

////}

//void DMA1_Stream1_IRQHandler(void)  //�������ж�
//{
//	DMA_Cmd(DMA1_Stream4, DISABLE); 
//	DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);                //�������־λ������ֻ����һ��   
//	
////  if(DMA_GetFlagStatus(DMA1_Stream1,DMA_FLAG_TCIF1)!=RESET)          //����ж��Ƿ���
////	{	
////		DMA_Cmd(DEBUG_USART_DMA_STREAM,DISABLE);                   //�ر�DMA����
////		DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);                //�������־λ������ֻ����һ��   
////    DMA_SetCurrDataCounter(DMA1_Stream1,RECEIVEBUFF_SIZE);    //�������ô������������
////	  DMA_Cmd(DMA1_Stream1,ENABLE);    		
////		
////	  Usart3_rec_len = RECEIVEBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);
////		measure_frame_receive=*(measure_frame_t*)(measure_recive_buffer);                        

////	}
//}


///************************��Hi220��ͻ,����ʱȡ��ע��*****************************************************/
////void DEBUG_USART_IRQHandler(void)
////{
////	if(USART_GetITStatus(DEBUG_USART,USART_IT_IDLE) != RESET)          //����ж��Ƿ���
////	{	
////		(void)USART3->SR;
////		(void)USART3->DR;
////			
////		DMA_Cmd(DEBUG_USART_DMA_STREAM,DISABLE);                //�ر�DMA����
////	  DMA_ClearFlag(DMA1_Stream1,DMA_FLAG_TCIF1);             //���DMA1_Steam1������ɱ�־,����dma����֮ǰ����ֹ���ݴ�λ
////		                                                        //�������־λ������ֻ����һ�� 
////	  Usart3_rec_len = RECEIVEBUFF_SIZE - DMA_GetCurrDataCounter(DMA1_Stream1);
////		measure_frame_receive=*(measure_frame_t*)(measure_recive_buffer);
////	  DMA_SetCurrDataCounter(DMA1_Stream1,RECEIVEBUFF_SIZE);  //�������ô������������
////		DMA_Cmd(DMA1_Stream1,ENABLE);                           //����DMA����
////	}
////}
///***************************************************************************************/




//uint8_t check_the_recive(void)
//{
//if((measure_frame_receive.head==frame_head)&&(measure_frame_receive.tail==frame_tail))
//	return 1;
//else
//	{
//	measure_recive_count=0;
//		return 0;
//	}
//}

//uint32_t get_the_distance()
//{

//		return from_frame_to_the_distance_mm(&measure_frame_receive);
//}

////����֡�õ�����
//uint32_t from_frame_to_the_distance_mm(void *frame)
//{
//	uint32_t distance=0;
//  distance+=((measure_frame_t*)(frame))->value[3];
//	distance+=((measure_frame_t*)(frame))->value[2]<<8;
//  distance+=((measure_frame_t*)(frame))->value[1]<<16;
//	return distance;
//}

////���β�������֡����
//uint8_t check_the_single_recive(void)
//{
//if((measure_frame_receive.head==frame_head)&&(measure_frame_receive.tail==single_frame_tail))
//	return 1;
//else
//	{
//	measure_recive_count=0;
//		return 0;
//	}
//}

//void stop_measure()
//{
//	frame_value[0]=0; 
//	frame_value[1]=0; 
//	frame_value[2]=0; 
//	frame_value[3]=0; 
//	Frame_Make(STOP_MEASURE,frame_value,sizeof(frame_value));
//	Frame_Send();
//}

//void set_continue_measure_mode()
//{
//	frame_value[0]=0; 
//	frame_value[1]=0; 
//	frame_value[2]=0; 
//	frame_value[3]=0; //���β���1����������0
//	Frame_Make(SET_MEASURE_MODE,frame_value,sizeof(frame_value));
//	Frame_Send();
//}

//void set_single_measure_mode()
//{
//	frame_value[0]=0; 
//	frame_value[1]=0; 
//	frame_value[2]=0; 
//	frame_value[3]=1; //���β���1����������0
//	Frame_Make(SET_MEASURE_MODE,frame_value,sizeof(frame_value));
//	Frame_Send();
//}

//void start_measure()
//{
//	frame_value[0]=0; 
//	frame_value[1]=0; 
//	frame_value[2]=0; 
//	frame_value[3]=0; 
//  Frame_Make(START_MEASURE,frame_value,sizeof(frame_value));
//  Frame_Send();
//}


////����crc
//uint8_t crc_high_first(uint8_t *ptr,uint8_t len)
//{
//uint8_t i;
//uint8_t crc=0x00;

//while(len--)
//{
//	crc^=*ptr++;
//	for(i=8;i>0;--i)
//	{
//		if(crc&0x80)
//			crc=(crc<<1)^0x31;
//		else
//			crc=(crc<<1);
//	}
//}
//return crc;
//}

//uint32_t average_filter( float new_value)
//{
//    static float value_buf[AVERAGE_FILTER_N];
//    uint32_t sum  = 0;
//    uint8_t count = 0, i = 0;
//    for ( count = 0; count < AVERAGE_FILTER_N - 1; count++)
//    {
//        value_buf[count] = value_buf[count + 1] ;
//				sum += value_buf[count];
//    }
//    value_buf[AVERAGE_FILTER_N - 1] = new_value;
//		sum += value_buf[AVERAGE_FILTER_N - 1];
//		return sum / AVERAGE_FILTER_N;
//}


//void Distance_handle(void)    //��������ȡ������Ϣ��λΪmm������ʱ��λΪcm
//{
//	//1����ȡ������Ϣ
////  uint32_t distance ;
//	distance = (get_the_distance() - 80)/10.0 ;
//	if( distance  <  MAX_MEASURE)            //����żȻ����
//	{
//		 distance_new = distance;               //������40m�������ݣ�ѡ��ʹ���ϴεľ�����Ϣ
//		 distance_last = distance;
//	}
//	else                                     
//	{
//		 distance_new = distance_last ;	
//	}
//	if(distance_new > MAX_ATTACK_DISTANCE)
//	{
//		 distance_new = MAX_ATTACK_DISTANCE;
//	}
//	
//	Measure_filter_distance = average_filter(distance_new);     //�������ݾ�ֵ�˲�
//	
//	#if DISTANCE_ENABLE                       
//	 if(Measure_filter_distance == 0 && new_location.dis !=0)
//			Gimbal_Auto_Shoot.Distance = new_location.dis ;
//	 else if(Measure_filter_distance != 0)
//			Gimbal_Auto_Shoot.Distance = Measure_filter_distance ;
//	 else
//			Gimbal_Auto_Shoot.Distance = 150 ;
//  #else                                    
//	    Gimbal_Auto_Shoot.Distance = 150 ;
//  #endif
//	 
//	//2�����������Ϣ�����ݾ�����Ϣ���в���(����������)
//	if(Gimbal_Auto_Shoot.Distance < MAX_ATTACK_DISTANCE) 
//	{
//		 now_distance = Gimbal_Auto_Shoot.Distance;
//		 last_distance = Gimbal_Auto_Shoot.Distance;
//	}
//	else                                     
//	{
//		 now_distance = last_distance ;	
//	}
//	if(now_distance > MAX_ATTACK_DISTANCE)
//	{
//		 now_distance = MAX_ATTACK_DISTANCE;
//	}
//}



/*********************************************END OF FILE**********************/
