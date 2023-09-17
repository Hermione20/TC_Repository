/* Includes ------------------------------------------------------------------*/
#include "can.h"
#include "led.h"
#include "delay.h"
#include "usart.h"
#include <stdio.h>

u8 CAN1_receive_buf[20];
u8 CAN2_receive_buf[20];

u8 CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{

		GPIO_InitTypeDef       gpio;
	  CAN_InitTypeDef        can;
  	CAN_FilterInitTypeDef  can_filter;
	
   	NVIC_InitTypeDef       nvic;

    //使能相关时钟
	  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能PORTA时钟	            
  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);//使能CAN1时钟	
	
	  //引脚复用映射配置
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource11,GPIO_AF_CAN1); //GPIOA11复用为CAN1
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource12,GPIO_AF_CAN1); //GPIOA12复用为CAN1
	  
		    //初始化GPIO
	  gpio.GPIO_Pin = GPIO_Pin_11| GPIO_Pin_12;
    gpio.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_Init(GPIOA, &gpio);//初始化PA11,PA12
	
		nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
  	nvic.NVIC_IRQChannelPreemptionPriority = 3;     // 主优先级为1
  	nvic.NVIC_IRQChannelSubPriority = 1;            // 次优先级为0
  	nvic.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&nvic);	
	
		CAN_DeInit(CAN1);
    CAN_StructInit(&can);
		
  	//CAN单元设置
   	can.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	can.CAN_ABOM=ENABLE;	//软件自动离线管理	  
  	can.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	can.CAN_NART=DISABLE;	//禁止报文自动传送 
  	can.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	can.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	can.CAN_Mode= mode;	 //模式设置 
  	can.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	can.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~ CAN_BS1_16tq
  	can.CAN_BS2=tbs2;// Tbs2范围CAN_BS2_1tq ~ CAN_BS2_8tq
  	can.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN1, &can);   // 初始化CAN1 
    
		
		//配置过滤器
 	  can_filter.CAN_FilterNumber=0;	  //过滤器0
  	can_filter.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	can_filter.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	can_filter.CAN_FilterIdHigh=0x0000;////32位ID
  	can_filter.CAN_FilterIdLow=0x0000;
  	can_filter.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	can_filter.CAN_FilterMaskIdLow=0x0000;
   	can_filter.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	can_filter.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&can_filter);//滤波器初始化
		nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
    nvic.NVIC_IRQChannelPreemptionPriority = 3;
    nvic.NVIC_IRQChannelSubPriority = 1;
    nvic.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&nvic);

		CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
	return 0;
}   

//		CAN_InitTypeDef        can;
//    CAN_FilterInitTypeDef  can_filter;
//    GPIO_InitTypeDef       gpio;
//    NVIC_InitTypeDef       nvic;

//    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
//    RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN1, ENABLE);

//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource11, GPIO_AF_CAN1);
//    GPIO_PinAFConfig(GPIOA, GPIO_PinSource12, GPIO_AF_CAN1);

//    gpio.GPIO_Pin = GPIO_Pin_11 | GPIO_Pin_12;
//    gpio.GPIO_Mode = GPIO_Mode_AF;
//    GPIO_Init(GPIOA, &gpio);
//    
//    nvic.NVIC_IRQChannel = CAN1_RX0_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 3;
//    nvic.NVIC_IRQChannelSubPriority = 1;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);
//    
//    nvic.NVIC_IRQChannel = CAN1_TX_IRQn;
//    nvic.NVIC_IRQChannelPreemptionPriority = 1;
//    nvic.NVIC_IRQChannelSubPriority = 0;
//    nvic.NVIC_IRQChannelCmd = ENABLE;
//    NVIC_Init(&nvic);    
//    
//    CAN_DeInit(CAN1);
//    CAN_StructInit(&can);
//    
//    can.CAN_TTCM = DISABLE;
//    can.CAN_ABOM = ENABLE;
//    can.CAN_AWUM = DISABLE;
//    can.CAN_NART = DISABLE;
//    can.CAN_RFLM = DISABLE;
//    can.CAN_TXFP = DISABLE;
//    can.CAN_Mode = CAN_Mode_Normal;
//    can.CAN_SJW  = CAN_SJW_1tq;
//    can.CAN_BS1 = CAN_BS1_9tq;
//    can.CAN_BS2 = CAN_BS2_4tq;
//    can.CAN_Prescaler = 3;   //CAN BaudRate 42/(1+9+4)/3=1Mbps
//    CAN_Init(CAN1, &can);

//		can_filter.CAN_FilterNumber=0;
//		can_filter.CAN_FilterMode=CAN_FilterMode_IdMask;
//		can_filter.CAN_FilterScale=CAN_FilterScale_32bit;
//		can_filter.CAN_FilterIdHigh=0x0000;
//		can_filter.CAN_FilterIdLow=0x0000;
//		can_filter.CAN_FilterMaskIdHigh=0x0000;
//		can_filter.CAN_FilterMaskIdLow=0x0000;
//		can_filter.CAN_FilterFIFOAssignment=0;//the message which pass the filter save in fifo0
//		can_filter.CAN_FilterActivation=ENABLE;
//		CAN_FilterInit(&can_filter);
//    CAN_ITConfig(CAN1,CAN_IT_FMP0,ENABLE);
////    CAN_ITConfig(CAN1,CAN_IT_TME,ENABLE); 
//		return 0;
//}   

 
u8 CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode)
{
		GPIO_InitTypeDef       GPIO_InitStructure; 
		CAN_InitTypeDef        CAN_InitStructure;
  	CAN_FilterInitTypeDef  CAN_FilterInitStructure;
   	NVIC_InitTypeDef       NVIC_InitStructure;

    //使能相关时钟
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能PORTA时钟	            

  	RCC_APB1PeriphClockCmd(RCC_APB1Periph_CAN2, ENABLE);//使能CAN1时钟	
	
    //初始化GPIO
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12| GPIO_Pin_13;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
    GPIO_Init(GPIOB, &GPIO_InitStructure);//初始化PA11,PA12
	
	  //引脚复用映射配置
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource12,GPIO_AF_CAN2); //GPIOA11复用为CAN1
	GPIO_PinAFConfig(GPIOB,GPIO_PinSource13,GPIO_AF_CAN2); //GPIOA12复用为CAN1
	  
		CAN_DeInit(CAN2);
    CAN_StructInit(&CAN_InitStructure);
		
  	//CAN单元设置
   	CAN_InitStructure.CAN_TTCM=DISABLE;	//非时间触发通信模式   
  	CAN_InitStructure.CAN_ABOM=ENABLE ;	//软件自动离线管理	  
  	CAN_InitStructure.CAN_AWUM=DISABLE;//睡眠模式通过软件唤醒(清除CAN->MCR的SLEEP位)
  	CAN_InitStructure.CAN_NART=DISABLE ;	//禁止报文自动传送 
  	CAN_InitStructure.CAN_RFLM=DISABLE;	//报文不锁定,新的覆盖旧的  
  	CAN_InitStructure.CAN_TXFP=DISABLE;	//优先级由报文标识符决定 
  	CAN_InitStructure.CAN_Mode= mode;	 //模式设置 
  	CAN_InitStructure.CAN_SJW=CAN_SJW_1tq;	//重新同步跳跃宽度(Tsjw)为tsjw+1个时间单位 CAN_SJW_1tq~CAN_SJW_4tq
  	CAN_InitStructure.CAN_BS1=tbs1; //Tbs1范围CAN_BS1_1tq ~CAN_BS1_16tq
  	CAN_InitStructure.CAN_BS2=tbs2;//Tbs2范围CAN_BS2_1tq ~	CAN_BS2_8tq
  	CAN_InitStructure.CAN_Prescaler=brp;  //分频系数(Fdiv)为brp+1	
  	CAN_Init(CAN2, &CAN_InitStructure);   // 初始化CAN1 
    
		//配置过滤器
		CAN_FilterInitStructure.CAN_FilterNumber=14;	  //过滤器0
  	CAN_FilterInitStructure.CAN_FilterMode=CAN_FilterMode_IdMask; 
  	CAN_FilterInitStructure.CAN_FilterScale=CAN_FilterScale_32bit; //32位 
  	CAN_FilterInitStructure.CAN_FilterIdHigh=0x0000;////32位ID
  	CAN_FilterInitStructure.CAN_FilterIdLow=0x0000;
  	CAN_FilterInitStructure.CAN_FilterMaskIdHigh=0x0000;//32位MASK
  	CAN_FilterInitStructure.CAN_FilterMaskIdLow=0x0000;
   	CAN_FilterInitStructure.CAN_FilterFIFOAssignment=CAN_Filter_FIFO0;//过滤器0关联到FIFO0
  	CAN_FilterInitStructure.CAN_FilterActivation=ENABLE; //激活过滤器0
  	CAN_FilterInit(&CAN_FilterInitStructure);//滤波器初始化


	    
  
  	NVIC_InitStructure.NVIC_IRQChannel = CAN2_RX0_IRQn;
  	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;     // 主优先级为1
  	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;            // 次优先级为0
  	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  	NVIC_Init(&NVIC_InitStructure);


		NVIC_InitStructure.NVIC_IRQChannel = CAN2_TX_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
	  
		CAN_ITConfig(CAN2,CAN_IT_FMP0,ENABLE);//FIFO0消息挂号中断允许.
		return 0;		
}

	void CAN1_TX_IRQHandler(void) //CAN TX
	{
	if (CAN_GetITStatus(CAN1,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
	{
		 CAN_ClearITPendingBit(CAN1,CAN_IT_TME);   
	}
	}

void CAN2_TX_IRQHandler(void) //CAN TX
{
  if (CAN_GetITStatus(CAN2,CAN_IT_TME)!= RESET)    //if transmit mailbox is empty 
  {
	   CAN_ClearITPendingBit(CAN2,CAN_IT_TME);   
  }
}


void CAN2_RX0_IRQHandler(void)
{
		u8 i;
    CanRxMsg rx_message;
    if (CAN_GetITStatus(CAN2,CAN_IT_FMP0)!= RESET) 
    {
       CAN_ClearITPendingBit(CAN2, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
       CAN_Receive(CAN2, CAN_FIFO0, &rx_message);  
       //电机编码器数据处理
       CAN2_Receive_Msg(rx_message.Data);
			 for(i=0;i<8;i++)
				CAN2_receive_buf[i]=rx_message.Data[i];	
    }
}


//中断服务函数	
void CAN1_RX0_IRQHandler(void)
{	 	 
 		u8 i;
    CanRxMsg rx_message;		 
    if (CAN_GetITStatus(CAN1,CAN_IT_FMP0)!= RESET) 
    {
//       CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);//|CAN_IT_FF0|CAN_IT_FOV0
//       CAN_Receive(CAN1, CAN_FIFO0, &rx_message);  
//       //电机编码器数据处理
//				for(i=0;i<8;i++)
//				CAN1_receive_buf[i]=rx_message.Data[i];
    }

}



/**
************************************************************************************************************************
* @Name     : CAN1_Send_Msg
* @brief    : can发送一组数据(固定格式:ID为0X12,标准帧,数据帧)	
* @param    : len     len:数据长度(最大为8)				  
* @retval   : 0,成功; 其他,失败;
* @Note     : none
************************************************************************************************************************
**/
u8 CAN1_Send_Msg(u8* msg,u8 len)
{	
  u8 mbox;
  u16 i=0;
  CanTxMsg TxMessage;
  TxMessage.StdId=0x251;	 // 标准标识符为0
  TxMessage.ExtId=0x12;	 // 设置扩展标示符（29位）
  TxMessage.IDE=0;		  // 使用扩展标识符
  TxMessage.RTR=0;		  // 消息类型为数据帧，一帧8位
  TxMessage.DLC=len;							 // 发送两帧信息
  for(i=0;i<len;i++)
  TxMessage.Data[i]=msg[i];				 // 第一帧信息          
  mbox= CAN_Transmit(CAN1, &TxMessage);   
  i=0;
  while((CAN_TransmitStatus(CAN1, mbox)==CAN_TxStatus_Failed)&&(i<0XFFF))i++;	//等待发送结束
  if(i>=0XFFF)return 1;
  return 0;		
}


/**
************************************************************************************************************************
* @Name     : Can1_Receive_Msg
* @brief    : This function process the can message representing the encoder data received from the CAN2 bus.
* @param    : buf 数据缓存区;	 
* @retval   : 0,无数据被收到;其他,接收的数据长度;
* @Note     : can口接收数据查询
************************************************************************************************************************
**/
u8 CAN1_Receive_Msg(u8 *buf)
{		
		
 	u32 i;
	CanRxMsg rm1;

    if( CAN_MessagePending(CAN1,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN1, CAN_FIFO0, &rm1);//读取数据	
	
		for(i=0;i<rm1.DLC;i++)
    buf[i]=rm1.Data[i]; 
	
	return rm1.Data[0];	
}
/**
************************************************************************************************************************
* @Name     : Can2_Receive_Msg
* @brief    : This function process the can message representing the encoder data received from the CAN2 bus.
* @param    : buf 数据缓存区;	 
* @retval   : 0,无数据被收到;其他,接收的数据长度;
* @Note     : can口接收数据查询
************************************************************************************************************************
**/
u8 CAN2_Receive_Msg(u8 *buf)
{		   		   
 	u32 i;
	CanRxMsg rm;
	rm.StdId=0x205;
	rm.RTR=0;
	rm.IDE=0;
    if(CAN_MessagePending(CAN2,CAN_FIFO0)==0)return 0;		//没有接收到数据,直接退出 
    CAN_Receive(CAN2, CAN_FIFO0, &rm);//读取数据	
    for(i=0;i<rm.DLC;i++)
    buf[i]=rm.Data[i];  
	return rm.Data[0];	
}

/**
************************************************************************************************************************
* @Name     : set_M3508_info
* @brief    : This function sends encoder information for the M3508 via CAN1  
*							can发送一组数据(固定格式:ID为0X200,标准帧,数据帧)	
* @param    : velocity     the velocity of the M3508 motor.
* @param    : temperature  the temperature of the M3508 motor.
* @retval   : void
* @Note     : none
************************************************************************************************************************
**/
void set_M3508_info()
{
		CanTxMsg tm;					  //CAN的发送报文结构体
		tm.StdId=0x250;	
		tm.ExtId=0;
		tm.IDE=0;					
		tm.RTR=0;
		tm.DLC=8;
		tm.Data[0]=CAN2_receive_buf[2];	  //高八位转移低八位
		tm.Data[1]=CAN2_receive_buf[3];	  //十六位数据赋值给八位数据，保留低八位
		tm.Data[2]=CAN2_receive_buf[6];
		tm.Data[3]=0;
		tm.Data[4]=0;
		tm.Data[5]=0;
		tm.Data[6]=0;
		tm.Data[7]=0;
//		tm.Data[0]=(uint8_t)(velocity>>8);//高八位转移低八位
//		tm.Data[1]=(uint8_t)(velocity);	  //十六位数据赋值给八位数据，保留低八位
//		tm.Data[2]=(uint8_t)(temperature>>8);
//		tm.Data[3]=(uint8_t)(temperature);
//		tm.Data[4]=0;
//		tm.Data[5]=0;
//		tm.Data[6]=0;
//		tm.Data[7]=0;
		CAN_Transmit(CAN1 ,&tm);
}



void Set_Gimbal_Current1(CAN_TypeDef *CANx, int16_t ch_1_iq, int16_t ch_2_iq, int16_t ch_3_iq, int16_t ch_4_iq)
{
    CanTxMsg tx_message;    
    tx_message.StdId = 0x1FF;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;

    tx_message.Data[0] = (unsigned char)(ch_1_iq >> 8);
    tx_message.Data[1] = (unsigned char)ch_1_iq;
    tx_message.Data[2] = (unsigned char)(ch_2_iq >> 8);
    tx_message.Data[3] = (unsigned char)ch_2_iq;
    tx_message.Data[4] = (unsigned char)(ch_3_iq >> 8);
    tx_message.Data[5] = (unsigned char)ch_3_iq;
    tx_message.Data[6] = (unsigned char)(ch_4_iq >> 8);
    tx_message.Data[7] = (unsigned char)ch_4_iq;
	
    CAN_Transmit(CANx,&tx_message);
}








