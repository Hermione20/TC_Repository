#include "can_bus.h"

static uint32_t can1_count=0;
static uint32_t can2_count=0;

CAN_RxHeaderTypeDef   RxHeader1;
uint8_t               RxData1[8]    = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
CAN_RxHeaderTypeDef   RxHeader2;
uint8_t               RxData2[8]    = {0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};

//初始化（can1，can2发送。）
void USER_can_Init(void)
{
	
	//配置can1过滤器
	CAN_FilterTypeDef Can1fig;
  Can1fig.FilterBank = 0;                         //过滤器组1
  Can1fig.FilterMode = CAN_FILTERMODE_IDMASK;     //工作在标识符屏蔽位模式
  Can1fig.FilterScale = CAN_FILTERSCALE_16BIT;    //过滤器位宽为单个32位。
	  /* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
  Can1fig.FilterIdHigh = 0x0000;
  Can1fig.FilterIdLow = 0x0000;
  Can1fig.FilterMaskIdHigh = 0x0000;              //过滤器高16位每位必须匹配
  Can1fig.FilterMaskIdLow = 0x0000;               //过滤器低16位每位必须匹配
  Can1fig.FilterFIFOAssignment = CAN_RX_FIFO0;    //过滤器被关联到FIFO 0 CAN_filter_FIFO
  Can1fig.FilterActivation = ENABLE;              //使能过滤器
	
  if (HAL_CAN_ConfigFilter(&hcan1, &Can1fig) != HAL_OK) //can1
  {
    /* Filter configuration Error */
    Error_Handler();
  }
	
	//配置can2过滤器
	CAN_FilterTypeDef Can2fig;
  Can2fig.FilterBank = 14;                         //过滤器组2
  Can2fig.FilterMode = CAN_FILTERMODE_IDMASK;     //工作在标识符屏蔽位模式
  Can2fig.FilterScale = CAN_FILTERSCALE_16BIT;    //过滤器位宽为单个32位。
	  /* 使能报文标示符过滤器按照标示符的内容进行比对过滤，扩展ID不是如下的就抛弃掉，是的话，会存入FIFO0。 */
  Can2fig.FilterIdHigh = 0x0000;
  Can2fig.FilterIdLow = 0x0000;
  Can2fig.FilterMaskIdHigh = 0x0000;              //过滤器高16位每位必须匹配
  Can2fig.FilterMaskIdLow = 0x0000;               //过滤器低16位每位必须匹配
  Can2fig.FilterFIFOAssignment = CAN_RX_FIFO1;    //过滤器被关联到FIFO 1 CAN_filter_FIFO
  Can2fig.FilterActivation = ENABLE;              //使能过滤器
  Can2fig.SlaveStartFilterBank = 14;
	if (HAL_CAN_ConfigFilter(&hcan2, &Can2fig) != HAL_OK) //can2
  {
    /* Filter configuration Error */
    Error_Handler();
  }
	

	//开启can模块
  if (HAL_CAN_Start(&hcan1) != HAL_OK) //can1
  {
    /* Start Error */
    Error_Handler();
  }
	if (HAL_CAN_Start(&hcan2) != HAL_OK) //can2
  {
    /* Start Error */
    Error_Handler();
  }

	
	//开启can中断
	if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)  //can1
  {
    /* Notification Error */
    Error_Handler();
  }
	if (HAL_CAN_ActivateNotification(&hcan2, CAN_IT_RX_FIFO1_MSG_PENDING) != HAL_OK)  //can2
  {
    /* Notification Error */
    Error_Handler();
  }
}



//can1发送函数
CAN_TxHeaderTypeDef TxHeader_CAN1;
void USER_CAN1_transmit(uint8_t*TxData,uint8_t TxDatalen,int BOX,int can1_id,int IDE_mode)
{
	

	TxHeader_CAN1.RTR = CAN_RTR_DATA;       //数据帧
	TxHeader_CAN1.IDE = CAN_ID_STD;         //选择标准长度ID和扩展长度ID 
  	TxHeader_CAN1.DLC = TxDatalen;	         //发送几位的数据（1-8）
	TxHeader_CAN1.StdId=can1_id;
	
	if (BOX==0){
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader_CAN1, TxData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
     /* Transmission request Error */
     Error_Handler();
    }
	}
	
	if (BOX==1){
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader_CAN1, TxData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
    {
     /* Transmission request Error */
     Error_Handler();
    }
	}
	
	if (BOX==2){
  if (HAL_CAN_AddTxMessage(&hcan1, &TxHeader_CAN1, TxData, (uint32_t*)CAN_TX_MAILBOX2) != HAL_OK)
    {
     /* Transmission request Error */
     Error_Handler();
    }
	}
}


//can2发送函数
CAN_TxHeaderTypeDef TxHeader_CAN2;
void USER_CAN2_transmit(uint8_t*TxData,uint8_t TxDatalen,int BOX,int can2_id,int IDE_mode)
{

  
	switch (IDE_mode)
  {
  	case 0:
			TxHeader_CAN2.RTR = CAN_RTR_DATA;   //数据帧
  		break;
  	case 1:
			TxHeader_CAN2.RTR = CAN_RTR_REMOTE; //远程帧
  		break;
  	default:
  		break;
  }
	TxHeader_CAN2.IDE = CAN_ID_STD;         //选择标准长度ID和扩展长度ID 
    TxHeader_CAN2.DLC = TxDatalen;	         //发送几位的数据（1-8）
	TxHeader_CAN2.StdId=can2_id;
	
	if (BOX==0){
  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader_CAN2, TxData, (uint32_t*)CAN_TX_MAILBOX0) != HAL_OK)
    {
     /* Transmission request Error */
     Error_Handler();
    }
	}
	
	if (BOX==1){
	while (HAL_CAN_AddTxMessage(&hcan2, &TxHeader_CAN2, TxData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
    {
			
    }	
//  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader_CAN2, TxData, (uint32_t*)CAN_TX_MAILBOX1) != HAL_OK)
//    {
//     /* Transmission request Error */
//     Error_Handler();
//    }
	}
	
	if (BOX==2){
  if (HAL_CAN_AddTxMessage(&hcan2, &TxHeader_CAN2, TxData, (uint32_t*)CAN_TX_MAILBOX2) != HAL_OK)
    {
     /* Transmission request Error */
     Error_Handler();
    }
	}
}

//can1回调函数
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get 1RX message */
  if (HAL_CAN_GetRxMessage(&hcan1,CAN_RX_FIFO0, &RxHeader1, RxData1) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
	CAN1_IdleCallback(); //
}

//can2回调函数
void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *CanHandle)
{
  /* Get 2RX message */
  if (HAL_CAN_GetRxMessage(&hcan2,CAN_RX_FIFO1, &RxHeader2, RxData2) != HAL_OK)
  {
    /* Reception Error */
    Error_Handler();
  }
	CAN2_IdleCallback();
}


void CAN1_IdleCallback()
{
	switch (RxHeader1.StdId)
  {
  	case 0x141: //L电机参数
			EncoderProcess_L();
  		break;
  	case 0x142: //R电机参数
			EncoderProcess_R();
  		break;
		case 0x00: //HT04电机的反馈
      Encoder_HT04_Process_CAN1();
  	default:
  		break;
  }
}



void CAN2_IdleCallback()
{	
	can2_count++;
	switch (RxHeader2.StdId)
  {
		case 0x201: //拨盘电机
			get_M3508_feedback(&M1Encoder,can2_count);
  		break;
//		case 0x202: //弹仓盖电机
//			get_M3508_feedback(&M2Encoder);
//  		break;
//		case 0x203: //电机的反馈
//      		get_M3508_feedback(&M3Encoder);
//  		break;	4
//		case 0x204: //电机的反馈
//      		get_M3508_feedback(&M4Encoder);
//  		break;
//		case 0x205: //电机的反馈
//      		get_M3508_feedback(&M5Encoder);
//  		break;
//		case 0x206: //电机的反馈
//      		get_M3508_feedback(&M6Encoder);
//  		break;
//		case 0x207: //电机的反馈
//      		get_M3508_feedback(&M7Encoder);
//			break;
//		case 0x208: //电机的反馈
//      		get_M3508_feedback(&M8Encoder);
//  		break;
  	default:
  		break;
  }
}

