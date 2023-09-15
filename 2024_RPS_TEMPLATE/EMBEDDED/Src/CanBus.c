/* Includes ------------------------------------------------------------------*/
#include "CanBus.h"
/*----------------------------------------------------------------------------*/

/**
************************************************************************************************************************
* @Name     : CAN1_Mode_Init
* @brief    : This function initializes the CAN1 struct, the struct of CAN_Filter, and the NVIC configuration
* @param    : tbs2    tbs2:ʱ���2��ʱ�䵥Ԫ.  Range from:CAN_BS2_1tq to CAN_BS2_8tq;
*	@param		:	tbs1		tbs1:ʱ���1��ʱ�䵥Ԫ.  Range from:CAN_BS1_1tq to CAN_BS1_16tq;
*	@param		:	brp			brp :�����ʷ�Ƶ��.       Range from:1 to 1024;                   tq=(brp)*tpclk1
* @param		: mode 		mode:CAN_Mode_Normal,��ͨģʽ;CAN_Mode_LoopBack,�ػ�ģʽ;
* @retval   : 0,��ʼ��OK;����,��ʼ��ʧ��; 
* @Note     : ������=Fpclk1/((tbs1+1+tbs2+1+1)*brp);
*							Fpclk1��ʱ���ڳ�ʼ����ʱ������Ϊ42M,�������CAN1_Mode_Init(CAN_BS2_6tq,CAN_BS1_7tq,6,CAN_Mode_LoopBack);
*							������Ϊ:42M/((6+7+1)*6)=500Kbps
************************************************************************************************************************
**/
void send_can_remote(CAN_TypeDef *CANx, int16_t romate_speed,int16_t romate_angle,int16_t get_speedw,int16_t start_angle)
{
		CanTxMsg tx_message;
    
    tx_message.StdId = 0x409;//send to gyro controll board
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    
    tx_message.Data[0] = (unsigned char)(romate_speed >> 8);
    tx_message.Data[1] = (unsigned char)romate_speed;;
    tx_message.Data[2] = (unsigned char)(romate_angle >> 8);
    tx_message.Data[3] = (unsigned char)romate_angle;;
    tx_message.Data[4] = (unsigned char)(get_speedw >> 8);
    tx_message.Data[5] = (unsigned char)get_speedw;
    tx_message.Data[6] = (unsigned char)(start_angle >> 8);
    tx_message.Data[7] = (unsigned char)start_angle;
	
    
    CAN_Transmit(CANx,&tx_message);

}