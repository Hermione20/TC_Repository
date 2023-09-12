#ifndef __CAN_H
#define __CAN_H	 
#include "public.h"


	
//����RX0�ж�ʹ��
#define CAN1_RX0_INT_ENABLE	1		//0,��ʹ��;1,ʹ��.								    
#define CAN2_RX0_INT_ENABLE 1

#define CAN1_TX0_INT_ENABLE 0
#define CAN2_TX0_INT_ENABLE 0


#define CAN1_Data_Receive_Process  do{CAN1_Receive_Msg(rx_message.Data);}while(0);
#define CAN2_Data_Receive_Process  do{CAN2_Receive_Msg(rx_message.Data);}while(0);	





u8 CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ�� 
u8 CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN��ʼ�� 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������
u8 CAN1_Receive_Msg(u8 *buf);							//��������
u8 CAN2_Receive_Msg(u8 *buf);

void CAN1_Configuration(void);
void set_M3508_info(void);
void CAN2_Configuration(void);
void Set_Gimbal_Current1(CAN_TypeDef *CANx, int16_t ch_1_iq, int16_t ch_2_iq, int16_t ch_3_iq, int16_t ch_4_iq);
#endif

















