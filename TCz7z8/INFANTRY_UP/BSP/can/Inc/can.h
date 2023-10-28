#ifndef __CAN_H
#define __CAN_H	 
#include "public.h"	    


//����RX0�ж�ʹ��
#define EN_CAN1	1		//0,��ʹ��;1,ʹ��.								    
#define EN_CAN2 1


void CAN1_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN1��ʼ��

void CAN2_Mode_Init(u8 tbs2,u8 tbs1,u16 brp,u8 mode);//CAN2��ʼ�� 
 
#define CAN1_Data_Receive_Process do{Can1ReceiveMsgProcess(&rx_message);}while(0);
#define CAN2_Data_Receive_Process do{Can2ReceiveMsgProcess(&rx_message);}while(0);
	 
u8 CAN1_Send_Msg(u8* msg,u8 len);						//��������
u8 CAN1_Receive_Msg(u8 *buf);							  //��������
u8 CAN2_Receive_Msg(u8 *buf);

#endif

















