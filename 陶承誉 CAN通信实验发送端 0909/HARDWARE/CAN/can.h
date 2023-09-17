#ifndef __CAN_H
#define __CAN_H	 
//#include "sys.h"	 
#include "stm32f4xx.h"
//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK STM32F407������
//CAN���� ����	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2014/5/7
//�汾��V1.0 
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	 

	
//����RX0�ж�ʹ��



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













