#ifndef __CANBUS_H
#define __CANBUS_H
#include "public.h"


/**********************���ֵ��̵��id||���ֵ��̵��id**************************************/
#define GM1Encoder_MOTOR 0x00
#define GM2Encoder_MOTOR 0X00
#define GM3Encoder_MOTOR 0X00
#define GM4Encoder_MOTOR 0X00

#define CM1Encoder_MOTOR 0x201
#define CM2Encoder_MOTOR 0x202
#define CM3Encoder_MOTOR 0x203
#define CM4Encoder_MOTOR 0x204
/*************************��̨���id******************************/
#define GIMBAL_YAW_MOTOR 0x142
#define GIMBAL_PITCH_MOTOR 0x141
/****************************Ӣ��С��̨���id***********************************/
#define SMALL_GIMBAL_MOTOR 0x209
#define SCOPE_MOTOR 0x208
/*********************************Ħ���ֵ��id**************************************/
#define LEFT_FRICTION 0X202
#define RIGHT_FRICTION 0X201
//�ڱ�
#define LEFT_UP_FRICTION 0X00
#define RIGHT_UP_FRICTION 0X00
#define LEFT_DOWN_FRIICTION 0X00
#define RIGHT_DOWN_FRICTION 0X00
/**********************************���̵��id**************************************/
#define DOWN_POKE 0x205    
#define UP_POKE 0X205
#define LEFT_POKE 0X00			//��һ�����ֻ��һ����һ
#define RIGHT_POKE 0X00
#define POKE 0X00
/*********************************�������°�ͨ��id*********************************/
#define UP_CAN2_TO_DOWN_CAN1_1 0X407
#define UP_CAN2_TO_DOWN_CAN1_2 0X408
#define UP_CAN2_TO_DOWN_CAN1_3 0X409



void Can1ReceiveMsgProcess(CanRxMsg * msg);
void Can2ReceiveMsgProcess(CanRxMsg * msg);




										
void can_bus_send_task(void);

#endif