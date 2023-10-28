#ifndef __CANBUS_H
#define __CANBUS_H
#include "main.h"


/**********************���ֵ��̵��id||���ֵ��̵��id**************************************/
#define GM1Encoder_MOTOR 0x00
#define GM2Encoder_MOTOR 0X00
#define GM3Encoder_MOTOR 0X00
#define GM4Encoder_MOTOR 0X00

#define CM1Encoder_MOTOR 0x00
#define CM2Encoder_MOTOR 0x00
#define CM3Encoder_MOTOR 0x00
#define CM4Encoder_MOTOR 0x00
/*************************��̨���id******************************/
#define GIMBAL_YAW_MOTOR 0x205
#define GIMBAL_PITCH_MOTOR 0X206
/****************************Ӣ��С��̨���id***********************************/
#define SMALL_GIMBAL_MOTOR 0X00
#define SCOPE_MOTOR 0X00
/*********************************Ħ���ֵ��id**************************************/
#define LEFT_FRICTION1 0X201
#define RIGHT_FRICTION1 0X202
#define LEFT_FRIICTION2 0X00
#define RIGHT_FRICTION2 0X00
/*********************************�������°�ͨ��id*********************************/
#define UP_CAN2_TO_DOWN_CAN1_1 0X407
#define UP_CAN2_TO_DOWN_CAN1_2 0X408
#define UP_CAN2_TO_DOWN_CAN1_3 0X409



void Can1ReceiveMsgProcess(CanRxMsg * msg);
void Can2ReceiveMsgProcess(CanRxMsg * msg);




										
void can_bus_send_task(void);

#endif
