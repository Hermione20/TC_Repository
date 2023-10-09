#include "PM01.h"


/**
  ******************************************************************************
  * @file    LK_TECH.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   ���ļ���дLK_TECH���ͺŵ���Ľ���,��ڲ�������ͨ��
						 �������ṹ�壬can���ߵļ�����can�ṹ�壬��������ʼֵ
						 �趨.������������senior�ļ�
						 
@verbatim
 ===============================================================================
 **/
 
/******************************capacitance_define*************************************/
volatile capacitance_message_t capacitance_message;

void PM01_message_Process(volatile capacitance_message_t *v,CAN_RxHeaderTypeDef * msg)
{
	uint8_t PM01_message_Data[8];
	switch (msg->StdId)
	{
	case 0x610:
	{
		v->mode = (PM01_message_Data[0] << 8) | PM01_message_Data[1];
		v->mode_sure = (PM01_message_Data[2] << 8) | PM01_message_Data[3];
	}
	break;
	case 0x611:
	{
		v->in_power = (PM01_message_Data[0] << 8) | PM01_message_Data[1];
		v->in_v = (PM01_message_Data[2] << 8) | PM01_message_Data[3];
		v->in_i = (PM01_message_Data[4] << 8) | PM01_message_Data[5];
	}
	break;
	case 0x612:
	{
		v->out_power = (PM01_message_Data[0] << 8) | PM01_message_Data[1];
		v->out_v = (PM01_message_Data[2] << 8) | PM01_message_Data[3];
		v->out_i = (PM01_message_Data[4] << 8) | PM01_message_Data[5];
	}
	break;
	case 0x613:
	{
		v->tempureture=(PM01_message_Data[0]<<8)|PM01_message_Data[1];
		v->time=(PM01_message_Data[2]<<8)|PM01_message_Data[3];
    v->this_time=(PM01_message_Data[4]<<8)|PM01_message_Data[5];
	}break;

	default:
		break;
	}
}

/**********************��������**************************/
void POWER_Control1(CAN_HandleTypeDef *hcanx ,uint16_t Power,uint16_t StdId) //���ò���ʹ������֡�����óɹ��������ã�����ʧ�ܷ��� 0x00 00
{
		uint8_t POWER_Control1_Data[8];
    POWER_Control1_Data[0] = (Power >> 8);
    POWER_Control1_Data[1] = Power;
    POWER_Control1_Data[2] = (0 >> 8);
    POWER_Control1_Data[3] = 0;
    POWER_Control1_Data[4] = 0x00;
    POWER_Control1_Data[5] = 0x00;
    POWER_Control1_Data[6] = 0x00;
    POWER_Control1_Data[7] = 0x00;
	
    USER_CAN_transmit(hcanx,POWER_Control1_Data,8,2,StdId,0);
}

void POWER_Control1l(CAN_HandleTypeDef *hcanx ,uint16_t StdId)//��ȡ���ݲ���Զ��֡���ʣ�ģ�鷴������������֡
{
    uint8_t POWER_Control1l_Data[6];   
    POWER_Control1l_Data[0] = 0x00;
    POWER_Control1l_Data[1] = 0x00;
    POWER_Control1l_Data[2] = 0x00;
    POWER_Control1l_Data[3] = 0x00;
    POWER_Control1l_Data[4] = 0x00;
    POWER_Control1l_Data[5] = 0x00;
    USER_CAN_transmit(hcanx,POWER_Control1l_Data,6,2,StdId,1);
}

void power_send_handle2(CAN_HandleTypeDef *hcanx)
{
    POWER_Control1l(hcanx,0x610);
    POWER_Control1l(hcanx,0x611);
    POWER_Control1l(hcanx,0x612);
    POWER_Control1l(hcanx,0x613);
}

void power_send_handle1(CAN_HandleTypeDef *hcanx,u16 Max_Power)
{
    POWER_Control1(hcanx,2, 0x600);
    POWER_Control1(hcanx,Max_Power * 100, 0x601);
    POWER_Control1(hcanx,2500, 0x602);
    POWER_Control1(hcanx,7 * 100, 0x603);
}

