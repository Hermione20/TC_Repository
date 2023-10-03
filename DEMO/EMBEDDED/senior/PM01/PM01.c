#include "PM01.h"



void PM01_message_Process(volatile capacitance_message_t *v,CanRxMsg * msg)
{
	switch (msg->StdId)
	{
	case 0x610:
	{
		v->mode = (msg->Data[0] << 8) | msg->Data[1];
		v->mode_sure = (msg->Data[2] << 8) | msg->Data[3];
	}
	break;
	case 0x611:
	{
		v->in_power = (msg->Data[0] << 8) | msg->Data[1];
		v->in_v = (msg->Data[2] << 8) | msg->Data[3];
		v->in_i = (msg->Data[4] << 8) | msg->Data[5];
	}
	break;
	case 0x612:
	{
		v->out_power = (msg->Data[0] << 8) | msg->Data[1];
		v->out_v = (msg->Data[2] << 8) | msg->Data[3];
		v->out_i = (msg->Data[4] << 8) | msg->Data[5];
	}
	break;
	case 0x613:
	{
		v->tempureture=(msg->Data[0]<<8)|msg->Data[1];
		v->time=(msg->Data[2]<<8)|msg->Data[3];
    v->this_time=(msg->Data[4]<<8)|msg->Data[5];
	}break;

	default:
		break;
	}
}

/**********************��������**************************/
void POWER_Control1(CAN_TypeDef *CANx ,uint16_t Power,uint16_t StdId) //���ò���ʹ������֡�����óɹ��������ã�����ʧ�ܷ��� 0x00 00
{
    CanTxMsg tx_message;    
    tx_message.StdId = StdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Data;
    tx_message.DLC = 0x08;
    tx_message.Data[0] = (Power >> 8);
    tx_message.Data[1] = Power;
    tx_message.Data[2] = (0 >> 8);
    tx_message.Data[3] = 0;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;
    tx_message.Data[6] = 0x00;
    tx_message.Data[7] = 0x00;
    CAN_Transmit(CANx,&tx_message);
}
void POWER_Control1l(CAN_TypeDef *CANx ,uint16_t StdId)//��ȡ���ݲ���Զ��֡���ʣ�ģ�鷴������������֡
{
    CanTxMsg tx_message;    
    tx_message.StdId = StdId;
    tx_message.IDE = CAN_Id_Standard;
    tx_message.RTR = CAN_RTR_Remote;//CAN_RTR_Data;
    tx_message.DLC = 0x06;
    tx_message.Data[0] = 0x00;
    tx_message.Data[1] = 0x00;
    tx_message.Data[2] = 0x00;
    tx_message.Data[3] = 0x00;
    tx_message.Data[4] = 0x00;
    tx_message.Data[5] = 0x00;

    CAN_Transmit(CANx,&tx_message);
}

void power_send_handle2(CAN_TypeDef *CANx)
{
    POWER_Control1l(CANx,0x610);
    POWER_Control1l(CANx,0x611);
    POWER_Control1l(CANx,0x612);
    POWER_Control1l(CANx,0x613);
}

void power_send_handle1(CAN_TypeDef *CANx,u16 Max_Power)
{
    POWER_Control1(CANx,2, 0x600);
    POWER_Control1(CANx,Max_Power * 100, 0x601);
    POWER_Control1(CANx,2500, 0x602);
    POWER_Control1(CANx,7 * 100, 0x603);
}

