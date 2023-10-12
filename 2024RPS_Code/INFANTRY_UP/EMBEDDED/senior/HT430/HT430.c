#include "HT430.h"

/**
  ******************************************************************************
  * @file    HT430.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   ���ļ���д��HT430�ͺŵ���Ľ��㣬��ڲ�������ͨ��
						 �������ṹ�壬can�ṹ�壬�Դ���HT430_J10�������
						 �ṹ����Ҫ���룬�������㷨����õ�ʱ�������ͨ��
						 �������ṹ��
						 
@verbatim
 ===============================================================================
 **/

HT430_J10_t HT430_J10;



/********************************HT430_J10************************************/

void HT_430_Information_Receive(CanRxMsg * msg,HT430_J10_t *HT430_J10_t,volatile Encoder *v)
{
  switch ((msg->StdId&0xfffe)>>4)
	{
		case 0x2f:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x40:
		{
			HT430_J10_t->Voltage=msg->Data[0]*0.2;
			HT430_J10_t->Currents=msg->Data[1]*0.03;
			HT430_J10_t->Temperature=msg->Data[2]*0.4;
			HT430_J10_t->DTC=msg->Data[3];
			HT430_J10_t->Operating_State=msg->Data[4];
		}break;
		
		case 0x53:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x54:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=(msg->Data[7]<<8|msg->Data[6]);
		}break;
		
		case 0x55:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x56:
		{
			HT430_J10_t->Angle=(msg->Data[1]<<8|msg->Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(msg->Data[5]<<24|msg->Data[4]<<16|msg->Data[3]<<8|msg->Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=msg->Data[7]<<8|msg->Data[6];
		}break;
		
		case 0x57:
		{
//			HT430_J10_t->V=(msg->Data[1]<<8|msg->Data[0])*0.1;
		}break;
		
		default:
		{
		}break;
	}
	HT430_J10_t->V=HT430_J10_t->V*360/16384/6;
	v->ecd_angle = HT430_J10_t->Total_Angle;
	v->filter_rate = HT430_J10_t->V;
}


/************************HT430**************************************/

/*���������У׼���������ǰ�Ѿ��Ա�����������У׼���û����в�ж��������壬��
ִ�и�����Ե������������У׼��ע�⣺���е��������У׼ʱ����ȷ��������ڿ�
��״̬��ͬʱ����У׼������������ŵ��ת����0x20��*/
void HT_430_Encoder_Calibration(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x20<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}

/*���õ����ǰλ��Ϊԭ�㣻����յ�����������õ����ǰλ��Ϊԭ�㲢���������
ģʽ�л�Ϊ�ر�ģʽ����0x21��*/

void HT_430_Encoder_Origin(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x21<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}


void Motor_Information_Request(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;
//	
//	Motor_HT430_CanTxMsg.StdId = (0x2f<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
//	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
	
	Motor_HT430_CanTxMsg.StdId=(0x40<<4|ID);
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}

/*���ϵͳ��ǰ���ϣ���ѹ���ϡ��������ϡ��¶ȹ��ϣ�����0x41��*/
void HT_430_Fault_Clear(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x41<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}

/*�رյ�����������ر�ģʽ������������̬���ܿ��ƣ�����ϵ��Ϊ��ģʽ����0x50��*/
void HT_430_Tuen_Off(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x50<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}

/*������ݵ�ǰ��Ȧ����ֵ�Ƕȣ��ص��趨��ԭ�㣻��0x51��*/
void HT_430_Origin_Total(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x51<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}
/*���������̵ľ���ص��趨��ԭ�㣬��ת�ĽǶȲ����� 180 �ȣ���0x52��*/

void HT_430_Back(CAN_TypeDef *CANx,int ID)
{
	CanTxMsg Motor_HT430_CanTxMsg;

	Motor_HT430_CanTxMsg.StdId=(0x52<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x00;
	
	CAN_Transmit(CANx,&Motor_HT430_CanTxMsg);
}
//���ʿ�������
void HT_430_Power_Open_Loop(CAN_TypeDef *CANx,int ID,int16_t Pow)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	Motor_HT430_CanTxMsg.StdId = (0x53<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x02;
	
	Motor_HT430_CanTxMsg.Data[0]=Pow&0x00ff;
	Motor_HT430_CanTxMsg.Data[1]=(Pow&0xff00)>>8;
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}
//�ٶȱջ�����Ŀ���ٶȣ���λΪ 0.1RPM���������� int16_t ����
void HT_430_V_Clossed_Loop(CAN_TypeDef *CANx,int ID,int16_t V)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	Motor_HT430_CanTxMsg.StdId = (0x54<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x02;
	
	Motor_HT430_CanTxMsg.Data[0]=V&0x00ff;
	Motor_HT430_CanTxMsg.Data[1]=(V&0xff00)>>8;
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}
//���ʿ�������
/*Ŀ�����ֵλ�� Count ֵ
�������� uint32_t*/
void HT_430_Absolute_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	uint32_t Count;
	
	Motor_HT430_CanTxMsg.StdId = (0x55<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x04;
	
	Count=Angle*16384/360;
	
	Motor_HT430_CanTxMsg.Data[0]=Count&0xff;
	Motor_HT430_CanTxMsg.Data[1]=(Count&0xff00)>>8;
	Motor_HT430_CanTxMsg.Data[2]=(Count&0xff0000)>>16;
	Motor_HT430_CanTxMsg.Data[3]=(Count&0xff000000)>>24;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}
/*������λ�ñջ����ƣ�������ڵ�ǰλ������˶��ĽǶȡ������������������Ϊ
int32_t�����ݵͰ汾Э���е� int16_t �������ͣ���������ֵΪ����ʱ����ʾ�����ת��
�����תһȦΪ 16384 �� Count����0x56��*/
void HT_430_Relative_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	uint32_t Count;
	
	Motor_HT430_CanTxMsg.StdId = (0x56<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x04;
	
	Count=Angle*16384/360;
	
	Motor_HT430_CanTxMsg.Data[0]=Count&0xff;
	Motor_HT430_CanTxMsg.Data[1]=(Count&0xff00)>>8;
	Motor_HT430_CanTxMsg.Data[2]=(Count&0xff0000)>>16;
	Motor_HT430_CanTxMsg.Data[3]=(Count&0xff000000)>>24;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}

/*λ�ñջ�Ŀ���ٶȶ�ȡ�����ã���ȡ�����ǰ���õ�λ�ñջ�Ŀ���ٶȣ������õ��λ
�ñջ�Ŀ���ٶȲ��������������ϵ��λ�ñջ�Ŀ���ٶȵ�Ĭ��ֵΪ��ͨ�� 0x0E ��
��浽�����ֵ����ǰ����д���λ�ñջ�Ŀ���ٶ�ֻ��д�뵽��������ϵ粻���档
д��ɹ��󣬵���ھ���ֵλ�û����λ�ñջ�ģʽ�½��������õ��ٶ��˶�����0x57��*
0x00����ȡλ�ñջ�Ŀ���ٶ�
0x01������λ�ñջ�Ŀ���ٶ�*/
void HT_430_Position_closed_Loop_T_R_OR_W(CAN_TypeDef *CANx,int ID,int16_t V,int Flag_RW)
{
	CanTxMsg Motor_HT430_CanTxMsg;
	
	Motor_HT430_CanTxMsg.StdId = (0x57<<4|ID);
	Motor_HT430_CanTxMsg.IDE = CAN_Id_Standard;
	Motor_HT430_CanTxMsg.RTR = CAN_RTR_Data;
	Motor_HT430_CanTxMsg.DLC = 0x03;
	
	Motor_HT430_CanTxMsg.Data[0]=Flag_RW;
	*(int16_t*)Motor_HT430_CanTxMsg.Data[1]=V;
	
	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
}
