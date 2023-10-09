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

void HT_430_Information_Receive(CAN_RxHeaderTypeDef * msg,HT430_J10_t *HT430_J10_t,volatile Encoder *v)
{
	
	uint8_t HT_430_Information_Receive_Data[8];
  switch ((msg->StdId&0xfffe)>>4)
	{
		case 0x2f:
		{
			HT430_J10_t->Angle=(HT_430_Information_Receive_Data[1]<<8|HT_430_Information_Receive_Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(HT_430_Information_Receive_Data[5]<<24|HT_430_Information_Receive_Data[4]<<16|HT_430_Information_Receive_Data[3]<<8|HT_430_Information_Receive_Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=HT_430_Information_Receive_Data[7]<<8|HT_430_Information_Receive_Data[6];
		}break;
		
		case 0x40:
		{
			HT430_J10_t->Voltage=HT_430_Information_Receive_Data[0]*0.2;
			HT430_J10_t->Currents=HT_430_Information_Receive_Data[1]*0.03;
			HT430_J10_t->Temperature=HT_430_Information_Receive_Data[2]*0.4;
			HT430_J10_t->DTC=HT_430_Information_Receive_Data[3];
			HT430_J10_t->Operating_State=HT_430_Information_Receive_Data[4];
		}break;
		
		case 0x53:
		{
			HT430_J10_t->Angle=(HT_430_Information_Receive_Data[1]<<8|HT_430_Information_Receive_Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(HT_430_Information_Receive_Data[5]<<24|HT_430_Information_Receive_Data[4]<<16|HT_430_Information_Receive_Data[3]<<8|HT_430_Information_Receive_Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=HT_430_Information_Receive_Data[7]<<8|HT_430_Information_Receive_Data[6];
		}break;
		
		case 0x54:
		{
			HT430_J10_t->Angle=(HT_430_Information_Receive_Data[1]<<8|HT_430_Information_Receive_Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(HT_430_Information_Receive_Data[5]<<24|HT_430_Information_Receive_Data[4]<<16|HT_430_Information_Receive_Data[3]<<8|HT_430_Information_Receive_Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=(HT_430_Information_Receive_Data[7]<<8|HT_430_Information_Receive_Data[6]);
		}break;
		
		case 0x55:
		{
			HT430_J10_t->Angle=(HT_430_Information_Receive_Data[1]<<8|HT_430_Information_Receive_Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(HT_430_Information_Receive_Data[5]<<24|HT_430_Information_Receive_Data[4]<<16|HT_430_Information_Receive_Data[3]<<8|HT_430_Information_Receive_Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=HT_430_Information_Receive_Data[7]<<8|HT_430_Information_Receive_Data[6];
		}break;
		
		case 0x56:
		{
			HT430_J10_t->Angle=(HT_430_Information_Receive_Data[1]<<8|HT_430_Information_Receive_Data[0])*(360/16384.0f);
			HT430_J10_t->Total_Angle=(HT_430_Information_Receive_Data[5]<<24|HT_430_Information_Receive_Data[4]<<16|HT_430_Information_Receive_Data[3]<<8|HT_430_Information_Receive_Data[2])*(360/16384.0f)*0.1f;
			HT430_J10_t->V=HT_430_Information_Receive_Data[7]<<8|HT_430_Information_Receive_Data[6];
		}break;
		
		case 0x57:
		{
//			HT430_J10_t->V=(HT_430_Information_Receive_Data[1]<<8|HT_430_Information_Receive_Data[0])*0.1;
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
void HT_430_Encoder_Calibration(CAN_HandleTypeDef *hcanx,int ID)
{
  uint8_t HT_430_Encoder_Calibration[8];
	USER_CAN_transmit(hcanx,HT_430_Encoder_Calibration,0,2,(0x20<<4|ID),0);
}

/*���õ����ǰλ��Ϊԭ�㣻����յ�����������õ����ǰλ��Ϊԭ�㲢���������
ģʽ�л�Ϊ�ر�ģʽ����0x21��*/

void HT_430_Encoder_Origin(CAN_HandleTypeDef *hcanx,int ID)
{
	uint8_t HT_430_Encoder_Origin[8];
	USER_CAN_transmit(hcanx,HT_430_Encoder_Origin,0,2,(0x21<<4|ID),0);
}


void Motor_Information_Request(CAN_HandleTypeDef *hcanx,int ID)
{
  uint8_t Motor_Information_Request_Data[8];
//	
//	Motor_HT430_CanTxMsg.StdId = (0x2f<<4|ID);

//	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));

	USER_CAN_transmit(hcanx,Motor_Information_Request_Data,0,2,(0x40<<4|ID),0);
}

/*���ϵͳ��ǰ���ϣ���ѹ���ϡ��������ϡ��¶ȹ��ϣ�����0x41��*/
void HT_430_Fault_Clear(CAN_HandleTypeDef *hcanx,int ID)
{
	uint8_t HT_430_Fault_Clear_Data[8];
	USER_CAN_transmit(hcanx,HT_430_Fault_Clear_Data,0,2,(0x41<<4|ID),0);
}

/*�رյ�����������ر�ģʽ������������̬���ܿ��ƣ�����ϵ��Ϊ��ģʽ����0x50��*/
void HT_430_Tuen_Off(CAN_HandleTypeDef *hcanx,int ID)
{
	uint8_t HT_430_Tuen_Off_Data[8];
	
	USER_CAN_transmit(hcanx,HT_430_Tuen_Off_Data,0,2,(0x50<<4|ID),0);
}

/*������ݵ�ǰ��Ȧ����ֵ�Ƕȣ��ص��趨��ԭ�㣻��0x51��*/
void HT_430_Origin_Total(CAN_HandleTypeDef *hcanx,int ID)
{
	uint8_t HT_430_Origin_Total_Data[8];

	
	USER_CAN_transmit(hcanx,HT_430_Origin_Total_Data,0,2,(0x51<<4|ID),0);
}
/*���������̵ľ���ص��趨��ԭ�㣬��ת�ĽǶȲ����� 180 �ȣ���0x52��*/

void HT_430_Back(CAN_HandleTypeDef *hcanx,int ID)
{
	uint8_t HT_430_Back_Data[8];

	USER_CAN_transmit(hcanx,HT_430_Back_Data,0,2,(0x52<<4|ID),0);
}
//���ʿ�������
void HT_430_Power_Open_Loop(CAN_HandleTypeDef *hcanx,int ID,int16_t Pow)
{
	uint8_t HT_430_Power_Open_Loop_Data[8];
	HT_430_Power_Open_Loop_Data[0]=Pow&0x00ff;
	HT_430_Power_Open_Loop_Data[1]=(Pow&0xff00)>>8;
//	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
	USER_CAN_transmit(hcanx,HT_430_Power_Open_Loop_Data,2,2,(0x53<<4|ID),0);
	
}
//�ٶȱջ�����Ŀ���ٶȣ���λΪ 0.1RPM���������� int16_t ����
void HT_430_V_Clossed_Loop(CAN_HandleTypeDef *hcanx,int ID,int16_t V)
{
	uint8_t HT_430_V_Clossed_Loop_Data[8];

	HT_430_V_Clossed_Loop_Data[0]=V&0x00ff;
	HT_430_V_Clossed_Loop_Data[1]=(V&0xff00)>>8;
//	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
		USER_CAN_transmit(hcanx,HT_430_V_Clossed_Loop_Data,2,2,(0x54<<4|ID),0);
}
//���ʿ�������
/*Ŀ�����ֵλ�� Count ֵ
�������� uint32_t*/
void HT_430_Absolute_Position_closed_Loop(CAN_HandleTypeDef *hcanx,int ID,uint32_t Angle)
{ 
	uint8_t HT_430_Absolute_Position_closed_Loop_Data[8];
	uint32_t Count;	
	Count=Angle*16384/360;
	HT_430_Absolute_Position_closed_Loop_Data[0]=Count&0xff;
	HT_430_Absolute_Position_closed_Loop_Data[1]=(Count&0xff00)>>8;
	HT_430_Absolute_Position_closed_Loop_Data[2]=(Count&0xff0000)>>16;
	HT_430_Absolute_Position_closed_Loop_Data[3]=(Count&0xff000000)>>24;
	
//	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
	USER_CAN_transmit(hcanx,HT_430_Absolute_Position_closed_Loop_Data,4,2,(0x55<<4|ID),0);
}
/*������λ�ñջ����ƣ�������ڵ�ǰλ������˶��ĽǶȡ������������������Ϊ
int32_t�����ݵͰ汾Э���е� int16_t �������ͣ���������ֵΪ����ʱ����ʾ�����ת��
�����תһȦΪ 16384 �� Count����0x56��*/
void HT_430_Relative_Position_closed_Loop(CAN_HandleTypeDef *hcanx,int ID,uint32_t Angle)
{
	uint8_t HT_430_Relative_Position_closed_Loop_Data[8];
	uint32_t Count;
	
	Count=Angle*16384/360;
	
	HT_430_Relative_Position_closed_Loop_Data[0]=Count&0xff;
	HT_430_Relative_Position_closed_Loop_Data[1]=(Count&0xff00)>>8;
	HT_430_Relative_Position_closed_Loop_Data[2]=(Count&0xff0000)>>16;
	HT_430_Relative_Position_closed_Loop_Data[3]=(Count&0xff000000)>>24;
	
//	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
	USER_CAN_transmit(hcanx,HT_430_Relative_Position_closed_Loop_Data,4,2,(0x56<<4|ID),0);
}

/*λ�ñջ�Ŀ���ٶȶ�ȡ�����ã���ȡ�����ǰ���õ�λ�ñջ�Ŀ���ٶȣ������õ��λ
�ñջ�Ŀ���ٶȲ��������������ϵ��λ�ñջ�Ŀ���ٶȵ�Ĭ��ֵΪ��ͨ�� 0x0E ��
��浽�����ֵ����ǰ����д���λ�ñջ�Ŀ���ٶ�ֻ��д�뵽��������ϵ粻���档
д��ɹ��󣬵���ھ���ֵλ�û����λ�ñջ�ģʽ�½��������õ��ٶ��˶�����0x57��*
0x00����ȡλ�ñջ�Ŀ���ٶ�
0x01������λ�ñջ�Ŀ���ٶ�*/
void HT_430_Position_closed_Loop_T_R_OR_W(CAN_HandleTypeDef *hcanx,int ID,int16_t V,int Flag_RW)
{
	uint8_t HT_430_Position_closed_Loop_T_R_OR_W_Data[8];
	

	HT_430_Position_closed_Loop_T_R_OR_W_Data[0]=Flag_RW;
	*(int16_t*)HT_430_Position_closed_Loop_T_R_OR_W_Data[1]=V;
	
//	CAN_TransmitStatus(CANx,CAN_Transmit(CANx,&Motor_HT430_CanTxMsg));
	USER_CAN_transmit(hcanx,HT_430_Position_closed_Loop_T_R_OR_W_Data,3,2,(0x57<<4|ID),0);
}
