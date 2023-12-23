#include "CanBus.h"


/**
  ******************************************************************************
  * @file    CanBus.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   ���ļ���������can���ߵķ������������������ģ��id��ȥͷ�ļ�����
						 
	* @notice  �й�can��ģ��Ľ��պ�����ʹ�����Ʋ���can_bus.c�ļ��в������еĽ���
						 ������ѡ��ģ��id�����ý��㺯����Ҳ����can_bus.h�ļ����޸�ģ��id
						 �й�can�ķ��ͺ�����ʹ��Ҳ�Ʋ���can_bus.c�ļ��������ܷ�����������
						 ����Ҫ���͵ĺ���������ؽ�can_bus_send_task�����ĵ��÷���controltask
						 �С�
@verbatim
 ===============================================================================
 **/
 
 

void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {
			case JM3Encoder_MOTOR:
		{
			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[2],msg,JM3Encoder_Offset,0.017368678);
			
		}
        break;
    case TM1Encoder_MOTOR:
		{
			MF_18bit_EncoderTask(&balance_chassis.Driving_Encoder[0],msg,TM1Encoder_Offset,0.002597741);
			
		}break;
    case TM2Encoder_MOTOR:
		{
			MF_18bit_EncoderTask(&balance_chassis.Driving_Encoder[1],msg,TM2Encoder_Offset,0.002597741);
			
		}break;
    default:
        break;
    }
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {
    case JM1Encoder_MOTOR:
		{
			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[0],msg,JM1Encoder_Offset,0.017368678);
			
		}
        break;
    case JM2Encoder_MOTOR:
		{
			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[1],msg,JM2Encoder_Offset,0.017368678);
			
		}
        break;
		
    
    case JM4Encoder_MOTOR:
		{
			MG_18bit_EncoderTask(&balance_chassis.joint_Encoder[3],msg,JM4Encoder_Offset,0.017368678);
			
		}
        break;
		

    default:
        break;
    }
}








void can_bus_send_task(void)
{
	CAN_MG_single_torsionControl(CAN2,b_chassis.joint_T[0],0x141,0.017368678);
	CAN_MG_single_torsionControl(CAN2,-b_chassis.joint_T[1],0x142,0.017368678);
	CAN_MG_single_torsionControl(CAN1,-b_chassis.joint_T[2],0x143,0.017368678);
	CAN_MG_single_torsionControl(CAN2,b_chassis.joint_T[3],0x144,0.017368678);
	
	CAN_MF_single_torsionControl(CAN1,b_chassis.driving_T[0],0x141,0.002597741);
//	CAN_LK_TechspeedControl(CAN1,36000,0x141);
	CAN_MF_single_torsionControl(CAN1,-b_chassis.driving_T[1],0x142,0.002597741);
	
}

