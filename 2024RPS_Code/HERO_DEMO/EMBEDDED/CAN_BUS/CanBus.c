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
		case LEFT_FRICTION:
			{
				M3508orM2006EncoderTask(&general_friction.left_motor,msg);
			}break;
			case RIGHT_FRICTION:
			{
				M3508orM2006EncoderTask(&general_friction.right_motor,msg);
			}break;
    case GIMBAL_PITCH_MOTOR:
		{
			MF_EncoderTask(&Pitch_Encoder,msg,GMPitchEncoder_Offset);
		}break;
		case UP_POKE:
		{
			M3508orM2006EncoderTask(&general_poke.up_poke,msg);
		}break;
		
    default:
        break;
    }
}

void Can2ReceiveMsgProcess(CanRxMsg *msg)
{
		PM01_message_Process(&capacitance_message,msg);
    switch (msg->StdId)
    {
    case GIMBAL_YAW_MOTOR:
    {
        MF_EncoderTask(&yaw_Encoder, msg, GMYawEncoder_Offset);
    }break;
    case CM1Encoder_MOTOR:
    {
        M3508orM2006EncoderTask(&Mecanum_chassis.Driving_Encoder[0],msg);
    }break;
    case CM2Encoder_MOTOR:
    {
        M3508orM2006EncoderTask(&Mecanum_chassis.Driving_Encoder[1],msg);
    }break;
    case CM3Encoder_MOTOR:
    {
        M3508orM2006EncoderTask(&Mecanum_chassis.Driving_Encoder[2],msg);
    }break;
    case CM4Encoder_MOTOR:
    {
        M3508orM2006EncoderTask(&Mecanum_chassis.Driving_Encoder[3],msg);
    }break;
		case DOWN_POKE:
		{
			M3508orM2006EncoderTask(&general_poke.down_poke,msg);
		}break;
    default:
        break;
    }
}

void can_bus_send_task(void)
{
	CAN_9015torsionControl(CAN2,gimbal_data.gim_ref_and_fdb.yaw_motor_input,GIMBAL_YAW_MOTOR);
	CAN_9015torsionControl(CAN1,gimbal_data.gim_ref_and_fdb.pitch_motor_input,GIMBAL_PITCH_MOTOR);
	
		Set_C620andC610_IQ1(CAN1,_42mm_shoot.shoot_ref_and_fdb.right_friction_motor_input,_42mm_shoot.shoot_ref_and_fdb.left_friction_motor_input,0,0);
		Set_C620andC610_IQ2(CAN1,_42mm_shoot.shoot_ref_and_fdb.up_poke_motor_input,0,0,0);
		Set_C620andC610_IQ2(CAN2,_42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input,0,0,0);
	
	
	Set_C620andC610_IQ1(CAN2,chassis.current[0],chassis.current[1],chassis.current[2],chassis.current[3]);
	
	
	
}

