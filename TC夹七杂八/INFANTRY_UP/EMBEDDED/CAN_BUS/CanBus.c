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

		case 0x203:
		{
			M3508orM2006EncoderTask(&general_poke.left_poke,msg);
		}break;
		case LEFT_FRICTION1:
		{
			M3508orM2006EncoderTask(&general_friction.left_motor1,msg);	
		}break;
		case RIGHT_FRICTION1:
		{
			M3508orM2006EncoderTask(&general_friction.right_motor1,msg);
		}break;
    default:
    break;
    }
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    switch (msg->StdId)
    {
    case GIMBAL_YAW_MOTOR:
			GM6020EncoderTask(&yaw_Encoder,msg,GMYawEncoder_Offset);
        /* code */
        break;
		case GIMBAL_PITCH_MOTOR:
			GM6020EncoderTask(&Pitch_Encoder,msg,GMPitchEncoder_Offset);

    default:
        break;
    }
}








void can_bus_send_task(void)
{
	
	Set_GM6020_IQ1(CAN2,gimbal_data.gim_ref_and_fdb.yaw_motor_input,gimbal_data.gim_ref_and_fdb.pitch_motor_input,0,0);
	Set_C620andC610_IQ1(CAN1,0,0,shoot.poke_current[0],0);
	Set_C620andC610_IQ1(CAN1,shoot.fric_current[0],shoot.fric_current[1],0,0);
}

