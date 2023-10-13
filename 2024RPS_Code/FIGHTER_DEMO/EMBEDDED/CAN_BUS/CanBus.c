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
    case GIMBAL_YAW_MOTOR:
		{
			
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
		

    default:
        break;
    }
		HT_430_Information_Receive(msg,&HT430_J10,&Pitch_Encoder);
}








void can_bus_send_task(void)
{
	
	Set_GM6020_IQ2(CAN2,gimbal_data.gim_ref_and_fdb.yaw_motor_input,0,0,0);
//	HT_430_Power_Open_Loop(CAN2,1,gimbal_data.gim_ref_and_fdb.pitch_motor_input);
	HT_430_V_Clossed_Loop(CAN2,1,gimbal_data.gim_ref_and_fdb.pit_speed_ref);
}

