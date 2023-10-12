#include "CanBus.h"


/**
  ******************************************************************************
  * @file    CanBus.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件用于配置can总线的发送与接收任务，若设置模块id可去头文件设置
						 
	* @notice  有关can的模块的接收函数的使用请移步至can_bus.c文件中并在其中的接受
						 函数里选择模块id并调用解算函数，也可在can_bus.h文件中修改模块id
						 有关can的发送函数的使用也移步至can_bus.c文件，并在总发送任务函数中
						 配置要发送的函数，请务必将can_bus_send_task函数的调用放在controltask
						 中。
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

