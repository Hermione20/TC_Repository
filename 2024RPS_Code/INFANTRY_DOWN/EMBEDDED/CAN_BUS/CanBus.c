#include "CanBus.h"



	
void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    
	  
		can_chassis_receive_task(msg);
	#if POWER_LIMIT_HANDLE
		cap_limit_mode_switch();
	#endif
		PM01_message_Process(&capacitance_message,msg);

    
}
uint16_t cnt[4]=0;
void Can2ReceiveMsgProcess(CanRxMsg * msg)
{

    switch (msg->StdId)
    {

			
		case CM1Encoder_MOTOR:
			M3508orM2006EncoderTask(&steering_wheel_chassis.Driving_Encoder[0],msg);
			break;
		
		case CM2Encoder_MOTOR:
			M3508orM2006EncoderTask(&steering_wheel_chassis.Driving_Encoder[1],msg);
			break;
		
		case CM3Encoder_MOTOR:
			M3508orM2006EncoderTask(&steering_wheel_chassis.Driving_Encoder[2],msg);
			break;
		
		case CM4Encoder_MOTOR:
			M3508orM2006EncoderTask(&steering_wheel_chassis.Driving_Encoder[3],msg);
			break;
		
		case GM1Encoder_MOTOR:
		{
			cnt[0]++;
			GM6020EncoderTask(&steering_wheel_chassis.Heading_Encoder[0],msg,GM1Encoder_Offset);
		}
			break;
		
		case GM2Encoder_MOTOR:
		{
			cnt[1]++;
			GM6020EncoderTask(&steering_wheel_chassis.Heading_Encoder[1],msg,GM2Encoder_Offset);
		}
			break;
		
		case GM3Encoder_MOTOR:
		{
			cnt[2]++;
			GM6020EncoderTask(&steering_wheel_chassis.Heading_Encoder[2],msg,GM3Encoder_Offset);
		}
			break;
		
		case GM4Encoder_MOTOR:
		{
			cnt[3]++;
			GM6020EncoderTask(&steering_wheel_chassis.Heading_Encoder[3],msg,GM4Encoder_Offset);
		}
			break;
		
    default:
        break;
    }
}

