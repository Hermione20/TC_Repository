#include "CanBus.h"

static uint32_t can1_count = 0;
static uint32_t can2_count = 0;

	
void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    can1_count++;
	  
		can_chassis_receive_task(msg);
		cap_limit_mode_switch();
		PM01_message_Process(&capacitance_message,msg);

    
}

void Can2ReceiveMsgProcess(CanRxMsg * msg)
{
    can2_count++;
    switch (msg->StdId)
    {

			
		case CM1Encoder_MOTOR:
			M3508orM2006EncoderTask(can2_count,&steering_wheel_chassis.Driving_Encoder[0],msg);
			break;
		
		case CM2Encoder_MOTOR:
			M3508orM2006EncoderTask(can2_count,&steering_wheel_chassis.Driving_Encoder[1],msg);
			break;
		
		case CM3Encoder_MOTOR:
			M3508orM2006EncoderTask(can2_count,&steering_wheel_chassis.Driving_Encoder[2],msg);
			break;
		
		case CM4Encoder_MOTOR:
			M3508orM2006EncoderTask(can2_count,&steering_wheel_chassis.Driving_Encoder[3],msg);
			break;
		
		case GM1Encoder_MOTOR:
			GM6020EncoderTask(can2_count,&steering_wheel_chassis.Heading_Encoder[0],msg,GM1Encoder_Offset);
			break;
		
		case GM2Encoder_MOTOR:
			GM6020EncoderTask(can2_count,&steering_wheel_chassis.Heading_Encoder[1],msg,GM2Encoder_Offset);
			break;
		
		case GM3Encoder_MOTOR:
			GM6020EncoderTask(can2_count,&steering_wheel_chassis.Heading_Encoder[2],msg,GM3Encoder_Offset);
			break;
		
		case GM4Encoder_MOTOR:
			GM6020EncoderTask(can2_count,&steering_wheel_chassis.Heading_Encoder[3],msg,GM4Encoder_Offset);
			break;
		
    default:
        break;
    }
}

