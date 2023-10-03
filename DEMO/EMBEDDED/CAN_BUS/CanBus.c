#include "CanBus.h"

static uint32_t can1_count = 0;
static uint32_t can2_count = 0;


void Can1ReceiveMsgProcess(CanRxMsg * msg)
{
    can1_count++;
    switch (msg->StdId)
    {
			case UP_CAN2_TO_DOWN_CAN1_1:
		{
			can_chassis_data.if_follow_gim									=(msg->Data[0]);
			can_chassis_data.speed_mode											=(msg->Data[1]);
			can_chassis_data.chassis_mode										=(msg->Data[2]);
			can_chassis_data.rotate_speed 									=((msg->Data[3]<<8)|msg->Data[4]);
			can_chassis_data.yaw_Encoder_filter_rate				=(msg->Data[5]<<8)|msg->Data[6];
			can_chassis_data.chassis_power_limit						=(msg->Data[7]);
			cap_limit_mode_switch();			
		}break;
			case UP_CAN2_TO_DOWN_CAN1_2:
		{
			can_chassis_data.x															=(msg->Data[0]<<8)|msg->Data[1];
			can_chassis_data.y															=(msg->Data[2]<<8)|msg->Data[3];
			can_chassis_data.chassis_power									=(msg->Data[4]<<8)|msg->Data[5];
			can_chassis_data.chassis_power_buffer						=(msg->Data[6]<<8)|msg->Data[7];
		}break;
		
			case UP_CAN2_TO_DOWN_CAN1_3:
		{
			can_chassis_data.yaw_Encoder_ecd_angle					=((msg->Data[0]<<56)|(msg->Data[1]<<48)|(msg->Data[2]<<40)|(msg->Data[3]<<32)|(msg->Data[4]<<24)|(msg->Data[5]<<16)|(msg->Data[6]<<8)|msg->Data[7]);
		}break;

		  case GIMBAL_YAW_MOTOR:
		{
			GM6020EncoderTask(can1_count,&yaw_Encoder,msg,GMYawEncoder_Offset);
		}break;
			
    default:
        break;
		
		PM01_message_Process(&capacitance_message,msg);
    }
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

