#include "can_chassis_transmit.h"


/**************************************************chassis_data************************************/
chassis_data_t can_chassis_data;



void can_chassis_send1()
{
	uint8_t can_chassis_send1_Data[8];
	can_chassis_send1_Data[0] = can_chassis_data.if_follow_gim;
	can_chassis_send1_Data[1] = can_chassis_data.speed_mode;
	can_chassis_send1_Data[2] = can_chassis_data.chassis_mode;
	can_chassis_send1_Data[3] = (uint8_t)(can_chassis_data.rotate_speed >> 8);
	can_chassis_send1_Data[4] = (uint8_t)(can_chassis_data.rotate_speed);
	can_chassis_send1_Data[5] = (uint8_t)(can_chassis_data.yaw_Encoder_filter_rate >> 8);
	can_chassis_send1_Data[6] = (uint8_t)can_chassis_data.yaw_Encoder_filter_rate;
	can_chassis_send1_Data[7] = (uint8_t)can_chassis_data.chassis_power_limit;

	USER_CAN2_transmit(can_chassis_send1_Data,8,2,UP_CAN2_TO_DOWN_CAN1_1,0);//can1·¢ËÍ
}

void can_chassis_send2()
{
  uint8_t can_chassis_send2_Data[8];
	can_chassis_send2_Data[0] = (uint8_t)(can_chassis_data.x >> 8);
	can_chassis_send2_Data[1] = (uint8_t)can_chassis_data.x;
	can_chassis_send2_Data[2] = (uint8_t)(can_chassis_data.y >> 8);
	can_chassis_send2_Data[3] = (uint8_t)can_chassis_data.y;
	can_chassis_send2_Data[4] = (uint8_t)(can_chassis_data.chassis_power >> 8);
	can_chassis_send2_Data[5] = (uint8_t)can_chassis_data.chassis_power;
	can_chassis_send2_Data[6] = (uint8_t)(can_chassis_data.chassis_power_buffer >> 8);
	can_chassis_send2_Data[7] = (uint8_t)can_chassis_data.chassis_power_buffer;

	USER_CAN2_transmit(can_chassis_send2_Data,8,2,UP_CAN2_TO_DOWN_CAN1_2,0);//can1·¢ËÍ
}

void can_chassis_send3()
{
	uint8_t can_chassis_send3_Data[8];	
	can_chassis_send3_Data[0] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 56);
	can_chassis_send3_Data[1] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 48);
	can_chassis_send3_Data[2] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 40);
	can_chassis_send3_Data[3] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 32);
	can_chassis_send3_Data[4] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 24);
	can_chassis_send3_Data[5] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 16);
	can_chassis_send3_Data[6] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle >> 8);
	can_chassis_send3_Data[7] = (uint8_t)(can_chassis_data.yaw_Encoder_ecd_angle);
	
	USER_CAN2_transmit(can_chassis_send3_Data,8,2,UP_CAN2_TO_DOWN_CAN1_3,0);//can1·¢ËÍ
}


void can_chassis_task(
										u8 if_follow_gim,
										u8 speed_mode,
										u8 chassis_mode,
										double yaw_encoder_angle,
										int16_t yaw_encoder_filter_rate,
										int16_t x,
										int16_t y,
										int16_t rotate_speed,
										int16_t chassis_power,
										uint16_t chassis_power_buffer,
										u8 chassis_power_limit)
{
	can_chassis_data.if_follow_gim = if_follow_gim;
	can_chassis_data.speed_mode = speed_mode;
	can_chassis_data.chassis_mode = chassis_mode;
	can_chassis_data.yaw_Encoder_ecd_angle = (int64_t)(yaw_encoder_angle*10000);
	can_chassis_data.yaw_Encoder_filter_rate = yaw_encoder_filter_rate;
	can_chassis_data.x = x;
	can_chassis_data.y = y;
	can_chassis_data.rotate_speed = rotate_speed;
	can_chassis_data.chassis_power = chassis_power;
	can_chassis_data.chassis_power_buffer = chassis_power_buffer;
	can_chassis_data.chassis_power_limit = chassis_power_limit;

	can_chassis_send1();
	can_chassis_send2();
	can_chassis_send3();
}


void can_chassis_receive_task(CAN_RxHeaderTypeDef * msg)
{
	
	 uint8_t can_chassis_receive_Data[8];
		switch (msg->StdId)
   {
			case UP_CAN2_TO_DOWN_CAN1_1:
		{
			can_chassis_data.if_follow_gim									=(can_chassis_receive_Data[0]);
			can_chassis_data.speed_mode											=(can_chassis_receive_Data[1]);
			can_chassis_data.chassis_mode										=(can_chassis_receive_Data[2]);
			can_chassis_data.rotate_speed 									=((can_chassis_receive_Data[3]<<8)|can_chassis_receive_Data[4]);
			can_chassis_data.yaw_Encoder_filter_rate				=(can_chassis_receive_Data[5]<<8)|can_chassis_receive_Data[6];
			can_chassis_data.chassis_power_limit						=(can_chassis_receive_Data[7]);	
		}break;
			case UP_CAN2_TO_DOWN_CAN1_2:
		{
			can_chassis_data.x															=(can_chassis_receive_Data[0]<<8)|can_chassis_receive_Data[1];
			can_chassis_data.y															=(can_chassis_receive_Data[2]<<8)|can_chassis_receive_Data[3];
			can_chassis_data.chassis_power									=(can_chassis_receive_Data[4]<<8)|can_chassis_receive_Data[5];
			can_chassis_data.chassis_power_buffer						=(can_chassis_receive_Data[6]<<8)|can_chassis_receive_Data[7];
		}break;
		
			case UP_CAN2_TO_DOWN_CAN1_3:
		{
			can_chassis_data.yaw_Encoder_ecd_angle	=((can_chassis_receive_Data[0]<<56)|\
																								(can_chassis_receive_Data[1]<<48)|\
																								(can_chassis_receive_Data[2]<<40)|\
																								(can_chassis_receive_Data[3]<<32)|\
																								(can_chassis_receive_Data[4]<<24)|\
																								(can_chassis_receive_Data[5]<<16)|\
																								(can_chassis_receive_Data[6]<< 8)|\
																												can_chassis_receive_Data[7]);
		}break;
	}
}

