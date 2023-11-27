#include "control_task.h"


int time_tick = 0;

void control_task(void)
{
	time_tick++;


	if(time_tick%2==1)
	{
		can_chassis_task(CAN2,chassis.follow_gimbal,
							chassis.chassis_speed_mode ,
							chassis.ctrl_mode,yaw_Encoder.ecd_angle,
							yaw_Encoder.filter_rate,
							chassis.ChassisSpeed_Ref.left_right_ref,
							chassis.ChassisSpeed_Ref.forward_back_ref,
							chassis.ChassisSpeed_Ref.rotate_ref,
							judge_rece_mesg.power_heat_data.chassis_power,
							judge_rece_mesg.power_heat_data.chassis_power_buffer,
							judge_rece_mesg.game_robot_state.chassis_power_limit);
	}	
	if(time_tick%2 == 0)
	{
			gimbal_task();
			can_bus_send_task();
	}
}

void control_task_Init(void)
{
		gimbal_parameter_Init();
}
