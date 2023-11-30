#include "control_task.h"


int time_tick = 0;

void control_task(void)
{
	time_tick++;

	buff_karman_filter_calc(&buff_kalman_filter,yaw_angle_ref_aim,pit_angle_ref_aim,&new_location.buff_kf_flag);
	yaw_kalman_filter_calc(&yaw_kalman_filter,gimbal_gyro.yaw_Angle,gimbal_gyro.pitch_Angle,gimbal_gyro.yaw_Gyro,gimbal_gyro.pitch_Gyro,-yaw_Encoder.ecd_angle,Pitch_Encoder.ecd_angle,yaw_Encoder.filter_rate,Pitch_Encoder.filter_rate);
	if(time_tick%2==1)
	{
		chassis_task();
	}	
	if(time_tick%2 == 0)
	{
			gimbal_task();
			shoot_task();
			can_bus_send_task();
	}
}

void control_task_Init(void)
{
		gimbal_parameter_Init();
		chassis_param_init();
		shot_param_init();
}
