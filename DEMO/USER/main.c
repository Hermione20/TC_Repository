#include "main.h"



int main()
{
   BSP_Init();
	 PID_struct_init(&pid_yaw ,POSITION_PID ,10000,10000,3,2,0);
	
	while(1)
	{
			pid_calc(&pid_yaw,yaw_Encoder.filter_rate,200);
			Set_GM6020_IQ1(CAN2,0,0,pid_yaw.out,0);
//		get_romote_set();
//			chassis_task();
	}
}
