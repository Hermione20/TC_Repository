#include "main.h"
int i;

int main()
{
   BSP_Init();
	 control_task_init();
   PID_struct_init(&pid_yaw ,POSITION_PID ,15000,10000,3,2,0);
	 
	
	while(1)
	{

		if(i%84000==0)
		{

		}
		i++;
	}
}
