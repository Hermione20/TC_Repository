/* Includes ------------------------------------------------------------------*/
#include "CHASSIS_TASK.h"
/*----------------------------------------------------------------------------*/


/* Variables_definination-----------------------------------------------------------------------------------------------*/
chassis_t 		  chassis;
Chassis_angle_t Chassis_angle;
int   yaw_num_get;
float vx,vy;
/*----------------------------------------------------------------------------------------------------------------------*/


void chassis_task(void)
{
	yaw_num_get = -yaw_Encoder.ecd_angle/360;
	if(-yaw_Encoder .ecd_angle<0)
	{yaw_num_get -=1;}
	Chassis_angle .get_yaw_angle=(-yaw_Encoder.ecd_angle-yaw_num_get*360 )*ANGLE_TO_RAD;


}


/*
void get_remote_set()
{
 vx = chassis.vy;
 vy = chassis.vx;
 Chassis_angle.Remote_speed = sqrt((vx*vx)+(vy*vy));
if(Chassis_angle.Remote_speed >= 50)
{
 if(vx > 0)
 {
	 Chassis_angle.Remote_angle = atan(vy / vx);
   if(Chassis_angle.Remote_angle < 0)
   {
		 Chassis_angle.Remote_angle += 2*PI;
	 } 
 }
 else if(vx<0) 
 {
	 Chassis_angle.Remote_angle = atan(vy / vx);
	 Chassis_angle.Remote_angle += PI;
 }
 else
  {
		 if(vy < 0)
		 {
		 Chassis_angle.Remote_angle = 3 * PI /2;}
		 else if(vy > 0)
		 {
		 Chassis_angle.Remote_angle = PI / 2;
		 }
  }
 } 
}
*/

void get_romote_set()
{	
	float temp_angle=0;
	vx = chassis.vy;
	vy = chassis.vx;
	Chassis_angle.Remote_speed = sqrt((vx*vx)+(vy*vy));
	if(Chassis_angle.Remote_speed >= 50)
	{	
		temp_angle=atan2(vy,vx);
		Chassis_angle.Remote_angle = fmod(2*PI+temp_angle,2*PI);
	}
}
