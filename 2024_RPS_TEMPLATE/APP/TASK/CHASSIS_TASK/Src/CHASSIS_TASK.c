/* Includes ------------------------------------------------------------------*/
#include "CHASSIS_TASK.h"
/*----------------------------------------------------------------------------*/


/* Variables_definination-----------------------------------------------------------------------------------------------*/
chassis_t 		  chassis;
Chassis_angle_t Chassis_angle;
float vx,vy;
/*----------------------------------------------------------------------------------------------------------------------*/






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

//void get_romote_set()
//{
//	vx = chassis.vy;
//	vy = chassis.vx;
//	Chassis_angle.Remote_speed = sqrt((vx*vx)+(vy*vy));
//	if(Chassis_angle.Remote_speed >= 50)
//	{	
//		
//		Chassis_angle.Remote_angle = (2*pi+Chassis_angle.Remote_angle)%(2*pi);
//	}

