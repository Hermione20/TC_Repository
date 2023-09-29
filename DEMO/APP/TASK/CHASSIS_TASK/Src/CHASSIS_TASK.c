/* Includes ------------------------------------------------------------------*/
#include "CHASSIS_TASK.h"
/*----------------------------------------------------------------------------*/


/* Variables_definination-----------------------------------------------------------------------------------------------*/
 
ChassisSpeed_Ref_t ChassisSpeedRef;
Chassis_angle_t 	 Chassis_angle;
chassis_t 		 		 chassis;
steering_wheel_t   steering_wheel;

int   yaw_num_get,run_flag;
float vx,vy;
u8 chassis_rotate_flag;
int getnumb1;
int getnumb2;
int getnumb3;
int getnumb4;
int num1;
int num2;
int num3;
int num4;
float getlastangle1;
float getlastangle2;
float getlastangle3;
float getlastangle4;
float transition1[4];
float transition2[4];
float transition3[4];
float transition4[4];

float retransition1_angle[4];
float retransition2[4];
float retransition3[4];
float retransition4[4];

int yaw_num_get;
float angle_flag[4];
float deviation_angle_flag[4];


/*----------------------------------------------------------------------------------------------------------------------*/

void chassis_param_init(void)//底盘参数初始化
{
  memset(&chassis, 0, sizeof(chassis_t));
  chassis.ctrl_mode      = CHASSIS_STOP;
  chassis.last_ctrl_mode = CHASSIS_RELAX;
//  power_limit.power_limit_model = CAP_LIMIT;
//  chassis_speed_mode = NORMAL_SPEED_MODE;
	
  chassis.position_ref = 0;
  chassis_rotate_flag = 0;
  PID_struct_init(&pid_front_distance, POSITION_PID, 120, 10, 0.50f, 0.00005f,8 );
  PID_struct_init(&pid_right_distance, POSITION_PID, 120, 10, 0.50f, 0.00005f,10);
  PID_struct_init(&pid_angle_distance, POSITION_PID, 30,  10, 0.15f, 0,0);
  PID_struct_init(&pid_front_distance, POSITION_PID, 120, 10, 0.27f, 0.0001f,4);
  PID_struct_init(&pid_right_distance, POSITION_PID, 120, 10, 0.28f, 0.00015f,4);
  PID_struct_init(&pid_angle_distance, POSITION_PID, 30,  10, 0.15f, 0,0);

	  
		PID_struct_init(&pid_cha_6020_angle[0], POSITION_PID, 8000, 10, 8,0.1f,3);//24, 0.2f,20);
  	PID_struct_init(&pid_cha_6020_speed[0], POSITION_PID, 15000, 500, 210,0.1f,2);//38,0.5f,20);
	
		PID_struct_init(&pid_cha_6020_angle[1], POSITION_PID, 8000, 10, 8,0.1f,5);//25, 0.2f,15);
  	PID_struct_init(&pid_cha_6020_speed[1], POSITION_PID, 15000, 500, 180,0.1,4);//39,0.5f,20);
	
	  PID_struct_init(&pid_cha_6020_angle[2], POSITION_PID, 8000, 10, 8,0.1f,4);//20, 0.2f,20);
  	PID_struct_init(&pid_cha_6020_speed[2], POSITION_PID, 15000, 500, 200,0.5,8);//40,0.5f,20);
		
    PID_struct_init(&pid_cha_6020_angle[3], POSITION_PID, 8000, 10, 9,0.4f,4);//23, 0.2f,15);
  	PID_struct_init(&pid_cha_6020_speed[3], POSITION_PID, 15000, 500, 200,0.1,10);//42,0.5f,20);
	

#if STANDARD == 3
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,24,0.3, 10); //24 0.3 10    38.0f,3.0f, 40
    }
 PID_struct_init(&pid_chassis_angle, POSITION_PID, 500, 0, 300,0,500);//xisanhao
#elif STANDARD == 4
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.4, 0); //24 0.3 10    38.0f,3.0f, 40
    }
    PID_struct_init(&pid_chassis_angle, POSITION_PID, 500, 0, 300,0,500);//xisanhao
	#elif STANDARD == 5
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,22,0.3f, 0.5f); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, 500, 0, 300,0,500);//xisanhao
#elif STANDARD == 6
  for (int k = 0; k < 4; k++)
    {
      PID_struct_init(&pid_spd[k], POSITION_PID,15000, 15000,25,0.1, 5); //24 0.3 10    38.0f,3.0f, 40
    }
  PID_struct_init(&pid_chassis_angle, POSITION_PID, 500, 0, 300,0,500);//xisanhao
#endif
}

void limit_angle_to_0_2pi(float angle)
{
		if (angle>=2*PI)angle -= 2*PI;
		else if(angle<0)angle += 2*PI;
}


double convert_ecd_angle_to_0_2pi(double ecd_angle,float _0_2pi_angle)
{
	_0_2pi_angle=fmod(ecd_angle*ANGLE_TO_RAD,2*PI);	
	if(_0_2pi_angle<0)
		 _0_2pi_angle+=2*PI;

	return _0_2pi_angle;
}

//void get_chassis_speed_ref(Remote *rc)
//{
//	if(gim.ctrl_mode != GIMBAL_INIT)
//	{
//		ChassisSpeedRef.forward_back_ref = (rc->ch1- 1024u * 1.5);
//    ChassisSpeedRef.left_right_ref   = (rc->ch0- 1024u * 1.5);
//	}
//}
void get_chassis_ctrl_mode()
{
	if(chassis.chassis_gim==STANDBY||RC_CtrlData.inputmode==STOP)
	{
		chassis.ctrl_mode=CHASSIS_STOP;
	}
	else if((chassis.ctrl_mode!=CHASSIS_ROTATE)&&
				 (chassis .ctrl_mode!=CHASSIS_REVERSE)&&
				 ( chassis.ctrl_mode!=CHASSIS_CHANGE_REVERSE)&&
				 ( chassis.ctrl_mode!=CHASSIS_AUTO_SUP))
				{
					 chassis.ctrl_mode=MANUAL_FOLLOW_GIMBAL;
				}
}
void chassis_stop_handle(void)
{
  chassis.vy = 0;
  chassis.vx = 0;
  chassis.vw = 0;
}



float homing_flag,last_homing_flag;
void follow_gimbal_handle(void)
{

  if( (RC_CtrlData.Key_Flag.Key_A_Flag == 1)||
		  (RC_CtrlData.Key_Flag.Key_D_Flag == 1)||
			(RC_CtrlData.Key_Flag.Key_W_Flag == 1)||
			(RC_CtrlData.Key_Flag.Key_S_Flag == 1)
		)
    {
      if(chassis.chassis_speed_mode == NORMAL_SPEED_MODE)
        {
          chassis.forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
          chassis.left_right_speed   =  NORMAL_LEFT_RIGHT_SPEED;
        }
      else if(chassis.chassis_speed_mode == HIGH_SPEED_MODE&&chassis.ctrl_mode==CHASSIS_ROTATE)
        {			
					chassis.forward_back_speed =	CHASSIS_ROTATE_MOVING_FORWARD_BACK_SPEED;
          chassis.left_right_speed   =	CHASSIS_ROTATE_MOVING_LEFT_RIGHT_SPEED;
				}
			else
			  {					
          chassis.forward_back_speed =	HIGH_FORWARD_BACK_SPEED;
          chassis.left_right_speed   =	HIGH_LEFT_RIGHT_SPEED;
        }
    }

//加入麦轮解算前
  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;
//加入麦轮解算后
//		chassis.sin_chassis_angle = sin(GMYawEncoder.ecd_angle*ANGLE_TO_RAD);
//    chassis.cos_chassis_angle = cos(GMYawEncoder.ecd_angle*ANGLE_TO_RAD);
//    
//    chassis.foward_back_to_foward_back_rotate_speed = ChassisSpeedRef.forward_back_ref*chassis.cos_chassis_angle;
//    chassis.foward_back_to_left_right_rotate_speed = ChassisSpeedRef.forward_back_ref*chassis.sin_chassis_angle;
//    chassis.left_right_to_foward_back_rotate_speed = -ChassisSpeedRef.left_right_ref*chassis.sin_chassis_angle;
//    chassis.left_right_to_left_right_rotate_speed = ChassisSpeedRef.left_right_ref*chassis.cos_chassis_angle;
//    
//    chassis.vy = chassis.foward_back_to_left_right_rotate_speed + chassis.left_right_to_left_right_rotate_speed;
//    chassis.vx = chassis.foward_back_to_foward_back_rotate_speed + chassis.left_right_to_foward_back_rotate_speed;

		if(Chassis_angle.yaw_angle_0_2pi>=PI)
		{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi-(2*PI);}
		else
		{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi;}

//  gim.ctrl_mode!= GIMBAL_AUTO_SMALL_BUFF &&					
//	gim.ctrl_mode!=   GIMBAL_AUTO_BIG_BUFF &&
		if( chassis.chassis_gim==READY&&
				chassis.follow_gimbal)
    {
       Chassis_angle.get_speedw = pid_calc(&pid_chassis_angle,Chassis_angle.yaw_angle__pi_pi,0); 
    }
  else
    {
       Chassis_angle.get_speedw = 0;
    }

	if(fabs(pid_chassis_angle.get-pid_chassis_angle.set)<0.05)
	{
		Chassis_angle.get_speedw = 0;
	}
}


void separate_gimbal_handle(void)
{

	if( (RC_CtrlData.Key_Flag.Key_A_Flag == 1)||
		  (RC_CtrlData.Key_Flag.Key_D_Flag == 1)||
			(RC_CtrlData.Key_Flag.Key_W_Flag == 1)||
			(RC_CtrlData.Key_Flag.Key_S_Flag == 1)
		)
    {
       if(chassis.chassis_speed_mode == NORMAL_SPEED_MODE)
        {
          chassis.forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
          chassis.left_right_speed   =  NORMAL_LEFT_RIGHT_SPEED;
        }
  else if(chassis.chassis_speed_mode == HIGH_SPEED_MODE)
        {
          chassis.forward_back_speed =  HIGH_FORWARD_BACK_SPEED;
          chassis.left_right_speed   =  HIGH_LEFT_RIGHT_SPEED;
        }
    }

  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;
		
	Chassis_angle.get_speedw = 0;
}


u32 rotate_mode_count = 0;
u8  rotate_mode = 0;
u8  sin_constant_flag;
u8  get_speedw_flag = 0;
u16 gyro_speed=400;
void rotate_follow_gimbal_handle(void)
{
//	if( (RC_CtrlData.Key_Flag.Key_A_Flag == 1)||
//		  (RC_CtrlData.Key_Flag.Key_D_Flag == 1)||
//			(RC_CtrlData.Key_Flag.Key_W_Flag == 1)||
//			(RC_CtrlData.Key_Flag.Key_S_Flag == 1)
//		)
//    {
//      if(chassis.chassis_speed_mode == NORMAL_SPEED_MODE)
//        { 
//          chassis.forward_back_speed =  NORMAL_FORWARD_BACK_SPEED;
//          chassis.left_right_speed 	 =  NORMAL_LEFT_RIGHT_SPEED;
//        }
// else if(chassis.chassis_speed_mode ==  HIGH_SPEED_MODE)
//        { 
//          chassis.forward_back_speed =  CHASSIS_ROTATE_MOVING_FORWARD_BACK_SPEED;
//          chassis.left_right_speed   =  CHASSIS_ROTATE_MOVING_LEFT_RIGHT_SPEED;
//        }
//    }

//		if(high_speed_flag==1)
//		{
//			gyro_speed=550;
//		}
//		else
//		{
//			gyro_speed=400;
//		}
		
//if(gim.ctrl_mode==GIMBAL_AUTO_SMALL_BUFF||gim.ctrl_mode==GIMBAL_AUTO_BIG_BUFF)
//	{gyro_speed=100;}

  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;


if(chassis.last_ctrl_mode!=CHASSIS_ROTATE)
{
	if(get_speedw_flag==0)
	{get_speedw_flag=1;}
	else if(get_speedw_flag==1)
	{get_speedw_flag=0;}
}


if(get_speedw_flag==0)
{Chassis_angle.get_speedw = -gyro_speed;}
else if(get_speedw_flag==1)
{Chassis_angle.get_speedw = gyro_speed;}

}


 void reverse_follow_gimbal_handle(void)
{
   if((RC_CtrlData.Key_Flag.Key_A_Flag == 1)||
			(RC_CtrlData.Key_Flag.Key_D_Flag == 1)||
			(RC_CtrlData.Key_Flag.Key_W_Flag == 1)||
			(RC_CtrlData.Key_Flag.Key_S_Flag == 1)
	   )
    {
      if(chassis.chassis_speed_mode == NORMAL_SPEED_MODE)
        {
          chassis.forward_back_speed = NORMAL_FORWARD_BACK_SPEED;
          chassis.left_right_speed   = NORMAL_LEFT_RIGHT_SPEED;
        }
			else
			  {					
          chassis.forward_back_speed = HIGH_FORWARD_BACK_SPEED;
          chassis.left_right_speed   = HIGH_LEFT_RIGHT_SPEED;
        }
    }

  chassis.vy = ChassisSpeedRef.left_right_ref;
  chassis.vx = ChassisSpeedRef.forward_back_ref;


	if(Chassis_angle.yaw_angle_0_2pi>=2*PI)
	{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi-(2*PI);}
	else
	{Chassis_angle.yaw_angle__pi_pi=Chassis_angle.yaw_angle_0_2pi;}

//	gim.ctrl_mode!=GIMBAL_AUTO_SMALL_BUFF&&gim.ctrl_mode!=GIMBAL_AUTO_BIG_BUFF
  if (chassis.follow_gimbal&&chassis.chassis_gim==STANDBY)		
      Chassis_angle.get_speedw = pid_calc(&pid_chassis_angle,Chassis_angle.yaw_angle__pi_pi,PI);    
  else
       Chassis_angle.get_speedw = 0;

	if(fabs(pid_chassis_angle.get-pid_chassis_angle.set)<0.05)
			Chassis_angle.get_speedw = 0;
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
void start_angle_handle()
{
	
	Chassis_angle.start_angle[0] = Chassis_angle.yaw_angle_0_2pi + ((3*PI)/4);
	Chassis_angle.start_angle[1] = Chassis_angle.yaw_angle_0_2pi + ((5*PI)/4);
	Chassis_angle.start_angle[2] = Chassis_angle.yaw_angle_0_2pi + ((7*PI)/4);
	Chassis_angle.start_angle[3] = Chassis_angle.yaw_angle_0_2pi + ((1*PI)/4);
	for(int k=0;k<4;k++)
	{limit_angle_to_0_2pi(Chassis_angle.start_angle[k]);}

for(int k=0;k<4;k++)
{
	Chassis_angle.include_angle[k] = PI + Chassis_angle.Remote_angle - Chassis_angle.start_angle[k];
	limit_angle_to_0_2pi(Chassis_angle.include_angle[k]);

	Chassis_angle.handle_speed[k] = sqrt(Chassis_angle.get_speedw*Chassis_angle.get_speedw + Chassis_angle.Remote_speed*Chassis_angle.Remote_speed \
																				- 2*Chassis_angle.get_speedw*Chassis_angle.Remote_speed*(cos(Chassis_angle.include_angle[k])));

//	Chassis_angle.handle_speed1[k] = sqrt(Chassis_angle.get_speedw*Chassis_angle.get_speedw + 
//(Chassis_angle.Remote_speed*power_limit_start_flag)*(power_limit_start_flag*Chassis_angle.Remote_speed) 
//	- 2*Chassis_angle.get_speedw*(Chassis_angle.Remote_speed*power_limit_start_flag)*(cos(Chassis_angle.include_angle[k])));
}

for(int k=0;k<4;k++)
{
	
	transition1[k] = sin(Chassis_angle.include_angle[k]);
	transition2[k] = Chassis_angle.Remote_speed*transition1[k];
	transition3[k] = (transition2[k])/Chassis_angle.handle_speed[k];	
	VAL_LIMIT(transition3[k],-1,1);
	Chassis_angle.deviation_angle[k] = asin(transition3[k]);
	
	retransition2[k] = Chassis_angle.get_speedw*Chassis_angle.get_speedw+Chassis_angle.handle_speed[k]*Chassis_angle.handle_speed[k];
	retransition3[k] = Chassis_angle.Remote_speed*Chassis_angle.Remote_speed;
	
	if(Chassis_angle.get_speedw==0)
	{
	if(Chassis_angle.include_angle[k]>=0&&Chassis_angle.include_angle[k]<=(PI/2))
	{Chassis_angle.deviation_angle[k] += PI;
	 Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}

	else if(Chassis_angle.include_angle[k]>=(3*PI/2)&&Chassis_angle.include_angle[k]<=(2*PI))
	{Chassis_angle.deviation_angle[k] += PI;
	 Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}
	}
	else 
	{
	if(Chassis_angle.get_speedw<0)
	{Chassis_angle.deviation_angle[k] += PI;
	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}
	
	if(retransition3[k]>retransition2[k])	//突然需要很大的角速度，那么反向抵达
	{Chassis_angle.deviation_angle[k] += PI;
	Chassis_angle.deviation_angle[k] = -Chassis_angle.deviation_angle[k];}
  }
	}
}


void start_chassis_6020()
{
		chassis.cha_pid_6020.angle1_fdb = steering_wheel.left_front_GM6020.ecd_angle;
		chassis.cha_pid_6020.angle2_fdb = steering_wheel.right_front_GM6020.ecd_angle;
		chassis.cha_pid_6020.angle3_fdb = steering_wheel.right_behind_GM6020.ecd_angle;
		chassis.cha_pid_6020.angle4_fdb = steering_wheel.left_behind_GM6020.ecd_angle;

	if((Chassis_angle.Remote_speed>50)||
			((Chassis_angle.get_speedw<-5)||
			 (Chassis_angle.get_speedw> 5)))	
	{
		if((steering_wheel.left_front_GM6020.ecd_angle-getnumb1*360)>360)
		{getnumb1++;}
		else if
		((steering_wheel.left_front_GM6020.ecd_angle-getnumb1*360)<0)
		{getnumb1--;}

		if((steering_wheel.right_front_GM6020.ecd_angle-getnumb2*360)>360)
		{getnumb2++;}
		else if
		((steering_wheel.right_front_GM6020.ecd_angle-getnumb2*360)<0)
		{getnumb2--;}

		if((steering_wheel.right_behind_GM6020.ecd_angle-getnumb3*360)>360)
		{getnumb3++;}
		else if
		((steering_wheel.right_behind_GM6020.ecd_angle-getnumb3*360)<0)
		{getnumb3--;}

		if((steering_wheel.left_behind_GM6020.ecd_angle-getnumb4*360)>360)
		{getnumb4++;}
		else if
		((steering_wheel.right_behind_GM6020.ecd_angle-getnumb4*360)<0)
		{getnumb4--;}
	}
		if(Chassis_angle.Remote_speed <= 50)//静止时
	{
		chassis.cha_pid_6020.angle_ref[0] = (getnumb1)*360;
		chassis.cha_pid_6020.angle_ref[1] = (getnumb2)*360;
		chassis.cha_pid_6020.angle_ref[2] = (getnumb3)*360;
		chassis.cha_pid_6020.angle_ref[3] = (getnumb4)*360;
		if(Chassis_angle.get_speedw < 0) //方向反向
		{
		chassis.cha_pid_6020.angle_ref[0] = chassis.cha_pid_6020.angle_ref[0] - 180;
		chassis.cha_pid_6020.angle_ref[1] = chassis.cha_pid_6020.angle_ref[1] - 180;
		chassis.cha_pid_6020.angle_ref[2] = chassis.cha_pid_6020.angle_ref[2] - 180;
		chassis.cha_pid_6020.angle_ref[3] = chassis.cha_pid_6020.angle_ref[3] - 180;
		}	
	}
	else
	{
   chassis.cha_pid_6020.angle_ref[0] = Chassis_angle.deviation_angle[0] * RAD_TO_ANGLE + (getnumb1)*360;
   chassis.cha_pid_6020.angle_ref[1] = Chassis_angle.deviation_angle[1] * RAD_TO_ANGLE + (getnumb2)*360;
   chassis.cha_pid_6020.angle_ref[2] = Chassis_angle.deviation_angle[2] * RAD_TO_ANGLE + (getnumb3)*360;
   chassis.cha_pid_6020.angle_ref[3] = Chassis_angle.deviation_angle[3] * RAD_TO_ANGLE + (getnumb4)*360;
	}
	
	Ref_Fdb(chassis.cha_pid_6020.angle_ref[0],chassis.cha_pid_6020.angle_fdb[0]);//最多转劣弧
	Ref_Fdb(chassis.cha_pid_6020.angle_ref[1],chassis.cha_pid_6020.angle_fdb[1]);
	Ref_Fdb(chassis.cha_pid_6020.angle_ref[2],chassis.cha_pid_6020.angle_fdb[2]);
	Ref_Fdb(chassis.cha_pid_6020.angle_ref[3],chassis.cha_pid_6020.angle_fdb[3]);
  
	
//			chassis.cha_pid_3508.speed_ref[0] = -chassis.cha_pid_6020.angle_ref[0];
//		  chassis.cha_pid_3508.speed_ref[1] = -chassis.cha_pid_6020.angle_ref[1];
//			chassis.cha_pid_3508.speed_ref[2] = -chassis.cha_pid_6020.angle_ref[2];
				chassis.cha_pid_3508.speed_ref[3] = -chassis.cha_pid_6020.angle_ref[3];

//此处	chassis.cha_pid_6020.angle_ref 到时候放功率限制后的。
	for(int i=0;i<4;i++)
	{
	if((chassis.cha_pid_6020.angle_ref[i]-chassis.cha_pid_6020.angle_fdb[i])>90)
	{	  
		chassis.cha_pid_6020.angle_ref[i]-=180;
		chassis.cha_pid_6020.speed_ref[i] = -chassis.cha_pid_6020.angle_ref[i];
	}
	else if((chassis.cha_pid_6020.angle_ref[i]-chassis.cha_pid_6020.angle_fdb[i])<-90)
	{
		chassis.cha_pid_6020.angle_ref[i]+=180;
		chassis.cha_pid_6020.speed_ref[i] = -chassis.cha_pid_6020.angle_ref[i];
	}
	}
	
	for(int i=0;i<4;i++)
	pid_calc(&pid_cha_6020_angle[i],chassis.cha_pid_6020.angle_fdb[i],chassis .cha_pid_6020.angle_ref[i]);

	if(chassis .ctrl_mode==MANUAL_FOLLOW_GIMBAL||chassis.ctrl_mode==CHASSIS_ROTATE)
	{
		chassis.cha_pid_3508.speed_fdb[0]=steering_wheel.left_front_motor.filter_rate;
		chassis.cha_pid_3508.speed_fdb[1]=steering_wheel.right_front_motor.filter_rate;
		chassis.cha_pid_3508.speed_fdb[2]=steering_wheel.left_behind_motor.filter_rate;
		chassis.cha_pid_3508.speed_fdb[3]=steering_wheel.right_behind_motor.filter_rate;
	
	for(int i=0;i<4;i++)
	chassis.cha_pid_3508.speed_ref[i]=pid_cha_3508_angle[i].out;
	}
	else{}
	
	for (int i = 0; i < 4; i++)
	pid_calc(&pid_cha_6020_speed[i],chassis.cha_pid_6020.speed_fdb[i],chassis.cha_pid_6020.speed_ref[i]);
}

void set_3508current_6020voltage()
{		
		
//   chassis.current[i] = pid_calc(&pid_spd[i],Chassis_angle.wheel_speed_fdb[i] , power_limit_rate2*power_limit_rate1*Chassis_angle.handle_speed1[i]);
   for (int i = 0; i < 4; i++)
    {   
    if((chassis.ctrl_mode==MANUAL_FOLLOW_GIMBAL||chassis.ctrl_mode==CHASSIS_ROTATE))
				
		chassis.current[i] = 1.0f * pid_calc(&pid_cha_3508_speed[i],chassis.cha_pid_3508.speed_fdb[i],chassis.cha_pid_3508.speed_ref[i]);
		
		else{
//			OpenDoor;
			chassis.current[i]=0;
				}
		}			
		
//		if(chassis.ctrl_mode==CHASSIS_ROTATE)
//		mecanum_calc(chassis.current);
		

				
	//Chassis_angle.handle_speed1 功率控制记得			
		if(fabs(chassis.cha_pid_3508.speed_ref[0])<50&&
			 fabs(chassis.cha_pid_3508.speed_ref[1])<50&&
			 fabs(chassis.cha_pid_3508.speed_ref[2])<50&&
			 fabs(chassis.cha_pid_3508.speed_ref[3])<50)
		{
			run_flag=0;
		}
		else
		{
			run_flag=1;
		}
		
		
for (int i = 0; i < 4; i++)
    {   
		if((chassis.ctrl_mode==MANUAL_FOLLOW_GIMBAL||chassis.ctrl_mode==CHASSIS_ROTATE)&&run_flag)//
		{
			chassis.voltage[i]=1.0f*(int16_t)pid_cha_6020_speed[i].out;
//			CloseDoor;
		}
		else{
//			OpenDoor;
			chassis.voltage[i]=0;
				}
		}
}

		

void get_remote_set()
{	
	float temp_angle=0;
		vx = chassis.vy;//vx，x是横轴
		vy = chassis.vx;//vy，y是纵轴
		Chassis_angle.Remote_speed = sqrt((vx*vx)+(vy*vy));
		if(Chassis_angle.Remote_speed >= 50)
		{	
			temp_angle=atan2(vy,vx);
			Chassis_angle.Remote_angle = fmod(2*PI+temp_angle,2*PI);
		}
		else
		{Chassis_angle.Remote_speed = 0;}
}

/**
************************************************************************************************************************
* @Name     : chassis_task
* @brief    : 
* @retval   : None
* @Note     : 
************************************************************************************************************************
**/
void chassis_task()
{
//	yaw_num_get = -yaw_Encoder.ecd_angle/360;
//	if(-yaw_Encoder .ecd_angle<0)
//	{yaw_num_get -=1;}
//	Chassis_angle.get_yaw_angle=(-yaw_Encoder.ecd_angle-yaw_num_get*360)*ANGLE_TO_RAD;
	
	convert_ecd_angle_to_0_2pi(yaw_Encoder.ecd_angle,Chassis_angle.yaw_angle_0_2pi);
	
	switch (chassis.ctrl_mode)
    {

    case CHASSIS_STOP:
    {
				chassis_stop_handle();
    }
    break;

    case MANUAL_FOLLOW_GIMBAL:   //跟随云台模式
    {
				follow_gimbal_handle();
    }
    break;
    case CHASSIS_ROTATE:         //小陀螺
    {;
				rotate_follow_gimbal_handle();
    }
    break;
    case CHASSIS_REVERSE:
    {
				reverse_follow_gimbal_handle();
    }
    break;
    case CHASSIS_SEPARATE:
    {
				separate_gimbal_handle();
    }
    break;
    default:
    {
				chassis_stop_handle();
    }
    break;
    }
	
	get_remote_set();
	start_angle_handle();
	start_chassis_6020();
	set_3508current_6020voltage();

}
