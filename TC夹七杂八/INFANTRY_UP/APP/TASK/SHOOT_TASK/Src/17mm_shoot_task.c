#include "17mm_shoot_task.h"

/* Variables_definination-----------------------------------------------------------------------------------------------*/
shoot_t shoot;

u8 press_l_state_switch;
int fric_run_time=0;

uint32_t start_shooting_count = 0;//正转计时
uint32_t start_reversal_count1 = 0;//反转计时
uint32_t start_reversal_count2 = 0;
uint8_t lock_rotor1 = 0;//堵转标志位
uint8_t lock_rotor2 = 0;
/*----------------------------------------------------------------------------------------------------------------------*/

void shoot_task()
{
	
	shoot_state_mode_switch();
#if SHOOT_TYPE == 3
	performance_select();
#endif

	shoot_bullet_handle();
	shoot_friction_handle();

#if SHOOT_TYPE != 6//飞机没热量
	heat0_limit();
#endif

}

void shoot_state_mode_switch()
{
	 /****************************键鼠射击状态更新**********************************************/		
				switch(RC_CtrlData.inputmode)
				{
				case KEY_MOUSE_INPUT:
				{
				if(RC_CtrlData.mouse.press_l==1)
				  shoot.poke_run=1;
				else
					shoot.poke_run=0;
				 
				if(RC_CtrlData.Key_Flag.Key_C_TFlag)
						shoot.fric_wheel_run=1;
				 else
						shoot.fric_wheel_run=0;
				 
				 if(RC_CtrlData.Key_Flag.Key_Q_TFlag)
				 {shoot.bulletspead_level=1;}
				 else
				 {shoot.bulletspead_level=0;}
				 
				     //鼠标左键	（单发标志位press_l_state_switch）			
				 if(RC_CtrlData.mouse.last_press_l==0&&RC_CtrlData.mouse.press_l==1)
						press_l_state_switch=1;
				 else
						press_l_state_switch=0;
					
				RC_CtrlData.mouse.last_press_l=RC_CtrlData.mouse.press_l;
			 }break;
				case REMOTE_INPUT:
			{
				if(RC_CtrlData.rc.ch4>1500)
				  shoot.poke_run=1;
				else
					shoot.poke_run=0;	
				
				if(RC_CtrlData.RemoteSwitch.s3to1)
						shoot.fric_wheel_run=1;
				 else
						shoot.fric_wheel_run=0;
			}break;
				default:
			 break;		
			}
}

#if SHOOT_TYPE == 3//步兵发射模式选择
void performance_select()
{
		shoot_mode_switch();
		speed_switch();
		heat_switch();
}

void shoot_mode_switch()
{
		//弹速限制来改变不同的模式 
  if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==30)          //弹速限制在30为速度模式
	{shoot.shoot_mode_selection=ZOOM;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==50)    //弹速限制为15，但是冷却限制为50
	{shoot.shoot_mode_selection=COOLING;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==150)   //弹速限制为15，但是冷却限制为150
	{shoot.shoot_mode_selection=OUTBURST;}
}

void speed_switch()
{
		static float last_speed_limit;
		if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit!=last_speed_limit)
		{ 
			if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<16)
			{shoot.friction_pid.speed_ref[0] = FRICTION_SPEED_15;}
			else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit>16&judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<19)
			{shoot.friction_pid.speed_ref[0] = FRICTION_SPEED_18;}
			else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit>19&judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit<31)
			{shoot.friction_pid.speed_ref[0] = FRICTION_SPEED_30;}
			else
			{shoot.friction_pid.speed_ref[0] = FRICTION_SPEED_15;}               
		}
		last_speed_limit=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit;     //数据更新
}

void heat_switch()
{
											//+\0.002*shoot.cooling_ratio
	//剩余发弹量=（热量上限-热量值）/10      ————一个弹丸的热量为10
  shoot.remain_bullets=(judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-\
										 judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat)/10;    //剩余发射量
  shoot.cooling_ratio=judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;                  //冷却速率                                                          //冷却速度
      
		//计算射频，但是实际上没什么用，只不过是一种计算方式
		if(judge_rece_mesg.game_robot_state.robot_level==1)//level_冷却
			shoot.shoot_frequency=10;						
		else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
			shoot.shoot_frequency=14;
		else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
			shoot.shoot_frequency=14;
		else
			shoot.shoot_frequency=10;
				
#if STANDARD == 3				
				if(shoot.bulletspead_level==1)
				{
					shoot.shoot_frequency=1.5*shoot.shoot_frequency;
				}
//				shoot_frequency=24; 
#elif STANDARD == 4
				if(shoot.bulletspead_level==1)
				{
					shoot.shoot_frequency=15; 
				}
//				shoot_frequency=20; 
#elif STANDARD == 5			
				if(shoot.bulletspead_level==1)
				{
					shoot.shoot_frequency=1.5*shoot.shoot_frequency;
				}
#endif							
}
#endif

#if SHOOT_TYPE == 3//发射热量限制
void heat0_limit(void)           //热量限制
{  //由于发送的数据为50hz，而且每次进入该操作需要5次循环，因此计算时 公式为：冷却上限-热量值+0.1*冷却值
  shoot.limit_heart0 = judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat+0.002*judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;
	if(shoot.limit_heart0>15)//residual_heat)
    shoot.ctrl_mode=1;
  else
    {
     shoot.ctrl_mode=0;
    }
}//无人机
#elif SHOOT_TYPE == 6
//哨兵
#elif SHOOT_TYPE == 7
void heat0_limit(void)
{
		shoot.max_heart0 = 240;
		shoot.cooling_ratio = 80;
		shoot.limit_heart0 = shoot.max_heart0 - judge_rece_mesg.power_heat_data.shooter_id2_17mm_cooling_heat + 0.002 * shoot.cooling_ratio;
		shoot.limit_heart1 = shoot.max_heart0 - judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat + 0.002 * shoot.cooling_ratio;
		shoot.total_speed = 0;

		if ( (shoot.limit_heart0 < 35)|(shoot.limit_heart1 < 35) )
		{
				shoot.poke_current[0]=0;
				shoot.poke_current[1]=0;
				shoot.ctrl_mode = SHOT_DISABLE;//0
		}else
		{
				shoot.ctrl_mode = REMOTE_CTRL_SHOT;//1
		}
}
#endif


#if SHOOT_TYPE == 3//拨盘 trigger poke
void shoot_bullet_handle(void)
{	
	static u8  press_l_flag;
	shoot.single_angle=60*36.109f;
	
	if(shoot.fric_wheel_run==1&&
					 shoot.poke_run==1&&
					 shoot.ctrl_mode==1)
	{ 
		fric_run_time++;
		if(fric_run_time<20&&shoot.remain_bullets>4)           //刚开始发弹且剩余量很多时可以提高射频     
		{shoot.shoot_frequency=shoot.shoot_frequency*1.2;}	
	}
	else
	{
		fric_run_time=0;
		if(shoot.shoot_frequency!=0)
		{pid_trigger_angle[0].set=pid_trigger_angle[0].get;}//松手立即停     
		shoot.shoot_frequency=0;
	}	

	if(shoot.shoot_frequency!=0)                          //如果射频不为0（射频用来设定热量限制）
	{
		//计算2ms一次  time_tick 1ms更新一次
			 if(shoot.shoot_frequency>20)                   //高射速时，计算射击时间
			 {
				 shoot.will_time_shoot=(shoot.remain_bullets-4)*1000/shoot.shoot_frequency;             //没有裁判系统就读不到剩余弹量，就没法发射
			 }
			 else if(shoot.shoot_frequency>13)
			 {
				 shoot.will_time_shoot=(shoot.remain_bullets-3.5)*1000/shoot.shoot_frequency;
			 }
			 else
			 {
				 shoot.will_time_shoot=(shoot.remain_bullets-1.5)*1000/shoot.shoot_frequency;
			 }
	}
	
	if(gimbal_data.ctrl_mode!=GIMBAL_AUTO_SMALL_BUFF&&
		 gimbal_data.ctrl_mode!=GIMBAL_AUTO_BIG_BUFF)//正常模式   
	  {//热量限制
		  if(shoot.will_time_shoot>0&&
				 shoot.fric_wheel_run==1&&
							 shoot.poke_run==1&&				
						  shoot.ctrl_mode==1)
		{
			pid_trigger_angle[0].set=pid_trigger_angle[0].get-shoot.shoot_frequency*45*36/500;		//一秒shoot_frequency发，一发拨盘转45°，减速比是1：36。每两毫秒执行一次故除以500。	
			shoot.poke_current[0]=pid_double_loop_cal(&pid_trigger_angle[0],&pid_trigger_speed[0],
																								 pid_trigger_angle[0].set,
																								 general_poke.left_poke.ecd_angle,
																								 &pid_trigger_speed[0].get,
																								 general_poke.left_poke.filter_rate,0);
		}
		else
		{
			 pid_trigger_angle[0].set = pid_trigger_angle[0].get;//松手必须停
			shoot.poke_current[0]=0;
		}		
	}
	else//打幅单发模式
	{
		if(shoot.fric_wheel_run==1)
		{
			if(press_l_state_switch==1)
			{
				press_l_flag=1;
			}
			//每一个标志位代表一个
			if(press_l_flag==1)
			{ 
					pid_trigger_angle_buf.set-=shoot.single_angle;
					press_l_flag=0;
			}
			else if(press_l_flag==0&&fabs(pid_trigger_angle_buf.set-pid_trigger_angle_buf.get)<10)
			{
					pid_trigger_angle_buf.set=pid_trigger_angle_buf.get;		
			}
			shoot.poke_current[0]=pid_double_loop_cal(&pid_trigger_angle_buf,&pid_trigger_speed_buf,
													pid_trigger_angle_buf.set,
													general_poke.left_poke.ecd_angle,
													&pid_trigger_speed_buf.set,
													general_poke.left_poke.filter_rate,0);	
		}
	}
}
#elif SHOOT_TYPE == 7 || SHOOT_TYPE == 6
void shoot_bullet_handle(void)
{
	if(shoot.ctrl_mode != SHOT_DISABLE)
	{
		if(shoot.poke_run)
		{
			start_shooting_count++;
			if((start_shooting_count >= 25)&&(abs(general_poke.left_poke.filter_rate) < 20))
			{
				lock_rotor1 = 1;
				start_shooting_count = 0;
			}
			if((start_shooting_count >= 25)&&(abs(general_poke.right_poke.filter_rate) < 20))
			{
				lock_rotor2 = 1;
				start_shooting_count = 0;
			}
			if(lock_rotor1 == 1)
			{
				start_reversal_count1++;
				if(start_reversal_count1 > 20)
				{
					lock_rotor1 = 0;
					start_reversal_count1 = 0;
				}
				shoot.poke_pid.speed_ref[0] = SHOOT_MOTOR_SPEED;
			}
			if(lock_rotor2 == 1)
			{
				start_reversal_count2++;
				if(start_reversal_count2 > 20)
				{
					lock_rotor2 = 0;
					start_reversal_count2 = 0;
				}
				shoot.poke_pid.speed_ref[1] = SHOOT_MOTOR_SPEED;
			}
			if((shoot.fric_wheel_run)&&(lock_rotor1 == 0))
			shoot.poke_pid.speed_ref[0] = -SHOOT_MOTOR_SPEED;
			if(shoot.fric_wheel_run&&(lock_rotor2 == 0))
			shoot.poke_pid.speed_ref[1] = -SHOOT_MOTOR_SPEED;
		}else
		{
			shoot.poke_pid.speed_ref[0] = 0;
			shoot.poke_pid.speed_ref[1] = 0;
			start_shooting_count = 0;//清零正转计时
      start_reversal_count1 = 0;//清零反转计时
			start_reversal_count2 = 0;//清零反转计时
		}
	}else
	{
		shoot.poke_current[0]=0;
		shoot.poke_current[1]=0;
		start_shooting_count = 0;//清零正转计时
    start_reversal_count1 = 0;//清零反转计时
		start_reversal_count2 = 0;//清零反转计时
	}
	shoot.poke_pid.speed_fdb[0] = general_poke.left_poke.filter_rate;
	shoot.poke_pid.speed_fdb[1] = general_poke.right_poke.filter_rate;
	
	shoot.poke_current[0] = pid_calc(&pid_trigger_speed[0],shoot.poke_pid.speed_ref[0],shoot.poke_pid.speed_fdb[0]); 
	shoot.poke_current[1] = pid_calc(&pid_trigger_speed[1],shoot.poke_pid.speed_ref[1],shoot.poke_pid.speed_fdb[1]); 
}
#endif

#if SHOOT_TYPE == 3//摩擦轮 friction
void shoot_friction_handle()
{  
	if(shoot.fric_wheel_run==1)
	{
		pid_rotate[0].set=-shoot.friction_pid.speed_ref[0];
    pid_rotate[1].set= shoot.friction_pid.speed_ref[0];	
	}
	else
	{
    pid_rotate[0].set=0;
    pid_rotate[1].set=0;
  }
	pid_rotate[0].get = general_friction.left_motor1.filter_rate;
  pid_rotate[1].get = general_friction.right_motor1.filter_rate;

  shoot.fric_current[0]=pid_calc(& pid_rotate[0],pid_rotate[0].get, pid_rotate[0].set);
  shoot.fric_current[1]=pid_calc(& pid_rotate[1],pid_rotate[1].get, pid_rotate[1].set);
}	

#elif SHOOT_TYPE == 7||SHOOT_TYPE == 6
void shoot_friction_handle(void)
{
	if ( shoot.fric_wheel_run == 1 )			//摩擦轮转动
	{
		shoot.friction_pid.speed_ref[0] = -( FRICTION_SPEED );
		shoot.friction_pid.speed_ref[1] = ( FRICTION_SPEED );
		shoot.friction_pid.speed_ref[2] = ( FRICTION_SPEED );
		shoot.friction_pid.speed_ref[3] = -( FRICTION_SPEED );
	}
	else
	{
		shoot.friction_pid.speed_ref[0] = 0;
		shoot.friction_pid.speed_ref[1] = 0;
		shoot.friction_pid.speed_ref[2] = 0;
	  shoot.friction_pid.speed_ref[3] = 0;	
	}
	shoot.friction_pid.speed_fdb[0] = general_friction.left_motor1.filter_rate;
	shoot.friction_pid.speed_fdb[1] = general_friction.right_motor1.filter_rate;
	shoot.friction_pid.speed_fdb[2] = general_friction.left_motor2.filter_rate;
	shoot.friction_pid.speed_fdb[3] = general_friction.right_motor2.filter_rate;
	
	for(int i=0;i<4;i++)
	{
    shoot.fric_current[i] = pid_calc(&pid_rotate[i], shoot.friction_pid.speed_fdb[i], shoot.friction_pid.speed_ref[i]);            
	}
}
#endif


#if SHOOT_TYPE == 3//发射参数初始化
void shot_param_init(void)
{
  PID_struct_init(&pid_trigger_angle[0], POSITION_PID, 6000, 1000,5,0.2,15);
	PID_struct_init(&pid_trigger_speed[0],POSITION_PID,19000,10000,300,0.1,4);
	
	PID_struct_init(&pid_trigger_angle_buf,POSITION_PID, 6000, 1000,5,0.6,15);
	PID_struct_init(&pid_trigger_speed_buf,POSITION_PID,19000,10000,300,0.3,4);
	
  PID_struct_init(&pid_rotate[1], POSITION_PID,15500,11500,50,0,0);
  PID_struct_init(&pid_rotate[0], POSITION_PID,15500,11500,50,0,0);

//	shoot.friction_pid.speed_ref[0] =FRICTION_SPEED_15;            //没装裁判系统，因此先进行摩擦轮速度赋值
	
  shoot.ctrl_mode=1;
  shoot.limit_heart0=80;
}
#elif SHOOT_TYPE == 6 || SHOOT_TYPE == 7
void shot_param_init(void)
{

	for(int i=0;i<2;i++)
	{
	PID_struct_init(&pid_trigger_angle[i],
		              POSITION_PID,								
                  25000,
                  25000,
									70,
                  0.5,
                  0);
		
	}
	for(int i=0;i<4;i++)
	{
		PID_struct_init(&pid_rotate[i], POSITION_PID,15500,11500,50,0,0);
	}

	shoot.ctrl_mode = REMOTE_CTRL_SHOT;
	shoot.limit_heart0=120;
}

#endif
