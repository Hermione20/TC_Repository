#include "17mm_shoot_task.h"

/* Variables_definination-----------------------------------------------------------------------------------------------*/
shoot_t shoot;

int remaining_bullets;
float shoot_frequency;
int bulletspead_level;
/*----------------------------------------------------------------------------------------------------------------------*/

void shoot_task()
{
		performance_select();



}

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
	int cooling_rate;
	//剩余发弹量=（热量上限-热量值）/10      ————一个弹丸的热量为10
  remaining_bullets=(judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat+0.002*cooling_rate)/10;    //剩余发射量
  cooling_rate=judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;                  //冷却速率                                                          //冷却速度
      
		//计算射频，但是实际上没什么用，只不过是一种计算方式
		if(judge_rece_mesg.game_robot_state.robot_level==1)//level_冷却
			shoot_frequency=10;						
		else if(judge_rece_mesg.game_robot_state.robot_level==2)//level_2
			shoot_frequency=14;
		else if(judge_rece_mesg.game_robot_state.robot_level==3)//level_3
			shoot_frequency=14;
		else
			shoot_frequency=10;
				
#if STANDARD == 3				
				if(bulletspead_level==1)
				{
					shoot_frequency=1.5*shoot_frequency;
				}
//				shoot_frequency=24; 
#elif STANDARD == 4
				if(bulletspead_level==1)
				{
					shoot_frequency=15; 
				}
//				shoot_frequency=20; 
#elif STANDARD == 5			
				if(bulletspead_level==1)
				{
					shoot_frequency=1.5*shoot_frequency;
				}
#endif				
				


}

static void shoot_bullet_handle(void)
{


}