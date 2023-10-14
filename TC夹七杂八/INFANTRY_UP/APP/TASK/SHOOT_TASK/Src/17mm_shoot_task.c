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
		//�����������ı䲻ͬ��ģʽ 
  if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==30)          //����������30Ϊ�ٶ�ģʽ
	{shoot.shoot_mode_selection=ZOOM;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==50)    //��������Ϊ15��������ȴ����Ϊ50
	{shoot.shoot_mode_selection=COOLING;}
  else if(judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit==15&judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit==150)   //��������Ϊ15��������ȴ����Ϊ150
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
		last_speed_limit=judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit;     //���ݸ���
}

void heat_switch()
{
	int cooling_rate;
	//ʣ�෢����=����������-����ֵ��/10      ��������һ�����������Ϊ10
  remaining_bullets=(judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat+0.002*cooling_rate)/10;    //ʣ�෢����
  cooling_rate=judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;                  //��ȴ����                                                          //��ȴ�ٶ�
      
		//������Ƶ������ʵ����ûʲô�ã�ֻ������һ�ּ��㷽ʽ
		if(judge_rece_mesg.game_robot_state.robot_level==1)//level_��ȴ
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