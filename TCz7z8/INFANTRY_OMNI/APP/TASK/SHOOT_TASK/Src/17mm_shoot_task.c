#include "17mm_shoot_task.h"

/* Variables_definination-----------------------------------------------------------------------------------------------*/
  shoot_t shoot;
	pid_t pid_trigger_angle[4] ={0};
	pid_t pid_trigger_angle_buf={0};
	pid_t pid_trigger_speed_buf={0};
/*----------------------------------------------------------------------------------------------------------------------*/

#if SHOOT_TYPE == 3//���������ʼ��
void shot_param_init()
{

  PID_struct_init(&pid_trigger_angle[0], POSITION_PID, 6000, 1000,5,0.2,15);
	PID_struct_init(&pid_trigger_speed[0],POSITION_PID,19000,10000,300,0.1,4);
	
	PID_struct_init(&pid_trigger_angle_buf,POSITION_PID, 4000 , 0    ,  130, 5  , 10);
	PID_struct_init(&pid_trigger_speed_buf,POSITION_PID,12000 , 5500 ,  30 , 0  , 0 );
	
  PID_struct_init(&pid_rotate[1], POSITION_PID,15500,11500,50,0,0);
  PID_struct_init(&pid_rotate[0], POSITION_PID,15500,11500,50,0,0);

//	shoot.friction_pid.speed_ref[0] =FRICTION_SPEED_15;            //ûװ����ϵͳ������Ƚ���Ħ�����ٶȸ�ֵ
	
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


#if SHOOT_TYPE == 3//��������ģʽѡ��
void performance_select()
{
		shoot_mode_switch();
		speed_switch();
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
											//+\0.002*shoot.cooling_ratio
	//ʣ�෢����=����������-����ֵ��/10      ��������һ�����������Ϊ10
  shoot.remain_bullets=(judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-\
										 judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat)/10;    //ʣ�෢����
  shoot.cooling_ratio=judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;      //��ȴ����
      
	//������Ƶ
		if(judge_rece_mesg.game_robot_state.robot_level==1)//level_��ȴ
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

#if SHOOT_TYPE == 3//������������
void heat_shoot_frequency_limit()//������Ƶ���Ʋ���
{

		heat_switch();//����ģʽѡ��
		bullets_spilling();//�õ�

	if(shoot.shoot_frequency!=0)                          //�����Ƶ��Ϊ0����Ƶ�����趨�������ƣ�
	{
		//����2msһ��  time_tick 1ms����һ��
			 if(shoot.shoot_frequency>20)                   //������ʱ���������ʱ��
			 {
				 shoot.will_time_shoot=(shoot.remain_bullets-4)*1000/shoot.shoot_frequency;             //û�в���ϵͳ�Ͷ�����ʣ�൯������û������
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
}	
	
void bullets_spilling()//������Ƶ���Ʋ���
{
	static 	int fric_run_time=0;
	
	if(shoot.fric_wheel_run==1&&
					 shoot.poke_run==1&&
					 shoot.ctrl_mode==1)
	{ 
		fric_run_time++;
		if(fric_run_time<20&&shoot.remain_bullets>4)           //�տ�ʼ������ʣ�����ܶ�ʱ���������Ƶ     
		{shoot.shoot_frequency=shoot.shoot_frequency*1.2;}	
	}
	else
	{
		fric_run_time=0;
		if(shoot.shoot_frequency!=0)
		{pid_trigger_angle[0].set=pid_trigger_angle[0].get;}//��������ͣ     
		shoot.shoot_frequency=0;
	}	

}
void heat0_limit(void)           //��������
{  //���ڷ��͵�����Ϊ50hz������ÿ�ν���ò�����Ҫ5��ѭ������˼���ʱ ��ʽΪ����ȴ����-����ֵ+0.1*��ȴֵ
  shoot.limit_heart0 = judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_limit-judge_rece_mesg.power_heat_data.shooter_id1_17mm_cooling_heat+0.002*judge_rece_mesg.game_robot_state.shooter_id1_17mm_cooling_rate;
	if(shoot.limit_heart0>15)//residual_heat)
    shoot.ctrl_mode=1;
  else
    {
     shoot.ctrl_mode=0;
    }
}//���˻�
#elif SHOOT_TYPE == 6
//�ڱ�
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


#if SHOOT_TYPE == 3//���� trigger poke
void shoot_bullet_handle(void)
{	
	static u8  press_l_flag;
	shoot.single_angle=45;

	heat_shoot_frequency_limit();//������Ƶ����

	if(gimbal_data.ctrl_mode!=GIMBAL_AUTO_SMALL_BUFF&&
		 gimbal_data.ctrl_mode!=GIMBAL_AUTO_BIG_BUFF)//����ģʽ   
	  {//��������
		  if(shoot.will_time_shoot>0&&
				 shoot.fric_wheel_run==1&&
							 shoot.poke_run==1&&				
						  shoot.ctrl_mode==1)
		{
			shoot.poke_pid.angle_ref[0]=shoot.poke_pid.angle_fdb[0]-shoot.shoot_frequency*45*36/500;		//һ��shoot_frequency����һ������ת45�㣬���ٱ���1��36��ÿ������ִ��һ�ιʳ���500��	
			
			shoot.poke_pid.angle_fdb[0]=general_poke.poke.ecd_angle;
			shoot.poke_pid.speed_fdb[0]=general_poke.poke.filter_rate;
			shoot.poke_current[0]=pid_double_loop_cal(&pid_trigger_angle[0],&pid_trigger_speed[0],
																								  shoot.poke_pid.angle_ref[0],
																								  shoot.poke_pid.angle_fdb[0],
																								 &shoot.poke_pid.speed_ref[0],
																								  shoot.poke_pid.speed_fdb[0],0);
		}
		else
		{
			  pid_trigger_angle[0].set = pid_trigger_angle[0].get;//���ֱ���ͣ
			 shoot.poke_current[0]=0;
		}		
	}
	else//�������ģʽ
	{
		if(shoot.fric_wheel_run==1)
		{
			
			if(RC_CtrlData.mouse.press_l==1||RC_CtrlData.RemoteSwitch.trigger==1)
			 {if(press_l_flag==0)
				{
				press_l_flag=1;	
				shoot.poke_run=1;	
				}
				else
				{
				shoot.poke_run=0;	
				}
			}
		else
				{press_l_flag=0;}	
			

			if(shoot.poke_run==1)
				shoot.poke_pid.angle_ref[0]-=shoot.single_angle;
				
			
			shoot.poke_pid.angle_fdb[0]=general_poke.poke.ecd_angle/36.109f;
			shoot.poke_pid.speed_fdb[0]=general_poke.poke.rate_rpm;
			shoot.poke_current[0]=pid_double_loop_cal(&pid_trigger_angle_buf,&pid_trigger_speed_buf,
													 shoot.poke_pid.angle_ref[0],
													 shoot.poke_pid.angle_fdb[0],
													&shoot.poke_pid.speed_ref[0],
													 shoot.poke_pid.speed_fdb[0],0);	
		}
	}
}
#elif SHOOT_TYPE == 7 || SHOOT_TYPE == 6
void shoot_bullet_handle(void)
{
	static uint32_t start_shooting_count = 0;//��ת��ʱ
	static uint32_t start_reversal_count1 = 0;//��ת��ʱ
	static uint32_t start_reversal_count2 = 0;
	static uint8_t lock_rotor1 = 0;//��ת��־λ
	static uint8_t lock_rotor2 = 0;
	
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
			start_shooting_count = 0;//������ת��ʱ
      start_reversal_count1 = 0;//���㷴ת��ʱ
			start_reversal_count2 = 0;//���㷴ת��ʱ
		}
	}else
	{
		shoot.poke_current[0]=0;
		shoot.poke_current[1]=0;
		start_shooting_count = 0;//������ת��ʱ
    start_reversal_count1 = 0;//���㷴ת��ʱ
		start_reversal_count2 = 0;//���㷴ת��ʱ
	}
	shoot.poke_pid.speed_fdb[0] = general_poke.left_poke.filter_rate;
	shoot.poke_pid.speed_fdb[1] = general_poke.right_poke.filter_rate;
	
	shoot.poke_current[0] = pid_calc(&pid_trigger_speed[0],shoot.poke_pid.speed_ref[0],shoot.poke_pid.speed_fdb[0]); 
	shoot.poke_current[1] = pid_calc(&pid_trigger_speed[1],shoot.poke_pid.speed_ref[1],shoot.poke_pid.speed_fdb[1]); 
}
#endif

#if SHOOT_TYPE == 3//Ħ���� friction
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
	pid_rotate[0].get = general_friction.left_motor.filter_rate;
  pid_rotate[1].get = general_friction.right_motor.filter_rate;

  shoot.fric_current[0]=pid_calc(& pid_rotate[0],pid_rotate[0].get, pid_rotate[0].set);
  shoot.fric_current[1]=pid_calc(& pid_rotate[1],pid_rotate[1].get, pid_rotate[1].set);
}	

#elif SHOOT_TYPE == 7||SHOOT_TYPE == 6
void shoot_friction_handle(void)
{
	if ( shoot.fric_wheel_run == 1 )			//Ħ����ת��
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
	shoot.friction_pid.speed_fdb[0] = general_friction.left_up_motor.filter_rate;
	shoot.friction_pid.speed_fdb[1] = general_friction.right_up_motor.filter_rate;
	shoot.friction_pid.speed_fdb[2] = general_friction.left_down_motor.filter_rate;
	shoot.friction_pid.speed_fdb[3] = general_friction.right_down_motor.filter_rate;
	
	for(int i=0;i<4;i++)
	{
    shoot.fric_current[i] = pid_calc(&pid_rotate[i], shoot.friction_pid.speed_fdb[i], shoot.friction_pid.speed_ref[i]);            
	}
}
#endif

/**
************************************************************************************************************************
* @Name     : shoot_state_mode_switch
* @brief    : ң����/���� ����״̬����
* @retval   : 
* @Note     : 
************************************************************************************************************************
**/
void shoot_state_mode_switch()
{
	 /****************************�������״̬����**********************************************/		
		switch(RC_CtrlData.inputmode)
			{
					case REMOTE_INPUT:
				{
					if(RC_CtrlData.RemoteSwitch.s3to1)
							shoot.fric_wheel_run=1;
					 else
							shoot.fric_wheel_run=0;
				}break;
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
				 }break;

					default:
					break;		
			}
}

/**
************************************************************************************************************************
* @Name     : shoot_task
* @brief    : ���亯��������
* @retval   : 
* @Note     : 
************************************************************************************************************************
**/
void shoot_task()
{

	shoot_state_mode_switch();
#if SHOOT_TYPE == 3
	performance_select();
#endif

	shoot_bullet_handle();
	shoot_friction_handle();

#if SHOOT_TYPE != 6//�ɻ�û����
	heat0_limit();
#endif
}
