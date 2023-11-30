#include "42mm_shoot_task.h"

/**
  ******************************************************************************
  * @file   42mm_shoot_task.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   42mm���跢��ģ�飬��ģ�鶨����Ӣ�۶������̹���˫Ħ���ֵ�
							����ơ���������λ��λ��Դ�ļ��ϲ���ע�⣡����Ӣ�۷�����
							���ٶȷ���ѡ��rate_rpm��filter_rate��
	
	* @notice  ��ģ�����������23����Ӣ�ۻ�е��������δ���Ĵ���ά�����뿪��
							��Աά��ģ��Ķ����ԣ�ά��ͨ���ԣ���ֹ�����ڸò��ֵ��߼������
						 д�����ģ���ڣ���ģ���������Ŀ��ƣ��ο�����ĸ�ֵ���Ʋ�
						 ģʽѡ��
						 
	* @notice  ����ģ��ĵ������Ʋ���control_task���Ƽ���̨����Ƶ��Ϊ2ms
						 �Ƽ���̨�������Ƶ��Ϊ2ms����controltask��shoot_task
						 ��control_task_Init�����shoot_param_init
						 
	*	@introduction ��ģ�����״̬���ķ�ʽ������ַ����״̬�������ź�
									��������ģʽ���л���ѡ������mode_switch_tasks��ȫģ���
									����_42mm_shoot�ṹ�������ģ����Զ���Ħ���ֵļ��ԣ�����
									�������������
									#define RIGHT_FRICTION_POLARITY -1
									
 ===============================================================================
 **/


_42mm_shoot_t _42mm_shoot;


 /**
  ******************************************************************************
																			��������
	Ħ����ת�����ã����ٶ�Ӧ10��12��14��16��
	Ӣ���²���ת��
	Ӣ���²�����������
	42mm���������ϲ��̽Ƕ�
	Ħ������ת��������
	 =============================================================================
 **/
#define FRICTION_SPEED_10 2000
#define FRICTION_SPEED_12 2100
#define FRICTION_SPEED_14 2575
#define FRICTION_SPEED_16 3200

#define POKE_SPEED -200			//Ӣ���²���ת��
#define POKE_MAX_OUT 6000		//Ӣ���²�����������
#define ONE_POKE_ANGLE_42 90.0f	//42mm���������ϲ��̽Ƕ�
//Ħ������ת��������
#define RIGHT_FRICTION_POLARITY -1
#define LEFT_FRICTION_POLARITY  1



/******************** VARIABVLE ******************/
uint16_t frictionSpeed_42=0;		//42mm����
u8 over_heat = 0;                   //��������־λ
int lock_cnt = 0;
int reverse_cnt = 0;
int set_cnt = 0;
u8 poke_init_flag = 0;
u8 press_l_first_in = 0;

 /**
  ******************************************************************************
																��̨�ṹ���ʼ��
		pid��������
		
	 =============================================================================
 **/
void shoot_param_init(void)
{
		//�ṹ���ڴ�����
    memset(&_42mm_shoot,0,sizeof(_42mm_shoot_t));

    //42mmĦ����
  PID_struct_init(&_42mm_shoot.pid_right_friction_speed, POSITION_PID,10000,1000,  70 , 0.01f ,100);//����Ħ����
  PID_struct_init(&_42mm_shoot.pid_left_friction_speed, POSITION_PID,10000,1000,  72 , 0.01f ,10);

  //42mm����
    PID_struct_init(&_42mm_shoot.pid_downpoke_speed,   POSITION_PID, 6000 , 13000,  5  , 0.5, 0 );//�²����ٶȻ�
	PID_struct_init(&_42mm_shoot.pid_uppoke_angle, POSITION_PID, 2000 , 0    ,  120, 5  , 10); //��������
    PID_struct_init(&_42mm_shoot.pid_uppoke_speed, POSITION_PID, 9900 , 5500 ,  20 , 0  , 0 );
}


 /**
  ******************************************************************************
																��̨�ܿ�������		
	 =============================================================================
 **/
void shoot_task(void)
{
    Shoot_42mm_speed_Select(0);//�����Զ�ѡ��
    heat_limit_42mm(RC_CtrlData.Key_Flag.Key_Z_Flag);//��������
    shoot_friction_handle_42();//Ħ���ֿ�������
    shoot_bullet_handle_42();//���̿�������

}


 /**
  ******************************************************************************
																�����Զ�ѡ��
		������ڲ���Ϊ����Ħ�����ٶȣ���Ϊ0����Ĭ�ϲ���ϵͳѡ�񣬷�ִ֮�в����ڵ��ٶ�
	 =============================================================================
 **/
static void Shoot_42mm_speed_Select(uint16_t test_frictionSpeed_42) // 42mm����
{
    if (test_frictionSpeed_42 == 0)
    {
			//������ϵͳ�������޽���ѡ��
        if (judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit == 10)
            frictionSpeed_42 = FRICTION_SPEED_10;
        else if (judge_rece_mesg.game_robot_state.shooter_id1_42mm_speed_limit == 16)
            frictionSpeed_42 = FRICTION_SPEED_16;
        else
            frictionSpeed_42 = FRICTION_SPEED_10;
    }
    else
    {
        frictionSpeed_42 = test_frictionSpeed_42;
    }
}




 /**
  ******************************************************************************
																��������		
		������ڲ���Ϊ�Ƿ����������־��Ϊ1���ԣ�Ϊ0����
	 =============================================================================
 **/
void heat_limit_42mm(u8 ifignore)
{
    float residue_heart;//ʣ������
    residue_heart=(judge_rece_mesg.game_robot_state.shooter_id1_42mm_cooling_limit           //ͨ������ϵͳ����ʣ������
                       -judge_rece_mesg.power_heat_data.shooter_id1_42mm_cooling_heat);
    if (ifignore)
    {
        over_heat = 0;
    }
    else
    {
        if (residue_heart >= 100)
            over_heat = 0;
        else
            over_heat = 1;
    }
}


 /**
  ******************************************************************************
																Ħ���ֿ�������	
	Ħ���������ں���ת����뷴ת����
	 =============================================================================
 **/
void shoot_friction_handle_42(void)
{
    //Ħ����״̬�ж���ѡ��
    if (RC_CtrlData.inputmode != STOP)//�ǹؿ�
	{
        if((RC_CtrlData.RemoteSwitch.s3to1 || RC_CtrlData.Key_Flag.Key_C_TFlag))//����Ħ��������
        {
					 if(_42mm_shoot.friction_state!=BACK)
					 {
						 _42mm_shoot.friction_state = START;
					 }
            
            if (general_friction.right_motor.rate_rpm>(30*RIGHT_FRICTION_POLARITY)||general_friction.left_motor.rate_rpm<(30*LEFT_FRICTION_POLARITY))//�ж�Ħ����תûת����
            {
							//ûת����
                lock_cnt++;
                if(lock_cnt == 500)//���������500���ж�Ħ���ֶ�ת
                {
                     _42mm_shoot.friction_state = LOCK;
                     lock_cnt = 0;//���������
                }
            }else
            {		//ת�����ˣ���Ϊ��ͨ
                _42mm_shoot.friction_state = NORMAL;
                lock_cnt = 0;//���������
            }

            if (_42mm_shoot.friction_state == LOCK)//����ת��˵�����迨��Ħ�����ڣ���֯��ת
            {
                _42mm_shoot.friction_state = BACK;
            }
            if (_42mm_shoot.friction_state == BACK)
            {
                reverse_cnt++;//��תʱ�����
                if (reverse_cnt == 100)//��תһ��ʱ���ָ�Ħ��������
                {
                    _42mm_shoot.friction_state = START;
                    reverse_cnt = 0;
                }
                
            }           
            
        }else
        {
             _42mm_shoot.friction_state = Stop;
        }
    }else
    {
        _42mm_shoot.friction_state = Stop;
    }

    //����Ħ����״̬��ʼִ��Ħ����
    switch (_42mm_shoot.friction_state)
    {
    case START:
     {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = RIGHT_FRICTION_POLARITY*frictionSpeed_42;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = LEFT_FRICTION_POLARITY*frictionSpeed_42;
     }
        break;
    case NORMAL:
    {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = RIGHT_FRICTION_POLARITY*frictionSpeed_42;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = LEFT_FRICTION_POLARITY*frictionSpeed_42;
    }
         break;
    case BACK:
    {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = -RIGHT_FRICTION_POLARITY*frictionSpeed_42;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = -LEFT_FRICTION_POLARITY*frictionSpeed_42;
    }
				break;
    default:
    {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = 0;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = 0;
    }
        break;
    }
		//����Ħ���ַ���
    _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_fdb = general_friction.left_motor.rate_rpm;
    _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_fdb = general_friction.right_motor.rate_rpm;
		//����Ħ����pid���
    _42mm_shoot.shoot_ref_and_fdb.left_friction_motor_input = pid_calc(&_42mm_shoot.pid_left_friction_speed,_42mm_shoot.shoot_ref_and_fdb.left_friction_speed_fdb,_42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref);
    _42mm_shoot.shoot_ref_and_fdb.right_friction_motor_input = pid_calc(&_42mm_shoot.pid_right_friction_speed,_42mm_shoot.shoot_ref_and_fdb.right_friction_speed_fdb,_42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref);

}


 /**
  ******************************************************************************
																���̿�������		
		�����߼����ϲ��̲��ýǶȻ��������²���Ϊ���ֵ�·�����������ٶȻ�����
							�������ƶ�ת�����������ջ����
	 =============================================================================
 **/
void shoot_bullet_handle_42(void)
{
		//������ֵ
    _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb = general_poke.up_poke.ecd_angle/36.0f;  //2006������ٱ�1��36
    _42mm_shoot.shoot_ref_and_fdb.up_poke_speed_fdb = general_poke.up_poke.rate_rpm;

    _42mm_shoot.shoot_ref_and_fdb.down_poke_speed_fdb = general_poke.down_poke.rate_rpm;
		
	//�����ְ���߼�Ϊ��������������������һ������ź�
    if (RC_CtrlData.mouse.press_l == 1 || RC_CtrlData.RemoteSwitch.trigger == 1)//�������
    {
        if (press_l_first_in == 0)
        {
             press_l_first_in = 1;
             _42mm_shoot.shoot_flag = 1;
        }
        else
        {
             _42mm_shoot.shoot_flag = 0;
        }
    }
    else
    {
        press_l_first_in = 0;
    }
		
    if(_42mm_shoot.friction_state == NORMAL&&over_heat==0)
    {
			
			 _42mm_shoot.shoot_ref_and_fdb.down_poke_speed_ref = POKE_SPEED;
        if(poke_init_flag == 0)
        {
					//���̲�����ʼ��
            _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref = _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb;
            _42mm_shoot.pid_downpoke_speed.max_out = 6000;
            poke_init_flag = 1;
        }
        if(_42mm_shoot.shoot_flag)
        {
					//������䣬�ϲ��̸����ۼӣ���������
            _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref+=ONE_POKE_ANGLE_42;
            _42mm_shoot.pid_uppoke_angle.iout = 0;
        }
        if(_42mm_shoot.shoot_flag)
			_42mm_shoot.pid_downpoke_speed.max_out=POKE_MAX_OUT;
		else
		{
			//��ⵯ·�Ƿ������������򽵵͵��������2000
			if(_42mm_shoot.pid_downpoke_speed.out<-4200 && abs(general_poke.down_poke.rate_rpm)<20)
				set_cnt++;
			else
				set_cnt = 0;
			if(set_cnt == 20)
				_42mm_shoot.pid_downpoke_speed.max_out=2000;
		}

        _42mm_shoot.shoot_ref_and_fdb.up_poke_motor_input = pid_double_loop_cal(&_42mm_shoot.pid_uppoke_angle,
                                                                                &_42mm_shoot.pid_uppoke_speed,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb,
                                                                                &_42mm_shoot.shoot_ref_and_fdb.up_poke_speed_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_speed_fdb,
                                                                                0);
        _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input = pid_calc(&_42mm_shoot.pid_downpoke_speed,_42mm_shoot.shoot_ref_and_fdb.down_poke_speed_fdb,_42mm_shoot.shoot_ref_and_fdb.down_poke_speed_ref);
    }else if(_42mm_shoot.friction_state == BACK)
		{
			_42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref-=0.5f;
			_42mm_shoot.shoot_ref_and_fdb.up_poke_motor_input = pid_double_loop_cal(&_42mm_shoot.pid_uppoke_angle,
                                                                                &_42mm_shoot.pid_uppoke_speed,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb,
                                                                                &_42mm_shoot.shoot_ref_and_fdb.up_poke_speed_ref,
                                                                                _42mm_shoot.shoot_ref_and_fdb.up_poke_speed_fdb,
                                                                                0);
		}else
    {
        poke_init_flag = 0;
        _42mm_shoot.shoot_ref_and_fdb.up_poke_motor_input = 0;
         _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input = 0;
    }
}

