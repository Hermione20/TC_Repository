#include "42mm_shoot_task.h"




_42mm_shoot_t _42mm_shoot;

#define RIGHT_FRICTION_POLARITY -1
#define LEFT_FRICTION_POLARITY  1
/******************** VARIABVLE ******************/
uint16_t frictionSpeed_42=0;		//42mm弹速
u8 over_heat = 0;                   //超热量标志位
int lock_cnt = 0;
int reverse_cnt = 0;
int set_cnt = 0;
u8 up_poke_init_flag = 0;
u8 press_l_first_in = 0;

void shoot_task(void)
{
    Shoot_42mm_speed_Select(0);
    heat_limit_42mm(RC_CtrlData.Key_Flag.Key_R_Flag);
    shoot_friction_handle_42();

}

static void Shoot_42mm_speed_Select(uint16_t test_frictionSpeed_42) // 42mm弹速//读裁判系统弹速上限进行选择
{
    if (test_frictionSpeed_42 == 0)
    {
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


void heat_limit_42mm(u8 ifignore)
{
    float residue_heart;
    residue_heart=(judge_rece_mesg.game_robot_state.shooter_id1_42mm_cooling_limit           //获取相关的资料
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

void shoot_friction_handle_42(void)
{
    //friction_mode_select
    if (RC_CtrlData.inputmode != STOP)
	{
        if((RC_CtrlData.RemoteSwitch.s3to1 || RC_CtrlData.Key_Flag.Key_C_TFlag))
        {
            _42mm_shoot.friction_state = START;
            if (fabs(general_friction.left_motor1.rate_rpm)<30||fabs(general_friction.right_motor1.rate_rpm)<30)
            {
                lock_cnt++;
                if(lock_cnt == 100)
                {
                     _42mm_shoot.friction_state = LOCK;
                     lock_cnt = 0;
                }
            }else
            {
                _42mm_shoot.friction_state = NORMAL;
                lock_cnt = 0;
            }

            if (_42mm_shoot.friction_state == LOCK)
            {
                _42mm_shoot.friction_state = BACK;
            }
            if (_42mm_shoot.friction_state == BACK)
            {
                reverse_cnt++;
                if (reverse_cnt == 30)
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

    //frction_motor_set
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
    default:
    {
        _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref = 0;
        _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref = 0;
    }
        break;
    }

    _42mm_shoot.shoot_ref_and_fdb.left_friction_speed_fdb = general_friction.left_motor1.rate_rpm;
    _42mm_shoot.shoot_ref_and_fdb.right_friction_speed_fdb = general_friction.right_motor1.rate_rpm;

    _42mm_shoot.shoot_ref_and_fdb.left_friction_motor_input = pid_calc(&_42mm_shoot.pid_left_friction_speed,_42mm_shoot.shoot_ref_and_fdb.left_friction_speed_fdb,_42mm_shoot.shoot_ref_and_fdb.left_friction_speed_ref);
    _42mm_shoot.shoot_ref_and_fdb.right_friction_motor_input = pid_calc(&_42mm_shoot.pid_right_friction_speed,_42mm_shoot.shoot_ref_and_fdb.right_friction_speed_fdb,_42mm_shoot.shoot_ref_and_fdb.right_friction_speed_ref);

}

void shoot_bullet_handle_42(void)
{
    _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb = general_poke.right_poke.ecd_angle/36.0f;
    _42mm_shoot.shoot_ref_and_fdb.up_poke_speed_fdb = general_poke.right_poke.rate_rpm;

    _42mm_shoot.shoot_ref_and_fdb.down_poke_speed_fdb = general_poke.left_poke.rate_rpm;

    if (RC_CtrlData.mouse.press_l == 1 || RC_CtrlData.RemoteSwitch.trigger == 1)
    {
        // 可以射击并且是吊塔模式时，或者射击模式为正常射击时，进入这个模式
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
        if(up_poke_init_flag == 0)
        {
            _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref = _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_fdb;
            _42mm_shoot.pid_downpoke_speed.max_out = 6000;
            up_poke_init_flag = 1;
        }
        if(_42mm_shoot.shoot_flag)
        {
            _42mm_shoot.shoot_ref_and_fdb.up_poke_angle_ref+=ONE_POKE_ANGLE_42;
            _42mm_shoot.pid_uppoke_angle.iout = 0;
        }
        if(_42mm_shoot.shoot_flag)
			_42mm_shoot.pid_downpoke_speed.max_out=POKE_MAX_OUT;
		else
		{
			if(_42mm_shoot.pid_downpoke_speed.out<-4200 && abs(general_poke.left_poke.rate_rpm)<20)
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
                                                                                )
    }else
    {
        up_poke_init_flag = 0;
        _42mm_shoot.shoot_ref_and_fdb.up_poke_motor_input = 0;
         _42mm_shoot.shoot_ref_and_fdb.down_poke_motor_input = 0;
    }
}