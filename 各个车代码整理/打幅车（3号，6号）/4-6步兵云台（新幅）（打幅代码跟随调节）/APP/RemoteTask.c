#include <stm32f4xx.h>
#include "main.h"
int w=0;

RC_Ctl_t RC_CtrlData;
ChassisSpeed_Ref_t ChassisSpeedRef;
Gimbal_Ref_t GimbalRef;
Chassis_angle_t Chassis_angle;
CHASSIS_1234_t CHASSIS_1234;
Ramp_t FBVoltageRamp ;
FrictionWheelState_e friction_wheel_state = FRICTION_WHEEL_OFF;
static RemoteSwitch_t switch1;   //ң������ದ��
static volatile Shoot_State_e shootState = NOSHOOTING;
static volatile Shooting_State_e Shooting_State = NORMAL_SHOOTING;//�������ģʽ
static InputMode_e inputmode = STOP;//REMOTE_INPUT;   //����ģʽ�趨
uint16_t FBSpeed;
RampGen_t frictionRamp = RAMP_GEN_DAFAULT;  //Ħ����б��
RampGen_t frictionRamp_Off = RAMP_GEN_DAFAULT;  //Ħ����б��
RampGen_t LRSpeedRamp = RAMP_GEN_DAFAULT;   //mouse�����ƶ�б��
RampGen_t FBSpeedRamp = RAMP_GEN_DAFAULT;   //mouseǰ���ƶ�б��
RampGen_t chassisRamp = RAMP_GEN_DAFAULT;

ramp_second_t FBSpeedRampSecond = RAMP_SECOND_GEN_DAFAULT;   //mouseǰ���ƶ�б��
int press_l_state_switch;
int16_t closeDelayCount = 0;   //�Ҽ��ر�Ħ����3s��ʱ����
u8 frictionSpeedmode;
uint16_t FRICTION_WHEEL_DUTY = FRICTION_WHEEL_MAX_DUTY;
extern power_limit_t power_limit;
extern voltage_pid_t voltage_pid;
extern chassis_speed_mode_e chassis_speed_mode;

extern chassis_t chassis;
int control_stop_flag;
chassis_mode_selection_e chassis_mode_selection;
Shoot_Limit_Mode_e Shoot_Limit_Mode;

extern Shoot_Aim_Mode_e Shoot_Aim_Mode;

//int16_t current_dir;
u8 get_velocity_flag = 0;
Key_Flag_t Key_Flag;
u8 low_speed_flag = 0;
u8 high_speed_flag = 0;

//ң������ֵ
void GetRemoteSwitchAction(RemoteSwitch_t *sw, uint8_t val)
{
  static uint32_t switch_cnt = 0;
  /* ����״ֵ̬ */
  sw->switch_value_raw = val;
  sw->switch_value_buf[sw->buf_index] = sw->switch_value_raw;
  /* ȡ����ֵ����һ��ֵ */
  sw->switch_value1 = (sw->switch_value_buf[sw->buf_last_index] << 2)|
                      (sw->switch_value_buf[sw->buf_index]);

  /* ���ϵ�״ֵ̬������ */
  sw->buf_end_index = (sw->buf_index + 1)%REMOTE_SWITCH_VALUE_BUF_DEEP;

  /* �ϲ�����ֵ */
  sw->switch_value2 = (sw->switch_value_buf[sw->buf_end_index]<<4)|sw->switch_value1;
  /* �����ж� */
  if(sw->switch_value_buf[sw->buf_index] == sw->switch_value_buf[sw->buf_last_index])
    {
      switch_cnt++;
    }
  else
    {
      switch_cnt = 0;
    }

  if(switch_cnt >= 40)
    {
      sw->switch_long_value = sw->switch_value_buf[sw->buf_index];
    }

  sw->buf_last_index = sw->buf_index;
  sw->buf_index++;
  if(sw->buf_index == REMOTE_SWITCH_VALUE_BUF_DEEP)
    {
      sw->buf_index = 0;
    }
}
//return the state of the remote 0:no action 1:action
uint8_t IsRemoteBeingAction(void)
{
  return (abs(ChassisSpeedRef.forward_back_ref)>=10 || abs(ChassisSpeedRef.left_right_ref)>=10 || fabs(GimbalRef.yaw_speed_ref)>=10 || fabs(GimbalRef.pitch_speed_ref)>=10);
}

//����ģʽ����
void SetInputMode(Remote *rc)
{
  if(rc->s2 == 1)
    {
      inputmode = REMOTE_INPUT;

    }
  else if(rc->s2 == 3)
    {
      inputmode = KEY_MOUSE_INPUT;

    }
  else if(rc->s2 == 2)
    {
      inputmode = STOP;
      rotate_num_ture=0;
    }

}


InputMode_e GetInputMode()
{
  return inputmode;
}

/*********************ң��ģʽ���***********************/
/*
flag�� friction_rotor    0��Ħ����ֹͣ   1��Ħ���ֿ���   2��Ħ���ֹرգ�REF -> 0��
*/
void RemoteShootControl(RemoteSwitch_t *sw, uint8_t val)
{

  GetRemoteSwitchAction(sw, val);
  switch(friction_wheel_state)
    {
    case FRICTION_WHEEL_OFF:
    {
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�ӹرյ�start turning
        {
          SetShootState(NOSHOOTING);
          friction_rotor=0;
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          friction_wheel_state = FRICTION_WHEEL_START_TURNNING;
          LASER_ON();
          OpenDoor;
        }
    }
    break;
    case FRICTION_WHEEL_START_TURNNING:
    {
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�������ͱ��ر�
        {
          LASER_OFF();
          SetShootState(NOSHOOTING);
          friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          friction_rotor=2;
        }

      else
        {
          friction_rotor=1;
          if(frictionRamp.IsOverflow(&frictionRamp))
            {
              friction_wheel_state = FRICTION_WHEEL_ON;
            }

        }
    }
    break;
    case FRICTION_WHEEL_ON:
    {
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)   //�ر�Ħ����
        {
          LASER_OFF();
          friction_rotor=2;
          friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          SetShootState(NOSHOOTING);
          autoshoot_mode =NORMAL_SHOOT;
        }
      else if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)
        {
          CloseDoor;
        }
      else if(sw->switch_value_raw == 2)
        {
          SetShootState(SHOOTING);
					autoshoot_mode = AUTO_SHOOT;   //����������
        }
      else
        {
          SetShootState(NOSHOOTING);
          autoshoot_mode =NORMAL_SHOOT ;  //����������
        }
    }
    break;

    case FRICTION_WHEEL_STOP_TURNNING:
    {
      friction_rotor=2;
      if(frictionRamp.IsOverflow(&frictionRamp))
        {
          friction_rotor=0;
          friction_wheel_state = FRICTION_WHEEL_OFF;
        }
    }
    break;
    }
}



int rotate_num = 0;

/*********************���̳�ң��ģʽС����***********************/

u8 close_rotate_flag = 0;
void Remote_Rotate_Reverse_Control(RemoteSwitch_t *sw, uint8_t val)
{
  GetRemoteSwitchAction(sw, val);
  switch (chassis.ctrl_mode)
    {
    case MANUAL_FOLLOW_GIMBAL:
    {

      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO1)
        {

          chassis.ctrl_mode = CHASSIS_ROTATE;
        }
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_3TO2)
        {
//          chassis.ctrl_mode = CHASSIS_CHANGE_REVERSE;
          GimbalRef.yaw_angle_dynamic_ref +=180;
				
        }
    }
    break;
    case CHASSIS_ROTATE:
    {
      if((sw->switch_value1 == REMOTE_SWITCH_CHANGE_1TO3)&&(close_rotate_flag==0))
        {
          // ������̨��ԽǶȸ�����һ��360�ı���
          rotate_num = GMYawEncoder.ecd_angle/360 ;
          close_rotate_flag = 1;
        }
      if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-((rotate_num+1)*360)<=35)&&(GMYawEncoder.ecd_angle-((rotate_num+1)*360)>=-35))
        {
          chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
          chassis_rotate_flag ^=1;
          chassis.position_ref = (rotate_num+1)*360;
          close_rotate_flag = 0;
        }
      else if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-((rotate_num-1)*360)<=35)&&(GMYawEncoder.ecd_angle-((rotate_num-1)*360)>=-35))
        {
          chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
          chassis_rotate_flag ^=1;
          chassis.position_ref = (rotate_num-1)*360;
          close_rotate_flag = 0;
        }
      else if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-(rotate_num*360)<=35)&&(GMYawEncoder.ecd_angle-(rotate_num*360)>=-35))
        {
          chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
          chassis_rotate_flag ^=1;
          chassis.position_ref = rotate_num*360;
          close_rotate_flag = 0;
        }

    }
    break;
    case CHASSIS_REVERSE:
    {
      if(sw->switch_value1 == REMOTE_SWITCH_CHANGE_2TO3)
        {
          chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
        }

    }
    break;
    default:
      break;
    }

}
int rotate_num_ture=0;
u8 rotate_key_flag = 0;
u32 rotate_change_time = 0;
u8 reverse_key_flag = 0;
u32 reverse_change_time = 0;
u8 separate_key_flag = 0;
u32 separate_change_time = 0;
u8 remove_rotate_moade=0;
int auti_autoshoot_flag,auti_tran_autoshoot_flag;
u8 debug_high,debug_low;
/*********************���̳��������ģʽС����***********************/
void Mouse_Key_Rotate_Reverse_Control(void)
{
    switch (chassis.ctrl_mode)
    {
    case MANUAL_FOLLOW_GIMBAL:
    {
			//��仰��ȫ��������С����
      if((RC_CtrlData.key.v&KEY_CTRL)&&(rotate_key_flag == 0))    //�������Ctrl����key��־λ��0��ô�ͽ���С����ģʽ����key��־λ1
        {

          chassis.ctrl_mode = CHASSIS_ROTATE;
          rotate_key_flag = 1;
					remove_rotate_moade=0;
        }
    }
    break;
		case CHASSIS_SEPARATE:
    {
			//��仰��ȫ��������С����
      if((RC_CtrlData.key.v&KEY_CTRL)&&(rotate_key_flag == 0))    //�������Ctrl����key��־λ��0��ô�ͽ���С����ģʽ����key��־λ1
        {

          chassis.ctrl_mode = CHASSIS_ROTATE;
          rotate_key_flag = 1;
					remove_rotate_moade=1;
        }
    }
    break;
		case CHASSIS_REVERSE:
	  if((RC_CtrlData.key.v&KEY_CTRL)&&(rotate_key_flag == 0))    //�������Ctrl����key��־λ��0��ô�ͽ���С����ģʽ����key��־λ1
    {
				chassis.ctrl_mode = CHASSIS_ROTATE;
				rotate_key_flag = 1;
				remove_rotate_moade=0;
    }
		break;
    case CHASSIS_ROTATE:
    {
			if(remove_rotate_moade==0)
			{
				
				reverse_flag=1;
				//�ر�С���ݣ��Ǿ��ǿ���С����60��ң����ʱ���ſ��Թر�С����
				if((RC_CtrlData.key.v&KEY_CTRL)&&(rotate_key_flag == 0)&&(close_rotate_flag==0))    
					{
						// ������̨��ԽǶȸ�����һ��360�ı���
						rotate_num = GMYawEncoder.ecd_angle/360 ;
						close_rotate_flag = 1;
						rotate_key_flag = 1;
					}
				
				//�����ӻ�ת��ȥ��
				if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-((rotate_num+1)*360)<=35)&&(GMYawEncoder.ecd_angle-((rotate_num+1)*360)>=-35))
					{
						chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
						chassis_rotate_flag ^=1;
						chassis.position_ref = (rotate_num+1)*360;
						close_rotate_flag = 0;
						rotate_num_ture=abs(rotate_num+1);
					}
				else if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-((rotate_num-1)*360)<=35)&&(GMYawEncoder.ecd_angle-((rotate_num-1)*360)>=-35))
					{
						chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
						chassis_rotate_flag ^=1;
						chassis.position_ref = (rotate_num-1)*360;
						close_rotate_flag = 0;
						rotate_num_ture=abs(rotate_num-1);
					}
				else if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-(rotate_num*360)<=35)&&(GMYawEncoder.ecd_angle-(rotate_num*360)>=-35))
					{
						chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
						chassis_rotate_flag ^=1;
						chassis.position_ref = rotate_num*360;
						close_rotate_flag = 0;
						rotate_num_ture=abs(rotate_num);
					}
			}
			if(remove_rotate_moade==1)
			{
				//�ر�С���ݣ��Ǿ��ǿ���С����60��ң����ʱ���ſ��Թر�С����
				if((RC_CtrlData.key.v&KEY_CTRL)&&(rotate_key_flag == 0)&&(close_rotate_flag==0))    
					{
						// ������̨��ԽǶȸ�����һ��360�ı���
						rotate_num = GMYawEncoder.ecd_angle/360 ;
						close_rotate_flag = 1;
						rotate_key_flag = 1;
					}
				
				//�����ӻ�ת��ȥ��
				if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-((rotate_num+1)*360)<=35)&&(GMYawEncoder.ecd_angle-((rotate_num+1)*360)>=-35))
					{
						chassis.ctrl_mode = CHASSIS_SEPARATE;
						chassis_rotate_flag ^=1;
						chassis.position_ref = (rotate_num+1)*360;
						close_rotate_flag = 0;
						rotate_num_ture=abs(rotate_num+1);
					}
				else if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-((rotate_num-1)*360)<=35)&&(GMYawEncoder.ecd_angle-((rotate_num-1)*360)>=-35))
					{
						chassis.ctrl_mode = CHASSIS_SEPARATE;
						chassis_rotate_flag ^=1;
						chassis.position_ref = (rotate_num-1)*360;
						close_rotate_flag = 0;
						rotate_num_ture=abs(rotate_num-1);
					}
				else if((close_rotate_flag==1)&&(GMYawEncoder.ecd_angle-(rotate_num*360)<=35)&&(GMYawEncoder.ecd_angle-(rotate_num*360)>=-35))
					{
						chassis.ctrl_mode = CHASSIS_SEPARATE;
						chassis_rotate_flag ^=1;
						chassis.position_ref = rotate_num*360;
						close_rotate_flag = 0;
						rotate_num_ture=abs(rotate_num);
					}
			}
    }
    break;
    default:
      break;
    }

  if(rotate_key_flag == 1)
    {
      rotate_change_time++;
    }
  if(rotate_change_time>60)
    {
      rotate_key_flag = 0;
      rotate_change_time = 0;
    }

  if(reverse_key_flag == 1)
    {
      reverse_change_time++;
    }
  if(reverse_change_time>30)
    {
      reverse_key_flag = 0;
      reverse_change_time = 0;
    }
}

u8 auto_angle_key_flag = 0;
u32 auto_angle_change_time = 0;
u8 auto_angle_key_W_flag = 0;
u32 auto_angle_W_change_time = 0;
u8 auto_angle_key_S_flag = 0;
u32 auto_angle_S_change_time = 0;
u8 auto_angle_key_A_flag = 0;
u32 auto_angle_A_change_time = 0;
u8 auto_angle_key_D_flag = 0;
u32 auto_angle_D_change_time = 0;



void auto_angle_control(void)
{


}

int buff_shoot_handle;
/********************* �������ģʽ���**********************/
void MouseShootControl(Mouse *mouse,Key *key)
{
  /*************Ħ����״̬�л�**************/
  switch(friction_wheel_state)
    {
    case FRICTION_WHEEL_OFF:
    {

      if(key->last_v == 0 && (key->v &KEY_C))   //�ӹرյ�start turning
        {
          SetShootState(NOSHOOTING);
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          friction_wheel_state = FRICTION_WHEEL_START_TURNNING;
          LASER_ON();
          closeDelayCount = 0;
          friction_rotor=0;
        }
    }
    break;
    case FRICTION_WHEEL_START_TURNNING:
    {
      if(key->v &KEY_C)
        {
          closeDelayCount++;
        }
      else
        {
          closeDelayCount = 0;
        }
      if(closeDelayCount>70)   //�ر�Ħ����
        {
          LASER_OFF();
          friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          SetShootState(NOSHOOTING);
          friction_rotor=2;
        }
      else
        {
          //Ħ���ּ���

          friction_rotor=1;
          if(friction_rotor==1)
            {
              friction_wheel_state = FRICTION_WHEEL_ON;
              frictionSpeedmode=0;
            }
        }
    }
    break;
    case FRICTION_WHEEL_ON: //�������
    {

      if(key->v &KEY_C)
        {
          closeDelayCount++;
        }
      else
        {
          closeDelayCount = 0;
        }

      if(closeDelayCount>70)   //�ر�Ħ����
        {
          LASER_OFF();
          friction_rotor=2;
          friction_wheel_state = FRICTION_WHEEL_STOP_TURNNING;				  //�е��ر�Ħ����״̬
          frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_OFF_TICK_COUNT);
          frictionRamp.ResetCounter(&frictionRamp);
          SetShootState(NOSHOOTING);
        }
      //�������
      if(shot.ctrl_mode==1)
        {
          if(mouse->press_l== 1)  //������������
            {
              SetShootState(SHOOTING);
						}
						else
							
            {
              SetShootState(NOSHOOTING);
            }
				} 

          }
    break;

    case FRICTION_WHEEL_STOP_TURNNING:
    {
      friction_rotor=2;
      if(frictionRamp.IsOverflow(&frictionRamp))
        {
          friction_rotor=0;
          friction_wheel_state = FRICTION_WHEEL_OFF;
        }

    }
    break;
    }


    //������	�����һ��������־λpress_l_state_switch��			
					if(mouse->last_press_l==0&&mouse->press_l==1)
						press_l_state_switch=1;
					else
						press_l_state_switch=0;
					
					mouse->last_press_l=mouse->press_l;

        
				if(gim.ctrl_mode==GIMBAL_AUTO_BIG_BUFF)
				{
				 if(mouse->press_l== 1) 
					{											
						if(buff_shoot_handle==0)
						{
							BUFF_shoot_flag=1;
							buff_shoot_handle=1;				
						}						
					}
					else
					{
						buff_shoot_handle=0;
					}
				}
			
			
  if(gim.ctrl_mode==GIMBAL_FOLLOW_ZGYRO)
	{
		if(mouse->press_r&&new_location.flag==1)
		  {
          pid_clr(&pid_yaw_speed);
          pid_clr(&pid_pit_speed);
          pid_clr(&pid_yaw);
          pid_clr(&pid_pit);
          autoshoot_mode = AUTO_SHOOT;
			}
	  else
			{
				  pid_clr(&pid_yaw_speed_follow);
          pid_clr(&pid_pit_speed_follow);
          pid_clr(&pid_yaw_follow);
          pid_clr(&pid_pit_follow);
         if(autoshoot_mode==AUTO_SHOOT){
				GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;
       	GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;			
		    autoshoot_mode = NORMAL_SHOOT;}
		  }
		}
  key->last_v = key->v;
}


/*********ң������ֵ*************/
void RemoteDataPrcess(uint8_t *pData)
{
  if(pData == NULL)
    {
      return;
    }

  RC_CtrlData.rc.ch0 = ((int16_t)pData[0] | ((int16_t)pData[1] << 8)) & 0x07FF;
  RC_CtrlData.rc.ch1 = (((int16_t)pData[1] >> 3) | ((int16_t)pData[2] << 5)) & 0x07FF;
  RC_CtrlData.rc.ch2 = (((int16_t)pData[2] >> 6) | ((int16_t)pData[3] << 2) |
                        ((int16_t)pData[4] << 10)) & 0x07FF;
  RC_CtrlData.rc.ch3 = (((int16_t)pData[4] >> 1) | ((int16_t)pData[5]<<7)) & 0x07FF;
  RC_CtrlData.rc.s1 = ((pData[5] >> 4) & 0x000C) >> 2;
  RC_CtrlData.rc.s2 = ((pData[5] >> 4) & 0x0003);//ģʽ�л�
  RC_CtrlData.mouse.x = ((int16_t)pData[6]) | ((int16_t)pData[7] << 8);
  RC_CtrlData.mouse.y = ((int16_t)pData[8]) | ((int16_t)pData[9] << 8);
  RC_CtrlData.mouse.z = ((int16_t)pData[10]) | ((int16_t)pData[11] << 8);
  RC_CtrlData.mouse.press_l = pData[12];
  RC_CtrlData.mouse.press_r = pData[13];
  RC_CtrlData.key.v = ((int16_t)pData[14]) | ((int16_t)pData[15] << 8);
  SetInputMode(&RC_CtrlData.rc);


  switch(GetInputMode( ))
    {
    case REMOTE_INPUT:
    {
      //ң��������ģʽ
      RemoteControlProcess(&(RC_CtrlData.rc));
      SetWorkState(NORMAL_STATE);
    }
    break;
    case KEY_MOUSE_INPUT:
    {
      //���̿���ģʽ
      MouseKeyControlProcess(&RC_CtrlData.mouse,&RC_CtrlData.key);
      SetWorkState(NORMAL_STATE);
    }
    break;
    case STOP:
    {
      SetWorkState(PREPARE_STATE);
      //����ͣ��
    }
    break;
    }
}



//ң��������ģʽ����
void RemoteControlProcess(Remote *rc)
{
  if(gim.ctrl_mode != GIMBAL_INIT)
    {
      ChassisSpeedRef.forward_back_ref = (RC_CtrlData.rc.ch1- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
      ChassisSpeedRef.left_right_ref   = (rc->ch0- (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_CHASSIS_SPEED_REF_FACT;
			
    }

  if(gim.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
    {
			if(((rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT)<0.5&&((rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT)>-0.5)
			{
				GimbalRef.yaw_angle_dynamic_ref=GimbalRef.yaw_angle_dynamic_ref;
			}
			else
			{
				GimbalRef.yaw_angle_dynamic_ref   += (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_YAW_ANGLE_INC_FACT;
			}
      GimbalRef.pitch_angle_dynamic_ref += (rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
      
    }
  GimbalRef.pitch_speed_ref = rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET;    //speed_ref�����������ж���
  GimbalRef.yaw_speed_ref   = (rc->ch2 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET);
  GimbalAngleLimit();


  /*******************ң�����������ݴ���*******************/
#if REMOTE_SHOOT == 1
  RemoteShootControl(&switch1, rc->s1);
#elif REMOTE_SHOOT == 0
  Remote_Rotate_Reverse_Control(&switch1, rc->s1);            //�˴�����������ѡһ���󲦸˹��ֱܷ��Ƿ����С����
#endif
  /***********************************************************/

}
//����������ģʽ����
u8 buff_flag;
u8 key_V_flag = 0;
u8 key_V_push_flag = 0;
u32 key_V_push_times = 0;
u8 key_Q_flag = 0;
u8 key_Q_push_flag = 0;
u32 key_Q_push_times = 0;
u8 key_E_flag = 0;
u8 key_E_push_flag = 0;
u8 key_E_push_flag_click;
u32 key_E_push_times = 0;
u8 key_B_flag = 0;
u32 key_B_push_flag = 0;
u32 key_B_push_times = 0;
u8 key_F_flag=0;
u8 key_F_push_flag = 0;
u8 key_F_push_flag_click;
u32 key_F_push_times = 0;
u32 key_X_push_times = 0;
u8 key_Z_flag = 0;
u8 key_Z_push_flag = 0;
u32 key_Z_push_times = 0;
u8  big_buff=0;
u8  small_buff=0;


u8 chassis_separate_flag=0;

u8 reset_flag;
float click=30;
float click_time1;

void MouseKeyControlProcess(Mouse *mouse, Key *key)
{


  if(gim.ctrl_mode!= GIMBAL_INIT&&(gim.auto_ctrl_cmd!=CMD_TARGET_NUM))
    {

      auto_angle_control();
      Mouse_Key_Rotate_Reverse_Control();						//С���ݺ�תͷģʽ�л�



      if(key->v &KEY_W)  // key: W
        {
          debug_high = 1;
          Key_Flag.Key_W_S_Flag = 1;
          ChassisSpeedRef.forward_back_ref = forward_back_speed;
        }

      else if(key->v &KEY_S) //key: S
        {
          debug_low = 1;
          Key_Flag.Key_W_S_Flag = 1;
          ChassisSpeedRef.forward_back_ref = -forward_back_speed;
        }
      else
        {
          Key_Flag.Key_W_S_Flag = 0;
          ChassisSpeedRef.forward_back_ref = 0;

        }

      if(key->v &KEY_A)  // key: A
        {

          Key_Flag.Key_A_D_Flag = 1;
          ChassisSpeedRef.left_right_ref = -left_right_speed;

        }
      else if(key->v &KEY_D) //key: D
        {

          Key_Flag.Key_A_D_Flag = 1;
          ChassisSpeedRef.left_right_ref = left_right_speed;

        }
      else
        {
          ChassisSpeedRef.left_right_ref = 0;
          Key_Flag.Key_A_D_Flag = 0;
        }




      if(key->v &KEY_SHIFT)
        {
          chassis_speed_mode = HIGH_SPEED_MODE;
          high_speed_flag = 1;
        }
      else if((high_speed_flag == 1)&&((key->v &KEY_SHIFT)!=1))
        {
          chassis_speed_mode = NORMAL_SPEED_MODE;
          high_speed_flag = 0;
        }
				
				
#if STANDARD == 3
        if((key->v&KEY_V)&&(key_V_flag==0)&&(key_V_push_flag == 0))            
        {
          small_buff=1;
          key_V_push_flag = 1;
					rotate_num=0;
					rotate_num_ture=0;
          gim.ctrl_mode = GIMBAL_AUTO_BIG_BUFF;
          chassis.ctrl_mode=CHASSIS_STOP;
          key_V_flag = 1;
        }
      else if((key->v&KEY_V)&&(key_V_flag==1)&&(key_V_push_flag == 0))            //
        {
          small_buff=0;
//          send_signal("to_a");
          key_V_push_flag = 1;

          gim.ctrl_mode = GIMBAL_INIT;

          chassis.ctrl_mode=MANUAL_FOLLOW_GIMBAL;

          key_V_flag = 0;
        }

      if(key_V_push_flag == 1)
        {
          key_V_push_times++;
        }
      if(key_V_push_times>30)
        {
          key_V_push_flag = 0;
          key_V_push_times = 0;
        }


      if((key->v&KEY_Z)&&(key_Z_flag==0)&&(key_Z_push_flag == 0))            //xiaofu
        {
          big_buff=1;
          key_Z_push_flag = 1;
          gim.ctrl_mode = GIMBAL_AUTO_SMALL_BUFF;
          chassis.ctrl_mode=CHASSIS_STOP;
          key_Z_flag = 1;
										rotate_num=0;
					rotate_num_ture=0;
        }
      else if((key->v&KEY_Z)&&(key_Z_flag==1)&&(key_Z_push_flag == 0))            //
        {
          big_buff=0;
          key_Z_push_flag = 1;
//          send_signal("to_a");
          gim.ctrl_mode = GIMBAL_INIT;
//          send_signal("to_a");
          chassis.ctrl_mode=MANUAL_FOLLOW_GIMBAL;
//          send_signal("to_a");
          key_Z_flag = 0;
        }
      if(key_Z_push_flag == 1)
        {
          key_Z_push_times++;
        }
      if(key_Z_push_times>30)
        {
          key_Z_push_flag = 0;
          key_Z_push_times = 0;
        }
#elif STANDARD == 4
				  //���ٷ���
        if(key->v&KEY_E&&key_E_push_flag==0)
        {
					gim.ctrl_mode=GIMBAL_CHANGE_DIRCTION;
					key_E_push_flag=1;
				}
				if(key_E_push_flag==1)
				{
					key_E_push_times++;
					if(key_E_push_times>=100)
					{
						if(reverse_flag==0)
						reverse_flag=1;
						else
						reverse_flag=0;
						key_E_push_flag=0;
						key_E_push_times=0;
					}
				}

				
				//������̨����
        if(key->v &KEY_F&& chassis_separate_flag == 0)
        {
          chassis.ctrl_mode = CHASSIS_SEPARATE;          
					key_F_push_flag=1;
        }
				if((chassis_separate_flag==1)&&(key->v &KEY_F))
				{
					
					chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
					key_F_push_flag=1;
				}
				if(key_F_push_flag==1)
				{
					key_F_push_times++;
					if(key_F_push_times>=100)
					{
						if(chassis_separate_flag == 0)
						{
							chassis_separate_flag = 1;
							key_F_push_times=0;
							key_F_push_flag=0;
						}
						else if(chassis_separate_flag == 1)
						{
							chassis_separate_flag = 0;
							key_F_push_times=0;
							key_F_push_flag=0;
						}
					}
				 }
#elif STANDARD == 5
				 
				 //������̨����
        if(key->v &KEY_F&& chassis_separate_flag == 0)
        {
          chassis.ctrl_mode = CHASSIS_SEPARATE;          
					key_F_push_flag=1;
        }
				if((chassis_separate_flag==1)&&(key->v &KEY_F))
				{
					
					chassis.ctrl_mode = MANUAL_FOLLOW_GIMBAL;
					key_F_push_flag=1;
				}
				if(key_F_push_flag==1)
				{
					key_F_push_times++;
					if(key_F_push_times>=100)
					{
						if(chassis_separate_flag == 0)
						{
							chassis_separate_flag = 1;
							key_F_push_times=0;
							key_F_push_flag=0;
						}
						else if(chassis_separate_flag == 1)
						{
							chassis_separate_flag = 0;
							key_F_push_times=0;
							key_F_push_flag=0;
						}
					}
				 }
				
#endif


			//����Q����һ�����ⷢ��ģʽ��״̬���������״ֻ̬�ܳ���һС��ʱ�䣬���ǿ���ͨ��������Q���г�ʱ��ʹ�á����������жϵ�Ƶ���Ƕ��٣�����
			//key_Q_flag��һ����־λ���ǵ�һ�ΰ���Q���ǵڶ��ΰ���Q
      if((key->v&KEY_Q)&&(key_Q_flag==0)&&(key_Q_push_flag == 0))
        {
          key_Q_push_flag = 1;
          bulletspead_level=1;
          key_Q_flag = 1;
        }
      else if((key->v&KEY_Q)&&(key_Q_flag==1)&&(key_Q_push_flag == 0))            //
        {
          key_Q_push_flag = 1;
          bulletspead_level=0;
          key_Q_flag = 0;
        }

      if(key_Q_push_flag == 1)
        {
          key_Q_push_times++;                           //���������Q�ǾͿ�ʼ��ʱ
        }
      if(key_Q_push_times>30)
        {
          key_Q_push_flag = 0;                          //�����ʱ����0
          key_Q_push_times = 0;
        }


      if(key->v &KEY_R)  // key:R
        {
          OpenDoor;
          door_flag=1;

        }
      else if(key->v &KEY_G)
        {
          CloseDoor;
          door_flag=0;
        }
				
				
        if(key->v &KEY_B)   //KEY_B �ͻ���ͼ�θ�λ
        {
          key_B_flag=1;
        }

        if(key->v &KEY_X)                //����X��λ
        {
          key_X_push_times++;
        }
        else
        {
          key_X_push_times = 0;
					reset_flag=0;
        }
        if(key_X_push_times>100)
			  {reset_flag=1;}
        if(key_X_push_times>130)
        {
          SoftReset();
          key_X_push_times = 0;
        }



      VAL_LIMIT(mouse->x, -100, 100);
      VAL_LIMIT(mouse->y, -100, 100);
      GimbalRef.pitch_angle_dynamic_ref -= mouse->y* MOUSE_TO_PITCH_ANGLE_INC_FACT;  //���Y������Ϊ������ˡ�-���Ա�֤����Ϊ��//(rc->ch3 - (int16_t)REMOTE_CONTROLLER_STICK_OFFSET) * STICK_TO_PITCH_ANGLE_INC_FACT;
     if(click==30&&click_time1>300)
		 {click=-30;
		 click_time1=0;}
		 else if(click==-30&&click_time1>300)
     {click=30;click_time1=0;}
		 click_time1++;
		 GimbalRef.yaw_angle_dynamic_ref   += mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;
//			GimbalRef.yaw_angle_dynamic_ref   += click*MOUSE_TO_YAW_ANGLE_INC_FACT;//mouse->x* MOUSE_TO_YAW_ANGLE_INC_FACT;		 //���X������Ϊ������ˡ�+����֤����Ϊ��
      GimbalRef.pitch_speed_ref = mouse->y;    //speed_ref�����������ж���
      GimbalRef.yaw_speed_ref   = mouse->x;
		 
		 if(GimbalRef.yaw_speed_ref==0&&GimbalRef.pitch_speed_ref==0)
		 {
			 
			 control_stop_flag=1;
			 
		 }
		 else
		 {
			 control_stop_flag=0;
		 }
		 
      GimbalAngleLimit();
      MouseShootControl(mouse,key);
    }

}

Shooting_State_e GetShootingState()
{
  return Shooting_State;
}

void SetShootingState(Shooting_State_e v)
{
  Shooting_State = v;
}

Shoot_State_e GetShootState()
{
  return shootState;
}

void SetShootState(Shoot_State_e v)
{
  shootState = v;
}

FrictionWheelState_e GetFrictionState()
{
  return friction_wheel_state;
}

void SetFrictionState(FrictionWheelState_e v)
{
  friction_wheel_state = v;
}
//ң��������ֵ���ã�

float gimbal_pitch_angle_bias = 0;
void GimbalAngleLimit()
{
//	gimbal_pitch_angle_bias = fabs(GMPitchEncoder.ecd_angle)-fabs(pitch_Angle);
  if(gim.ctrl_mode == GIMBAL_INIT)
    {
      VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,  - PITCH_MAX,  + PITCH_MAX);
//		VAL_LIMIT(GimbalRef.yaw_angle_dynamic_ref, pid_yaw.get - 60,pid_yaw.get + 60);
    }
  else if(gim.ctrl_mode == GIMBAL_FOLLOW_ZGYRO)
    {

      VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,  PITCH_MIN + (GMPitchEncoder.ecd_angle)+pitch_Angle, PITCH_MAX + (GMPitchEncoder.ecd_angle)+pitch_Angle);


//		VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,  PITCH_MIN , PITCH_MAX );

//		VAL_LIMIT(GimbalRef.yaw_angle_dynamic_ref, YAW_MIN - (-GMYawEncoder.ecd_angle)+pid_yaw.get, YAW_MAX - (-GMYawEncoder.ecd_angle)+pid_yaw.get);

      //		VAL_LIMIT(GimbalRef.yaw_angle_dynamic_ref, pid_yaw.get + YAW_MIN,pid_yaw.get + YAW_MAX);
    }

}

//ң�������ݳ�ʼ����б�º����ȵĳ�ʼ��
void RemoteTaskInit()
{

  //б�³�ʼ��
  frictionRamp.SetScale(&frictionRamp, FRICTION_RAMP_TICK_COUNT);
//	frictionRamp_Off.SetScale(&frictionRamp_Off, FRICTION_RAMP_OFF_TICK_COUNT);
  chassisRamp.SetScale(&frictionRamp, CHASSIS_RAMP_TICK_COUNT);

//	LRSpeedRamp.SetScale(&LRSpeedRamp, MOUSE_LR_RAMP_TICK_COUNT);
  FBSpeedRamp.SetScale(&FBSpeedRamp, MOUSR_FB_RAMP_TICK_COUNT);
  FBSpeedRampSecond.init(&FBSpeedRampSecond,MOUSR_FB_RAMP_TICK_COUNT);

  chassisRamp.SetScale(&frictionRamp, CHASSIS_RAMP_TICK_COUNT);

  chassisRamp.ResetCounter(&chassisRamp);
  frictionRamp.ResetCounter(&frictionRamp);
//	frictionRamp_Off.ResetCounter(&frictionRamp_Off);
  LRSpeedRamp.ResetCounter(&LRSpeedRamp);
  FBSpeedRamp.ResetCounter(&FBSpeedRamp);
  memset(&Key_Flag, 0, sizeof(Key_Flag_t));
  //������̨����ֵ��ʼ��
  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
  GimbalRef.yaw_angle_dynamic_ref = 0.0f;
  ChassisSpeedRef.forward_back_ref = 0.0f;
  ChassisSpeedRef.left_right_ref = 0.0f;
  ChassisSpeedRef.rotate_ref = 0.0f;
  //Ħ��������״̬��ʼ��
  SetFrictionState(FRICTION_WHEEL_OFF);
  FRICTION_WHEEL_DUTY = FRICTION_WHEEL_MAX_DUTY;

}

//��굥�����
mouse_states_e mouse_read(void)
{
  mouse_states_e mouse_states;
  if(RC_CtrlData.mouse.press_l ==1 )
    {
      mouse_states = MOUSE_UP;
    }
  else
    {
      mouse_states = MOUSE_DOWN;
    }

  return mouse_states;
}
//��굥��״̬ɨ��
mouse_msg_e mouse_scan(void)
{
  static mouse_msg_e  mouse_now_msg = MOUSE_S1;
  static mouse_msg_e  mouse_last_msg = MOUSE_S1;
  mouse_msg_e mouse_return_msg = MOUSE_S1;
  static unsigned int  mousetime = 0;
  switch (mouse_last_msg)
    {
    case MOUSE_S1:
      if (MOUSE_DOWN == mouse_read())
        {
          mouse_now_msg = MOUSE_S2;
        }

      break;

    case MOUSE_S2:
      if (MOUSE_DOWN == mouse_read())
        {
          mousetime = mousetime + SCAN_TIME;
          if(mousetime >= DOWN_TIME)
            {
              mouse_now_msg = MOUSE_S3;
            }
          else
            {
              mouse_now_msg = MOUSE_S2;
            }

        }
      else if(MOUSE_UP == mouse_read())
        {
          mouse_now_msg = MOUSE_S1;
          mousetime = 0;
        }
      break;

    case MOUSE_S3:
      if (MOUSE_DOWN == mouse_read())
        {
          mousetime = mousetime + SCAN_TIME;
          if(mousetime >= 1000)
            {
              mouse_now_msg = MOUSE_S4;
            }
          else
            {
              mouse_now_msg = MOUSE_S3;
            }

        }
      else if(MOUSE_UP == mouse_read())
        {
          mouse_now_msg = MOUSE_S1;
          mousetime = 0;
        }
      break;
    case MOUSE_S4:
      if (MOUSE_DOWN == mouse_read())
        {
          mouse_now_msg = MOUSE_S4;
          mouse_return_msg = MOUSE_S4;
        }
      else if(MOUSE_UP == mouse_read())
        {
          mouse_now_msg = MOUSE_S1;
          mousetime = 0;
        }
      break;
    }


  if(mouse_now_msg == MOUSE_S1)
    {
      if(mouse_last_msg == MOUSE_S3)
        mouse_return_msg = MOUSE_S3;
    }
  mouse_last_msg = mouse_now_msg;
  return mouse_return_msg;
}
//���̵������
keyboard_states_e keyboard_read(Key *key,u32 ked_id)
{
  keyboard_states_e keyboard_states;

  if(key->v & ked_id)
    {
      keyboard_states = KEYBOARD_UP;
    }
  else
    {
      keyboard_states = KEYBOARD_DOWN;
    }

  return keyboard_states;
}
//����״̬ɨ��
keyboard_msg_e keyboard_scan(Key *key,u32 ked_id)
{
  static keyboard_msg_e  keyboard_now_msg = KEYBOARD_S1;
  static keyboard_msg_e  keyboard_last_msg = KEYBOARD_S1;
  keyboard_msg_e keyboard_return_msg = KEYBOARD_S1;
  static unsigned int  keyboardtime = 0;
  switch (keyboard_last_msg)
    {
    case KEYBOARD_S1:
      if (KEYBOARD_DOWN == keyboard_read(key,ked_id))
        {
          keyboard_now_msg = KEYBOARD_S2;
        }

      break;

    case KEYBOARD_S2:
      if (KEYBOARD_DOWN == keyboard_read(key,ked_id))
        {
          keyboardtime = keyboardtime + SCAN_TIME;
          if(keyboardtime >= DOWN_TIME)
            {
              keyboard_now_msg = KEYBOARD_S3;
            }
          else
            {
              keyboard_now_msg = KEYBOARD_S2;
            }

        }
      else if(KEYBOARD_UP == keyboard_read(key,ked_id))
        {
          keyboard_now_msg = KEYBOARD_S1;
          keyboardtime = 0;
        }
      break;

    case KEYBOARD_S3:
      if (KEYBOARD_DOWN == keyboard_read(key,ked_id))
        {
          keyboardtime = keyboardtime + SCAN_TIME;
          if(keyboardtime >=1000)
            {
              keyboard_now_msg = KEYBOARD_S4;
            }
          else
            {
              keyboard_now_msg = KEYBOARD_S3;
            }

        }
      else if(KEYBOARD_UP == keyboard_read(key,ked_id))//??????,????????
        {
          keyboard_now_msg = KEYBOARD_S1;
          keyboardtime = 0;
        }
      break;
    case KEYBOARD_S4:
      if (KEYBOARD_DOWN == keyboard_read(key,ked_id))
        {
          keyboard_now_msg = KEYBOARD_S4;
          keyboard_return_msg = KEYBOARD_S4;
        }
      else if(KEYBOARD_UP == keyboard_read(key,ked_id))
        {
          keyboard_now_msg = KEYBOARD_S1;
          keyboardtime = 0;
        }
      break;
    }//end switch

  if(keyboard_now_msg == KEYBOARD_S1)
    {
      if(keyboard_last_msg == KEYBOARD_S3)
        keyboard_return_msg = KEYBOARD_S3;
    }
  keyboard_last_msg = keyboard_now_msg;
  return keyboard_return_msg;
}

void SoftReset(void)
{
  __set_FAULTMASK(1);      // �ر������ж�
  NVIC_SystemReset();// ��λ
}
float vx,vy;

void remotecontrolangle()
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



