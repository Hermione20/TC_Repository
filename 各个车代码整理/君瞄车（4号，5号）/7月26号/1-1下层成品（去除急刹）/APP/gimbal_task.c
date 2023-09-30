#include "main.h"
#include "AHRS_MiddleWare.h"
/* gimbal back center time (ms) */
#define BACK_CENTER_TIME 1500
gimbal_t gim;
u32 Gimbal_pitch_mid_point = 0;
int32_t x_bias = 0;
int32_t y_bias = 0;
extern Mouse mouse;
float Delta_Dect_Angle_Yaw,Delta_Dect_Angle_Pit;
Gimbal_Auto_Shoot_t Gimbal_Auto_Shoot;

//自瞄参数
Gimbal_Auto_Shoot_t Gimbal_Auto_Shoot;
float Delta_Dect_Angle_Yaw;
float Delta_Dect_Angle_Pit;
float	Machine_Yaw_Compensation = 0;
float	Machine_Pitch_Compensation = 0;
float Speed_Distence_Compensation = 0;   //射速和距离的pitch轴补偿
float now_distance = 0;
float last_distance = 0;
float Rotate_compensation = 0;

//测试用
float det_angle = 0;
float Image_Gimbal_Delay_Compensation;

double shoot_angle_speed = 0;
double distance_s, distance_x, distance_y;
double x1, x2, x3, x4;
float angle_tan, shoot_radian, shoot_angle;
float test_angle;

void gimbal_task(void)
{//Set_Gimbal_Current1(CAN2,0,3000,0,0);

//	  Distance_handle();

#if IMU == ICM20948
  IMU_getYawPitchRoll();
#elif IMU == HI219
  Hi220_getYawPitchRoll();
#endif

  switch ( gim.ctrl_mode)
    {
    case GIMBAL_INIT:
      init_mode_handle();
      break;
    case GIMBAL_FOLLOW_ZGYRO:
      close_loop_handle();
      break;
    case GIMBAL_AUTO_SMALL_BUFF:
      auto_small_buff_handle();
      break;
    case GIMBAL_AUTO_BIG_BUFF:
      auto_big_buff_handle();
      break;
    case GIMBAL_AUTO_ANGLE:
      auto_angle_handle();
      break;
    case GIMBAL_FOLLOW_CHASSIS:
      gimbal_follow_chassis_handle();
      break;
    default:
      break;
    }

  switch (gim.ctrl_mode)
    {
    case GIMBAL_INIT:
    {
      pid_calc(&pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
      cascade_pid_ctrl();
      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed, -gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed.out, (int16_t)pid_pit_speed.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }
    }
    break;

    case GIMBAL_FOLLOW_ZGYRO:
    {
      if(autoshoot_mode == NORMAL_SHOOT)
        {
          pid_calc(&pid_yaw, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
          pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
          cascade_pid_ctrl();
//				OutData[0]=(int)(pid_yaw.set * 100);
//				OutData[1]=(int)(pid_yaw.get * 100);
//				OutData[2]=(int)(pid_pit.set * 100);
//				OutData[3]=(int)(pid_pit.get* 100);
        }
      else
        {
          pid_calc(&pid_yaw_follow, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);	//计算角度环输出
          pid_calc(&pid_pit_follow, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);	//计算角度环输出
          auto_aim_cascade_pid_ctrl();
        }

      /* safe protect */
      if (gimbal_is_controllable())
        {
          if(autoshoot_mode == NORMAL_SHOOT)
            {
              pid_calc(&pid_yaw_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
              pid_calc(&pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
              Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed.out, -(int16_t)pid_pit_speed.out);
            }
          else
            {
              pid_calc(&pid_yaw_speed_follow,gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
              pid_calc(&pid_pit_speed_follow, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
              Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_follow.out, -(int16_t)pid_pit_speed_follow.out);
            }
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }

    }
    break;
    case GIMBAL_AUTO_SMALL_BUFF:
    {
      pid_calc(&pid_yaw_small_buff, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit_small_buff, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);
//					OutData[0]=(int)pid_pit_small_buff.iout * 100;
//					OutData[1]=(int)pid_pit_small_buff.dout * 100;
//					OutData[2]=(int)(pid_pit_small_buff.set * 100);
//					OutData[3]=(int)(pid_pit_small_buff.get* 100);
      gim.pid.pit_speed_ref = pid_pit_small_buff.out;
      gim.pid.yaw_speed_ref = pid_yaw_small_buff.out;
      gim.pid.yaw_speed_fdb = yaw_Gyro;
      gim.pid.pit_speed_fdb = -pitch_Gyro;

//      OutData[0]=(int)(pid_yaw_small_buff.set * 100);
//      OutData[1]=(int)(pid_yaw_small_buff.get * 100);
//      OutData[2]=(int)(pid_pit_small_buff.iout * 100);
//      OutData[3]=(int)(pid_pit_small_buff.out * 100);

      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_speed_small_buff, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed_small_buff, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          //Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_small_buff.out, 0);

//					OutData[0]=(int)pid_yaw_speed_small_buff.set * 100;
//					OutData[1]=(int)pid_yaw_speed_small_buff.get * 100;
//					OutData[2]=(int)(pid_yaw_speed_small_buff.set * 100);
//					OutData[3]=(int)(pid_yaw_speed_small_buff.get* 100);

          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_small_buff.out, -(int16_t)pid_pit_speed_small_buff.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }
    }
    break;

    case GIMBAL_AUTO_BIG_BUFF:
    {
      pid_calc(&pid_yaw_big_buff, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit_big_buff, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);

      gim.pid.yaw_speed_ref = pid_yaw_big_buff.out;
      gim.pid.pit_speed_ref = pid_pit_big_buff.out;
      gim.pid.yaw_speed_fdb =	yaw_Gyro;
      gim.pid.pit_speed_fdb =	 - pitch_Gyro;

      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_speed_big_buff, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed_big_buff, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          //Set_Gimbal_Current(CAN2, 0, -(int16_t)pid_pit_speed_big_buff.out);
          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_big_buff.out, -(int16_t)pid_pit_speed_big_buff.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }

    }
    break;
    case GIMBAL_AUTO_ANGLE:
    {
      pid_calc(&pid_yaw_auto_angle, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit_auto_angle, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);

      gim.pid.yaw_speed_ref = pid_yaw_auto_angle.out;
      gim.pid.pit_speed_ref = pid_pit_auto_angle.out;
      gim.pid.yaw_speed_fdb =	yaw_Gyro;
      gim.pid.pit_speed_fdb =	pitch_Gyro;

      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_speed_auto_angle, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed_auto_angle, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_speed_auto_angle.out, (int16_t)pid_pit_speed_auto_angle.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }
    }
    break;
    case GIMBAL_FOLLOW_CHASSIS:
    {
      pid_calc(&pid_yaw_follow_chassis_angle, gim.pid.yaw_angle_fdb, gim.pid.yaw_angle_ref);
      pid_calc(&pid_pit, gim.pid.pit_angle_fdb, gim.pid.pit_angle_ref);

      gim.pid.yaw_speed_ref = pid_yaw_follow_chassis_angle.out;
      gim.pid.pit_speed_ref = pid_pit.out;
      gim.pid.yaw_speed_fdb =	yaw_Gyro;
      gim.pid.pit_speed_fdb =	pitch_Gyro;

      if (gimbal_is_controllable())
        {
          pid_calc(&pid_yaw_follow_chassis_speed, gim.pid.yaw_speed_fdb, gim.pid.yaw_speed_ref);
          pid_calc(&pid_pit_speed, gim.pid.pit_speed_fdb, gim.pid.pit_speed_ref);
          Set_Gimbal_Current(CAN2, -(int16_t)pid_yaw_follow_chassis_speed.out, (int16_t)pid_pit_speed.out);
        }
      else
        {
          Set_Gimbal_Current(CAN2, 0, 0);     //yaw + pitch
          gim.ctrl_mode = GIMBAL_RELAX;
        }
    }
    break;
    default:
      break;
    }

}
extern u8 debug_high,debug_low;
void cascade_pid_ctrl(void)
{
  gim.pid.yaw_speed_ref = pid_yaw.out;//LQ
  gim.pid.pit_speed_ref = pid_pit.out;
  gim.pid.yaw_speed_fdb =	yaw_Gyro;
  gim.pid.pit_speed_fdb =	-pitch_Gyro;
}

void auto_aim_cascade_pid_ctrl(void)
{
  gim.pid.pit_speed_ref = pid_pit_follow.out;
  gim.pid.yaw_speed_ref = pid_yaw_follow.out;
  gim.pid.yaw_speed_fdb = yaw_Gyro;
  gim.pid.pit_speed_fdb = -pitch_Gyro;
}


int32_t init_rotate_num = 0;
int32_t init_Yaw_angle = 0;

void init_mode_handle(void)
{
  gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;  //向上为负
  gim.pid.pit_angle_ref = 0;//-GMPitchEncoder.ecd_angle* (1 -GMPitchRamp.Calc(&GMPitchRamp));
  gim.pid.yaw_angle_fdb = -GMYawEncoder.ecd_angle;//

  init_rotate_num = -GMYawEncoder.ecd_angle/360;
  init_Yaw_angle = init_rotate_num*360;

  gim.pid.yaw_angle_ref = init_Yaw_angle;//-GMYawEncoder.ecd_angle*(1 -GMYawRamp.Calc(&GMYawRamp));

  if(gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref>=180)
    gim.pid.yaw_angle_ref+=360;
  else if(gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref<-180)
    gim.pid.yaw_angle_ref-=360;

  if (gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref >= -1.5f && gim.pid.yaw_angle_fdb-gim.pid.yaw_angle_ref <= 1.5f && gim.pid.pit_angle_fdb >= -4 && gim.pid.pit_angle_fdb <= 4)
    {
      gim.ctrl_mode = GIMBAL_FOLLOW_ZGYRO;
      chassis.follow_gimbal = 1;
      gim.pid.yaw_angle_fdb = yaw_Angle;
      GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;		//陀螺仪向右为正
      GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;	//陀螺仪向上为正
      Gimbal_pitch_mid_point = pitch_Angle;

      chassis.position_ref = GMYawEncoder.ecd_angle;

    }
}
void no_action_handle(void)
{
  gim.pid.yaw_angle_fdb = yaw_Angle;
  GimbalRef.yaw_angle_dynamic_ref = yaw_Angle;
  gim.pid.pit_angle_fdb = pitch_Angle;
  GimbalRef.pitch_angle_dynamic_ref = pitch_Angle;
}

auto_shoot_mode_e  autoshoot_mode = NORMAL_SHOOT ;

float Filter_for_Speed_Prediction(float input,float *data_temp)
{
  uint8_t i = 0;
  float sum=0;

  for(i=1; i<4; i++)
    {
      data_temp[i-1] = data_temp[i];
    }
  data_temp[3] = input;

  for(i=0; i<4; i++)
    {
      sum += data_temp[i];
    }
  return(sum*0.25f);
}

void AutoShootAngleLimit()
{
  VAL_LIMIT(GimbalRef.pitch_angle_dynamic_ref,  - PITCH_MAX,  + PITCH_MAX);
//	VAL_LIMIT(GimbalRef.yaw_angle_dynamic_ref, pid_yaw.get - 60,pid_yaw.get + 60);
}
float YawTimeSpeed=0,PitchTimeSpeed=0;
float Army_Speed_Yaw,Army_Speed_Pit,Gimbal_Speed_Yaw,initanleyaw,initanlepitch;
int addli;
uint8_t Rece_F;
float new_locationx_last,new_locationy_last,x_temp=0,y_temp=0;;
void close_loop_handle(void)
{
  static u8 flag_lost = 0;
  switch (autoshoot_mode)
    {
    case NORMAL_SHOOT:
    {
      gim.pid.pit_angle_fdb = pitch_Angle;
      gim.pid.yaw_angle_fdb = yaw_Angle;
      gim.pid.pit_angle_ref = GimbalRef.pitch_angle_dynamic_ref;
      gim.pid.yaw_angle_ref = GimbalRef.yaw_angle_dynamic_ref;
    }
    break;

    case AUTO_SHOOT:
    {
      if(new_location.flag)
        {
          Gimbal_Auto_Shoot.Recognized_Flag = 1;
          Gimbal_Auto_Shoot.Recognized_Timer = 0;
          Gimbal_Auto_Shoot.Err_Pixels_Yaw	= new_location.x;
          Gimbal_Auto_Shoot.Err_Pixels_Pit	= new_location.y;
          Gimbal_Auto_Shoot.Distance = (new_location.dis +100)* 10;//单位mm
					 Gimbal_Auto_Shoot.Distance = 6000;//单位mm
        }
      else
        {
          Gimbal_Auto_Shoot.Recognized_Timer++;
          if(Gimbal_Auto_Shoot.Recognized_Timer == 10)//500ms，时间太长，云台保持原有给定导致乱动作
            {
              Gimbal_Auto_Shoot.Recognized_Flag 	= 0;
              Gimbal_Auto_Shoot.Recognized_Timer 	= 0;
              Gimbal_Auto_Shoot.Err_Pixels_Yaw	= 0;
              Gimbal_Auto_Shoot.Err_Pixels_Pit	= 0;
            }
        }
      if(Gimbal_Auto_Shoot.Recognized_Flag)//(new_location.flag)
        {
          Gimbal_Auto_Shoot.Continue_Recognized_Cnt ++;
          VAL_LIMIT(Gimbal_Auto_Shoot.Continue_Recognized_Cnt,0,100);
          gim.pid.yaw_angle_fdb =  yaw_Angle;
          gim.pid.pit_angle_fdb = pitch_Angle;
          flag_lost = 0;

          Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample++;
          //-/-------------------- 收到一帧图像识别的数据，进行处理 ---------------------/-//
          //如果此时丢帧，那么new_location.x和new_location.y值将保持不变
          if(new_location.receNewDataFlag)
            {
              new_location.receNewDataFlag = 0;

              Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( Gimbal_Auto_Shoot.Err_Pixels_Yaw * IMAGE_LENGTH, FOCAL_LENGTH );
              Gimbal_Auto_Shoot.Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( Gimbal_Auto_Shoot.Err_Pixels_Pit * IMAGE_LENGTH, FOCAL_LENGTH );
							
							Gimbal_Auto_Shoot.Amror_pit = Gimbal_Auto_Shoot.Delta_Dect_Angle_Pit + gim.pid.pit_angle_fdb;
							Gimbal_Auto_Shoot.Amror_yaw = Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw + gim.pid.yaw_angle_fdb;

							
              //-/-------------------根据射速和距离进行pitch轴角度的补偿 -------------------/-//
              /*
              1、目标距离和射速对补偿角度的影响
              2、计算因摄像头和枪管不同心造成的云台偏差
              3、射速按等级判断，距离由传感器或者视觉给出，也要考虑没有距离信息的情况
              4、①没有距离信息，这里就不使用自瞄打击远处目标，只考虑三米之内的情况
              	 ②有距离信息，在计算出来的角度上进行进一步补偿
              5、在和目标Yaw轴偏差角度小于一定值后说明瞄到了目标上，此时再加距离的补偿
              */

              //1、获取并处理距离信息，根据距离信息进行补偿(参数待调整)
//					VAL_LIMIT(now_distance,100,1000);
              Speed_Distence_Compensation = Gimbal_Auto_Shoot.Distance * 0.001;
              //3、获取裁判系统射速信息,射速分别可为15,18,30（参数待调整）、
              if ( judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit < 16 )
                {
                  shoot_angle_speed = 14.0;
                  Gimbal_Auto_Shoot.Speed_Prediction.time_delay = 260e-3f;
									PitchTimeSpeed=0;//0
                }
              else if ( judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit > 16 & judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit < 20 )
                {
                  shoot_angle_speed = 17.0;
                  Gimbal_Auto_Shoot.Speed_Prediction.time_delay = 250e-3f;
									PitchTimeSpeed=-0.4;//-1.5
                }
              else if ( judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit > 20 & judge_rece_mesg.game_robot_state.shooter_id1_17mm_speed_limit < 31 )
                {
                  shoot_angle_speed = 27.0;
                  Gimbal_Auto_Shoot.Speed_Prediction.time_delay = 150e-3f;
								PitchTimeSpeed=-0.7;
                }
              else   //没有裁判系统数据，默认射速为15
                {
                  shoot_angle_speed = 14.0;
									Gimbal_Auto_Shoot.Speed_Prediction.time_delay = 260e-3f;
									PitchTimeSpeed=0;
                }

              /*
                  3、根据射速和距离信息进行补偿
                  另一种思路，将距离分段每段0.5m，1-6m下分为七段，和三种射速。
                  在不同的距离和不同射速下测补偿，拟合出补偿关于射速和距离的函数，作为补偿值
              */

              //4、机械补偿，自瞄先调该参数，保证指的是pitch和yaw轴的中心。
              //yaw轴为（摄像头和枪管中心的安装偏差角）补偿，pitch轴分别为（摄像头与枪管中心的安装偏差角）和（摄像头与枪管中心的距离）补偿
              //出现固定的角度偏差可以调节该参数。

              Machine_Pitch_Compensation = PITCH_ANGLE_BETWEEN_GUN_CAMERA + RAD_TO_ANGLE * atan2 ( HEIGHT_BETWEEN_GUN_CAMERA, Gimbal_Auto_Shoot.Distance );

              distance_s = ( Gimbal_Auto_Shoot.Distance ) / 1000;
              distance_x = ( cos ( Gimbal_Auto_Shoot.Amror_pit * ANGLE_TO_RAD ) * distance_s );
              distance_y = ( sin ( Gimbal_Auto_Shoot.Amror_pit * ANGLE_TO_RAD ) * distance_s );

              x1 = shoot_angle_speed * shoot_angle_speed;
              x2 = distance_x * distance_x;
              x3 = sqrt ( x2 - ( 19.6 * x2 * ( ( 9.8 * x2 ) / ( 2.0 * x1 ) + distance_y ) ) / x1 );
              x4 = 9.8 * x2;
              angle_tan = ( x1 * ( distance_x - x3 ) ) / ( x4 );
              shoot_radian = atan ( angle_tan );
              Gimbal_Auto_Shoot.shoot_angle = shoot_radian * RAD_TO_ANGLE+PitchTimeSpeed;
//					  Gimbal_Auto_Shoot.Speed_Prediction.time_delay = distance_x / ( shoot_angle_speed * cos( shoot_radian ) );

              //5、小陀螺补偿
              if(chassis.ctrl_mode == CHASSIS_ROTATE)
                {
                  if(chassis.vw > 0)
                    Rotate_compensation = -1.0f;
                  else
                    Rotate_compensation = 1.0f;
                }
              else
                {
                  Rotate_compensation = 0;
                }
              //--/----------  计算枪管距离目标点的偏差角（作为给定的一部分） ----------------------/--//

              Gimbal_Auto_Shoot.Ballistic_Compensation = Machine_Pitch_Compensation;

              //--/----------------------- 云台角度给定 ----------------------/--//
              gim.pid.yaw_angle_ref =  Gimbal_Auto_Shoot.Amror_yaw + YAW_ANGLE_BETWEEN_GUN_CAMERA;
             gim.pid.pit_angle_ref =  Gimbal_Auto_Shoot.shoot_angle + Gimbal_Auto_Shoot.Ballistic_Compensation;

								

              //--/----------------------- 云台角度给定 ----------------------/--//

              //--/--------------速度预测，计算目标相对于自己的移动速度---------/--//			调预测主要是云台参数和延迟时间
#if ARMY_SPEED_PREDICTION == 1

//				if ( fabs( Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw ) < 6.0f )
//				{
              if ( Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample % 5 == 0 )
                {
                  Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now;                    //上次采集的角度
                  Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now = Gimbal_Auto_Shoot.Amror_yaw;     //现在采集的角度
                  Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now;
                  Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now = Gimbal_Auto_Shoot.Amror_pit;
                  Gimbal_Auto_Shoot.Speed_Prediction.time1 = Gimbal_Auto_Shoot.Speed_Prediction.time2;
                  Gimbal_Auto_Shoot.Speed_Prediction.time2 = time_tick_1ms;

                  Gimbal_Auto_Shoot.Speed_Prediction.time_error = Gimbal_Auto_Shoot.Speed_Prediction.time2 - Gimbal_Auto_Shoot.Speed_Prediction.time1 + 2;
                  Gimbal_Auto_Shoot.Speed_Prediction.yaw_angle_error = Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Now - Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Angle_Pre;
                  Gimbal_Auto_Shoot.Speed_Prediction.pit_angle_error = Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Now - Gimbal_Auto_Shoot.Speed_Prediction.Pit_Angle_Pre;
                  Army_Speed_Yaw = ( ( Gimbal_Auto_Shoot.Speed_Prediction.yaw_angle_error )	//角速度计算
                                     / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;
                  Army_Speed_Pit = ( ( Gimbal_Auto_Shoot.Speed_Prediction.pit_angle_error )
                                     / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;


                  if ( fabs ( Army_Speed_Yaw ) < 30 )
                    {
                      Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed;
                      Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = Army_Speed_Yaw;
                      Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Acceleration = ( ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed
                          - Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed_Pre )
                          / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;
                    }
                  if ( fabs ( Army_Speed_Pit ) < 30 )
                    {
                      Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed_Pre = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed;
                      Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed = Army_Speed_Pit;
                      Gimbal_Auto_Shoot.Speed_Prediction.Pit_Acceleration = ( ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed
                          - Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed_Pre )
                          / ( Gimbal_Auto_Shoot.Speed_Prediction.time_error ) ) * 1000;
                    }
                }
              Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 1;
//						Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample = 0;
#endif
						  Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = AvgFilter ( Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed );
            }
          //-/---------------- 卡尔曼滤波并更新云台给定角度 ----------------------------/-//
#if ENABLE_KALMAN_FILTER == 1
          kalman_filter_calc ( &kalman_filter_F,
                               Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed,
                               Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed,
                               Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Acceleration,
                               Gimbal_Auto_Shoot.Speed_Prediction.Pit_Acceleration
                             );
          Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed = kalman_filter_F.filtered_value[0];
          Gimbal_Auto_Shoot.Filtered_Angular_Pit_Speed = kalman_filter_F.filtered_value[1];

          Gimbal_Auto_Shoot.Filtered_Yaw_Acceleration =  kalman_filter_F.filtered_value[2];
          Gimbal_Auto_Shoot.Filtered_Pit_Acceleration =  kalman_filter_F.filtered_value[3];
#endif

          //-------------------- 计算图像处理和云台动作延迟量，作为预测的补偿 ----------------------/-//
#if ARMY_SPEED_PREDICTION == 1
          if ( Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag == 1 )
            {
              Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 0;
#if ENABLE_KALMAN_FILTER == 1	           //使用卡尔曼滤波之后的速度进行补偿，补偿图像处理延迟和云台动作延迟，和云台参数有关，P尤为明显
//							Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
   //         Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );
          Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + YawTimeSpeed);
					
//              Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * ( YawTimeSpeed + Gimbal_Auto_Shoot.Speed_Prediction.time_delay );						
              Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Filtered_Angular_Pit_Speed * PIT_IMAGE_GIMBAL_DELAY_TIME;
//					    Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
//                                                          					+ Gimbal_Auto_Shoot.Filtered_Yaw_Acceleration
//					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
//					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay ) / 2;
//					    Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
//                                                          					+ Gimbal_Auto_Shoot.Speed_Prediction.Yaw_Acceleration
//					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay )
//					                                                          * ( YAW_IMAGE_GIMBAL_DELAY_TIME + Gimbal_Auto_Shoot.Speed_Prediction.time_delay ) / 2;
//					    Gimbal_Auto_Shoot.Yaw_Image_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Filtered_Angular_Yaw_Speed * (Gimbal_Auto_Shoot.Speed_Prediction_Gimbal.Pit_Dect_Angle + YAW_IMAGE_GIMBAL_DELAY_TIME);

#else							                       //使用直接求得的速度进行补偿	
              Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed * YAW_IMAGE_GIMBAL_DELAY_TIME;
              Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation  = Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed * PIT_IMAGE_GIMBAL_DELAY_TIME;
#endif

            }

          //-/------------------------ 加入补偿，更新云台给定角度 ----------------------/-//

          //根据偏差角计算偏差距离，由于距离信息存在误差，不使用计算的temp = fabs(Gimbal_Auto_Shoot.Distance * 2.0f * arm_sin_f32(0.5f * fabs(Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw) * 0.01745329252f ));
          //偏差角度小于2度，说明此时基本瞄准装甲片，两个时间参数应该需要调整。
          //该处应该加上pitch轴的预测判定
          if ( fabs ( Gimbal_Auto_Shoot.Delta_Dect_Angle_Yaw ) < 2.0 + fabs ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation ) )
            {
              Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 0;
              gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
              gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
            }
          //偏差角大于2°，分为另种情况。
          //1、刚识别到，偏差较大，如果直接补偿会导致云台抖动震荡；
          //2、识别到一定时间，但是由于无补偿导致跟踪滞后
          //3、云台在抖动，导致yaw反馈在中间跳跃
          else
            {
              Gimbal_Auto_Shoot.Continue_Large_Err_Cnt++;
              if ( Gimbal_Auto_Shoot.Continue_Large_Err_Cnt >= 150 )  //300ms，持续300ms的偏差，表明此时为跟踪滞后，需加入补偿
                {
                  Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 150;
                  gim.pid.yaw_angle_ref += ( Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation );
                  gim.pid.pit_angle_ref += ( Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation );
                }
            }
          Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation = 0;         //补偿一次后清零
          Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation = 0;

#endif

        }
      else
        {
          if(flag_lost == 0)
            {
              flag_lost = 1;
              gim.pid.yaw_angle_ref = yaw_Angle ;
              gim.pid.pit_angle_ref = pitch_Angle;

              pid_clr(&pid_pit_speed_follow);
              pid_clr(&pid_yaw_speed_follow);
              pid_clr(&pid_yaw_follow);
              pid_clr(&pid_pit_follow);
              pid_clr(&pid_yaw_speed);
              pid_clr(&pid_pit_speed);
              pid_clr(&pid_yaw);
              pid_clr(&pid_pit);
#if ENABLE_KALMAN_FILTER == 1
              kalman_filter_reset(&kalman_filter_F, &kalman_filter_I);
#endif
            }
          gim.pid.pit_angle_fdb = pitch_Angle;
          gim.pid.yaw_angle_fdb = yaw_Angle;

#if ARMY_SPEED_PREDICTION == 1
          Gimbal_Auto_Shoot.Speed_Prediction.Time_Sample = 0;
          Gimbal_Auto_Shoot.Speed_Prediction.Angular_Pit_Speed = 0;
          Gimbal_Auto_Shoot.Speed_Prediction.Angular_Yaw_Speed = 0;
          Gimbal_Auto_Shoot.Yaw_Gimbal_Delay_Compensation = 0;
          Gimbal_Auto_Shoot.Pit_Gimbal_Delay_Compensation = 0;
          Gimbal_Auto_Shoot.Image_Gimbal_Delay_Compensation_Flag = 0;
          Gimbal_Auto_Shoot.Continue_Large_Err_Cnt = 0;
#endif
        }
//-----------------------------云台限幅-----------------------//
//      VAL_LIMIT(gim.pid.pit_angle_ref, PITCH_MIN +(GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb, PITCH_MAX + (GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb);
//	VAL_LIMIT(gim.pid.yaw_angle_ref, YAW_MIN - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb, YAW_MAX - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb);
//-------------------------------------------------------------//
//										    OutData[0]= (int)(gim.pid.pit_angle_ref *10);
//										    OutData[1]= (int)(gim.pid.pit_angle_fdb*10);
//										    OutData[0]= (int)(gim.pid.pit_angle_ref *10);
//										    OutData[1]= (int)(gim.pid.pit_angle_fdb*10);
										    OutData[2]= (int)(gim.pid.yaw_angle_ref *10);
										    OutData[3]= (int)(gim.pid.yaw_angle_fdb*10);								
    }
    break;
    default:
      break;
    }
}

#if STANDARD == 3
float Yaw_remain = -1.7;//向右为正
float pitch_remain=6;//5.2;
#elif STANDARD == 4
float Yaw_remain = -0.2;//向右为正
float pitch_remain=6.15;//5.2;
#elif STANDARD == 5
float Yaw_remain = -1.3;//向右为正
float pitch_remain=2.9;//5.2;
#elif STANDARD == 6
float Yaw_remain = -1.3;//向右为正
float pitch_remain=2.9;//5.2;

#endif
u32 lost_aim_time = 0;
float remain_yaw_angle = 0.0f;
float remain_pitch_angle = 0.0f;

void auto_small_buff_handle(void)
{
  gim.pid.pit_angle_fdb = pitch_Angle;
  gim.pid.yaw_angle_fdb =  yaw_Angle;
  if(new_location.flag == 1)   // 未识别时收到的数据是0x60
    {
      if(new_location.receNewDataFlag == 1)
        {
          pitch_remain=6+key_f_pitch*0.1;
					Yaw_remain=-1.7+key_e_yaw*0.1;
          Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( ( (double) new_location.x ) * TARGET_SURFACE_LENGTH,FOCAL_LENGTH);
          Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( ( (double) new_location.y ) * TARGET_SURFACE_WIDTH,FOCAL_LENGTH);
          Delta_Dect_Angle_Pit += pitch_remain;
          Delta_Dect_Angle_Yaw += Yaw_remain;
          gim.pid.yaw_angle_ref =  Delta_Dect_Angle_Yaw + gim.pid.yaw_angle_fdb;
          gim.pid.pit_angle_ref =  Delta_Dect_Angle_Pit + gim.pid.pit_angle_fdb;
          remain_pitch_angle = gim.pid.pit_angle_ref;
          remain_yaw_angle = gim.pid.yaw_angle_ref;
          new_location.receNewDataFlag = 0;
        }
      else
        {
          gim.pid.yaw_angle_ref = remain_yaw_angle;
          gim.pid.pit_angle_ref =	remain_pitch_angle;
        }
    }
  else
    {

    }
  //-----------------------------云台限幅-----------------------//
  VAL_LIMIT(gim.pid.pit_angle_ref, PITCH_MIN + (GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb, PITCH_MAX + 5 + (GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb);
//		VAL_LIMIT(gim.pid.yaw_angle_ref, YAW_MIN - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb, YAW_MAX - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb);
//-------------------------------------------------------------//
}

void auto_big_buff_handle(void)
{
  gim.pid.pit_angle_fdb = pitch_Angle;
  gim.pid.yaw_angle_fdb =  yaw_Angle;
  if(new_location.flag == 1)   // 未识别时收到的数据是0x60
    {
      if(new_location.receNewDataFlag == 1)
        {
          Delta_Dect_Angle_Yaw = RAD_TO_ANGLE * atan2 ( ( (double) new_location.x ) * TARGET_SURFACE_LENGTH,FOCAL_LENGTH);
          Delta_Dect_Angle_Pit = RAD_TO_ANGLE * atan2 ( ( (double) new_location.y ) * TARGET_SURFACE_WIDTH,FOCAL_LENGTH);

          Delta_Dect_Angle_Pit += pitch_remain;
          Delta_Dect_Angle_Yaw += Yaw_remain;
          gim.pid.yaw_angle_ref =  Delta_Dect_Angle_Yaw + gim.pid.yaw_angle_fdb;
          gim.pid.pit_angle_ref =  Delta_Dect_Angle_Pit + gim.pid.pit_angle_fdb;
          remain_pitch_angle = gim.pid.pit_angle_ref;
          remain_yaw_angle = gim.pid.yaw_angle_ref;
          new_location.receNewDataFlag = 0;
        }
      else
        {
          gim.pid.yaw_angle_ref = remain_yaw_angle;
          gim.pid.pit_angle_ref =	remain_pitch_angle;
        }
    }
  else
    {

    }
  //-----------------------------云台限幅-----------------------//
  VAL_LIMIT(gim.pid.pit_angle_ref, PITCH_MIN + (GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb, PITCH_MAX + 5 + (GMPitchEncoder.ecd_angle)+gim.pid.pit_angle_fdb);
//		VAL_LIMIT(gim.pid.yaw_angle_ref, YAW_MIN - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb, YAW_MAX - (-GMYawEncoder.ecd_angle)+gim.pid.yaw_angle_fdb);
//-------------------------------------------------------------//
}


void gimbal_follow_chassis_handle(void)
{
  gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;
  gim.pid.yaw_angle_fdb = -GMYawEncoder.ecd_angle;
  gim.pid.yaw_angle_ref = -chassis.position_ref;
  gim.pid.pit_angle_ref = 0;

}



float yaw_angle_offset = 0;
float pit_angle_offset = 0;

void auto_angle_handle(void)
{

  gim.pid.pit_angle_fdb = GMPitchEncoder.ecd_angle;
  gim.pid.yaw_angle_fdb = -GMYawEncoder.ecd_angle;
  gim.pid.yaw_angle_ref = -chassis.position_ref + yaw_angle_offset;
  gim.pid.pit_angle_ref = shoot_angle + pit_angle_offset;



}

//根据回传的距离，弹速和pitch轴角度来得出俯仰角
float raw_data_to_pitch_angle()
{
  int shoot_angle_speed;
  float distance_s;
  float distance_x;
  float distance_y;
  float x1;
  float x2;
  float x3;
  float x4;
  float angle_tan;
  float shoot_radian;
  float shoot_angle;

  shoot_angle_speed=judge_rece_mesg.shoot_data.bullet_speed;
  distance_s=(Gimbal_Auto_Shoot.Distance-7)/100;
  distance_x=(cos((GMPitchEncoder.ecd_angle)*ANGLE_TO_RAD)*distance_s);
  distance_y=(sin((GMPitchEncoder.ecd_angle)*ANGLE_TO_RAD)*distance_s);

  x1=shoot_angle_speed*shoot_angle_speed;
  x2=distance_x*distance_x;
  x3=sqrt(x2-(19.6*x2*((9.8*x2)/(2*x1)+distance_y))/x1);
  x4=9.8*x2;
  angle_tan=(x1*(distance_x-x3))/(x4);
  shoot_radian=atan(angle_tan);
  shoot_angle=shoot_radian*RAD_TO_ANGLE;
  return shoot_angle;
}


void gimbal_param_init(void)
{
  memset(&gim, 0, sizeof(gimbal_t));
  gim.ctrl_mode      		 = GIMBAL_RELAX;
  gim.last_ctrl_mode 		 = GIMBAL_RELAX;
  gim.input.ac_mode        = NO_ACTION;
  gim.input.action_angle   = 5.0f;


#if STANDARD == 3
//标识3号码步兵

    PID_struct_init(&pid_pit, POSITION_PID, 800, 10,
                     16,0.01f,8); //8.0f, ,40    15,0.003f,5   18    16  5
    PID_struct_init(&pid_pit_speed, POSITION_PID, 27000, 25000,
                    200, 0.2, 0);//   170, 0.001f, 60--160 8 0    ->  180 10 50  ->190 12 70  ->120 8 0-》180.0f, 10.0f, 30 -> 235, 0.002f, 45
										//15 0.01 0...120 0 0       180 0.2 20
    //------------------------------------------------
     PID_struct_init(&pid_yaw, POSITION_PID, 1000, 0,
                    15, 0.1f, 25); //10.0f, 0, 100  13
     PID_struct_init(&pid_yaw_speed, POSITION_PID, 29000, 25000,
                  250,0.1f,10);//250.0f,6.0f, 50-->300.0f,5.0f, 70
	

#elif STANDARD ==4
////标识4号码步兵

	//	//----------------小陀螺P轴陀螺仪------------------------
 PID_struct_init(&pid_pit, POSITION_PID, 800, 10,
                   15,0.01f,8); //16 0.01 5
 PID_struct_init(&pid_pit_speed, POSITION_PID, 27000, 25000,
                   200,0.3,18);//260 0.03 60   ->  180 10 50  ->190 12 70

    //------------------------------------------------

     PID_struct_init(&pid_yaw, POSITION_PID, 1000, 0,
                     12, 0.17, 25); //7 0 10
     PID_struct_init(&pid_yaw_speed, POSITION_PID, 29000, 25000,
                    247,0.2,10);//180  2 
#elif STANDARD == 5
////标识5号码步兵
//----------------小陀螺P轴陀螺仪------------------------
    PID_struct_init(&pid_pit, POSITION_PID, 800, 10,
                     15,0.01f,8); //8.0f, ,40    15,0.003f,5   18    16  5
    PID_struct_init(&pid_pit_speed, POSITION_PID, 27000, 25000,
                    200, 0.3f, 18);//   170, 0.001f, 60--160 8 0    ->  180 10 50  ->190 12 70  ->120 8 0-》180.0f, 10.0f, 30 -> 235, 0.002f, 45
										//15 0.01 0...120 0 0       180 0.2 20
    //------------------------------------------------
     PID_struct_init(&pid_yaw, POSITION_PID, 1000, 0,
                    15, 0, 0); //10.0f, 0, 100  13
     PID_struct_init(&pid_yaw_speed, POSITION_PID, 29000, 25000,
                  20,0,0);//250.0f,6.0f, 50-->300.0f,5.0f, 70


#elif STANDARD == 6
////标识5号码步兵

  //	//----------------小陀螺P轴陀螺仪------------------------
  PID_struct_init(&pid_pit, POSITION_PID, 800, 10,
                  15,0.01f,8); //16 0.01 5
  PID_struct_init(&pid_pit_speed, POSITION_PID, 27000, 25000,
                  200,0.3,18);//260 0.03 60   ->  180 10 50  ->190 12 70

  //------------------------------------------------

  PID_struct_init(&pid_yaw, POSITION_PID, 1000, 0,
                  12, 0.17, 25); //7 0 10
  PID_struct_init(&pid_yaw_speed, POSITION_PID, 29000, 25000,
                  247,0.2,10);//180  2
#endif




#if STANDARD == 3
  /********************小符下的PID参数（3号）****************/

//    8.4调
  PID_struct_init(&pid_pit_small_buff, POSITION_PID, 70, 50,
                  25.0f, 0.5f, 300);  //60  60 16 0.25 150           28 0.5 160
  PID_struct_init(&pid_pit_speed_small_buff, POSITION_PID, 25000, 2000,
                  180.0f, 0.00f, 0);//   
  PID_struct_init(&pid_yaw_small_buff, POSITION_PID, 60, 30,
                  18.0f, 0.5f, 200 ); //               18 0.4 180
  PID_struct_init(&pid_yaw_speed_small_buff, POSITION_PID, 25000, 25000,
                  250.0f,0.0f,0);            




  /****************************************************/

#elif STANDARD == 4
  /********************小符下的PID参数（4号）****************/
  PID_struct_init(&pid_pit_small_buff, POSITION_PID, 70, 50,
                  25.0f, 0.5f, 200);  //60  60 16 0.25 150           28 0.5 160
  PID_struct_init(&pid_pit_speed_small_buff, POSITION_PID, 25000, 2000,
                  180.0f, 0.00f, 0);//
  PID_struct_init(&pid_yaw_small_buff, POSITION_PID, 60, 30,
                  25.0f, 0.5f, 230 ); //               18 0.4 180
  PID_struct_init(&pid_yaw_speed_small_buff, POSITION_PID, 25000, 25000,
                  250.0f,0.0f,0);
  /****************************************************/

#elif STANDARD == 5
  /********************小符下的PID参数（5号）****************/


  PID_struct_init(&pid_pit_small_buff, POSITION_PID, 200, 10,
                  6.0f, 0, 110);  //
  PID_struct_init(&pid_pit_speed_small_buff, POSITION_PID, 27000, 25000,
                  180.0f, 5.0f, 0);//160 8 0    ->  180 10 50  ->190 12 70



  PID_struct_init(&pid_yaw_small_buff, POSITION_PID, 250, 0,
                  7.4f, 0, 110 ); //
  PID_struct_init(&pid_yaw_speed_small_buff, POSITION_PID, 25000, 5000,
                  180.0f,4.0f,0);
  /****************************************************/

#elif STANDARD == 6
  /********************小符下的PID参数（6号）****************/

  PID_struct_init(&pid_pit_small_buff, POSITION_PID, 70, 50,
                  30.0f, 0.5f, 200);  //60  60 16 0.25 150           28 0.5 160
  PID_struct_init(&pid_pit_speed_small_buff, POSITION_PID, 25000, 2000,
                  180.0f, 0.00f, 0);//   
  PID_struct_init(&pid_yaw_small_buff, POSITION_PID, 60, 30,
                  18.0f, 0.5f, 200 ); //               18 0.4 180
  PID_struct_init(&pid_yaw_speed_small_buff, POSITION_PID, 25000, 25000,
                  250.0f,0.0f,0);            

  /****************************************************/

#endif

  /********************大符下的PID参数（3号）****************/


#if STANDARD == 3

  PID_struct_init(&pid_pit_big_buff, POSITION_PID, 200, 10,
                  9.0f, 0, 70);  //
  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                  160.0f, 4.0f, 0);//160 8 0    ->  180 10 50  ->190 12 70



  PID_struct_init(&pid_yaw_big_buff, POSITION_PID, 250, 0,
                  9.0f, 0, 110 ); //
  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 25000, 5000,
                  200.0f,4.0f,0);

#elif STANDARD == 4
  PID_struct_init(&pid_pit_big_buff, POSITION_PID, 200, 10,
                  11.0f, 0.00f, 0);  //
  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                  170.0f, 0.0f, 0);//160 8 0    ->  180 10 50  ->190 12 70
  PID_struct_init(&pid_yaw_big_buff, POSITION_PID, 250, 0,
                  10.0f, 0, 0 ); //
  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 25000, 25000,
                  100.0f,0.0f,0);


#elif STANDARD == 5
  PID_struct_init(&pid_pit_big_buff, POSITION_PID, 200, 10,
                  10.0f, 0, 90);  //
  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                  150.0f, 3.0f, 60);//160 8 0    ->  180 10 50  ->190 12 70



  PID_struct_init(&pid_yaw_big_buff, POSITION_PID, 250, 0,
                  10.0f, 0, 100 ); //
  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 25000, 25000,
                  160.0f,3.0f,50);

#elif STANDARD == 6
  PID_struct_init(&pid_pit_big_buff, POSITION_PID, 200, 10,
                  10.0f, 0, 90);  //
  PID_struct_init(&pid_pit_speed_big_buff, POSITION_PID, 27000, 25000,
                  150.0f, 3.0f, 60);//160 8 0    ->  180 10 50  ->190 12 70



  PID_struct_init(&pid_yaw_big_buff, POSITION_PID, 250, 0,
                  10.0f, 0, 100 ); //
  PID_struct_init(&pid_yaw_speed_big_buff, POSITION_PID, 25000, 25000,
                  160.0f,3.0f,50);
#endif

  /****************************************************/


  /********************吊射下的PID参数****************/

  PID_struct_init(&pid_pit_auto_angle, POSITION_PID, 200, 10,
                  12.0f, 0, 90);  //
  PID_struct_init(&pid_pit_speed_auto_angle, POSITION_PID, 27000, 25000,
                  250.0f, 5.0f, 60);//160 8 0    ->  180 10 50  ->190 12 70



  PID_struct_init(&pid_yaw_auto_angle, POSITION_PID, 250, 0,
                  13.0f, 0, 100 ); //
  PID_struct_init(&pid_yaw_speed_auto_angle, POSITION_PID, 25000, 25000,
                  160.0f,5.0f,30);

  /****************************************************/


  /********************自动补弹下YAW的PID参数****************/


  PID_struct_init(&pid_yaw_follow_chassis_angle, POSITION_PID, 250, 0,
                  13.0f, 0, 100 ); //
  PID_struct_init(&pid_yaw_follow_chassis_speed, POSITION_PID, 25000, 25000,
                  160.0f,5.0f,30);

  /****************************************************/

  /********************自瞄下的PID参数****************/
#if STANDARD == 3
  PID_struct_init ( &pid_pit_follow, POSITION_PID, 200, 10, 8
                    , 0.01, 8 ); //200, 10, 10 , 0.005f, 80.0f   200, 10, 10 , 0.005, 80.0f
  PID_struct_init ( &pid_pit_speed_follow, POSITION_PID, 27000, 25000, 180.0f, 0.2f, 0 ); //160 8 0    ->  180 10 50  ->190 12 70     27000, 25000,100.0f, 0.0007, 5


//     PID_struct_init(&pid_yaw_follow, POSITION_PID, 400, 15,12, 0.001f, 0 ); //
//     PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 29000, 5000,250,5, 10);
//
  PID_struct_init ( &pid_yaw_follow, POSITION_PID,  150,200,
                    20, 0.09f, 40 );
  PID_struct_init ( &pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                    300.0f, 0, 100 ); //I太大时，陀螺开启云台抖动严重
#elif STANDARD == 4
  PID_struct_init(&pid_pit_follow, POSITION_PID, 100, 10, 10, 0.001f, 60.0f);  //
  PID_struct_init(&pid_pit_speed_follow, POSITION_PID, 27000, 25000,180.0f, 0.8, 150);//160 8 0    ->  180 10 50  ->190 12 70
  PID_struct_init(&pid_yaw_follow, POSITION_PID, 1000, 1000, 13.0f, 0.017f, 8.0);
  PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                  80.0f,0.01f, 10);//I太大时，陀螺开启云台抖动严重


#elif STANDARD == 5
  PID_struct_init(&pid_pit_follow, POSITION_PID, 200, 10, 10, 0.005f, 80.0f);  //
  PID_struct_init(&pid_pit_speed_follow, POSITION_PID, 27000, 25000,120.0f, 6, 150);//160 8 0    ->  180 10 50  ->190 12 70

  PID_struct_init(&pid_yaw_follow, POSITION_PID, 1000, 1000,
                  6.0f, 0.00f, 150);
  PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                  250.0f,2.0f, 0 );//I太大时，陀螺开启云台抖动严重

#elif STANDARD == 6
  PID_struct_init(&pid_pit_follow, POSITION_PID, 200, 10, 10, 0.005f, 80.0f);  //
  PID_struct_init(&pid_pit_speed_follow, POSITION_PID, 27000, 25000,120.0f, 6, 150);//160 8 0    ->  180 10 50  ->190 12 70

  PID_struct_init(&pid_yaw_follow, POSITION_PID, 1000, 1000,
                  6.0f, 0.00f, 150);
  PID_struct_init(&pid_yaw_speed_follow, POSITION_PID, 29800, 29800,
                  250.0f,2.0f, 0);//I太大时，陀螺开启云台抖动严重

#endif


}

void gimbal_back_param(void)
{
  //斜坡初始化
  GMPitchRamp.SetScale(&GMPitchRamp, BACK_CENTER_TIME / GIMBAL_PERIOD);
  GMYawRamp.SetScale(&GMYawRamp, BACK_CENTER_TIME / GIMBAL_PERIOD);
  GMPitchRamp.ResetCounter(&GMPitchRamp);
  GMYawRamp.ResetCounter(&GMYawRamp);
  //云台给定角度初始化
  GimbalRef.pitch_angle_dynamic_ref = 0.0f;
  GimbalRef.yaw_angle_dynamic_ref   = 0.0f;
}



float AvgFilter(float new_value)
{
	float avg_value;
	float sum = 0;
	static float value[FILTER_NUM] = {0};
	for(int i = 0; i < FILTER_NUM - 1 ; i ++)
	{
			value[i] = value[i + 1];
			sum += value[i];
	}
	value[FILTER_NUM - 1] = new_value;
	sum += value[FILTER_NUM - 1];
	avg_value = sum / FILTER_NUM;
	return avg_value;
}
