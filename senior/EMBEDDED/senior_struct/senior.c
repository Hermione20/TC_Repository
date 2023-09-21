#include "senior.h"

/* Variables_definination-----------------------------------------------------------------------------------------------*/


/*******************************general_gyro_define************************************/
general_gyro_t gimbal_gyro;
general_gyro_t chassis_gyro;
/******************************general_chassis_define**********************************/
steering_wheel_t steering_wheel_chassis = {0};
Mecanum_wheel_t Mecanum_chassis  = {0};
/*******************************general_gimbal_define**********************************/
volatile Encoder Pitch_Encoder = {0};
volatile Encoder yaw_Encoder = {0};
hero_small_gimbal_t hero_small_gimbal = {0};
/*********************************friction_encoder*************************************/
friction_t general_friction = {0};
/***********************************poke_encoder***************************************/
poke_t general_poke = {0};
/*********************************capacitance_define***********************************/
volatile capacitance_message_t capacitance_message;
/***********************************judge define***************************************/
receive_judge_t judge_rece_mesg;
/*********************************auto_shoot_define************************************/
location new_location;
/**********************************remote_define***************************************/
RC_Ctl_t RC_CtrlData;


/*----------------------------------------------------------------------------------------------------------------------*/

