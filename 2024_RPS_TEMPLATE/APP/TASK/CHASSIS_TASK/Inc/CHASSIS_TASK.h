/* Includes ------------------------------------------------------------------*/
#include "public.h"
/*----------------------------------------------------------------------------*/

/**
************************************************************************************************************************
* @EnumName     : chassis_mode_e
* @brief    : This enumeration describes the various control modes of the chassis
* @param    : CHASSIS_RELAX     
*	@param		:	CHASSIS_STOP		
*	@param		:	MANUAL_SEPARATE_GIMBAL			
* @param		: MANUAL_FOLLOW_GIMBAL 		
* @param		: DODGE_MODE 		
* @param		: AUTO_SEPARATE_GIMBAL 	
* @param		: AUTO_FOLLOW_GIMBAL 	
* @param		: CHASSIS_ROTATE 		
* @param		: CHASSIS_REVERSE 	
* @param		: CHASSIS_CHANGE_REVERSE 		
* @param		: CHASSIS_SEPARATE 	
* @param		: CHASSIS_AUTO_SUP 		
* @Note     : 					
************************************************************************************************************************
**/
typedef enum
{
  CHASSIS_RELAX          = 0,
  CHASSIS_STOP           = 1,
  MANUAL_SEPARATE_GIMBAL = 2,
  MANUAL_FOLLOW_GIMBAL   = 3,
  DODGE_MODE             = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
  CHASSIS_ROTATE         = 7,
  CHASSIS_REVERSE        = 8,
  CHASSIS_CHANGE_REVERSE = 9,
  CHASSIS_SEPARATE 		 	 = 10,
  CHASSIS_AUTO_SUP       = 11,
} chassis_mode_e;

/**
************************************************************************************************************************
* @StructName : cha_pid_t
* @brief    	: This enumeration describes the various control modes of the chassis
* @param    	: anglex_ref    		
*	@param			:	anglex_fdb			
* @param			: speedx_ref 		
* @param			: speedx_fdb 		 		
* @Note    		: 					
************************************************************************************************************************
**/
typedef struct
{
  /* position loop */
  float angle1_ref;
  float angle2_ref;
  float angle1_fdb;
  float angle2_fdb;
	
	float angle3_ref;
  float angle4_ref;
  float angle3_fdb;
  float angle4_fdb;
  /* speed loop */
  float speed1_ref;
  float speed2_ref;
  float speed1_fdb;
  float speed2_fdb;

  float speed3_ref;
  float speed4_ref;
  float speed3_fdb;
  float speed4_fdb;
} cha_pid_t;

/**
************************************************************************************************************************
* @StructName : cha_pid_t
* @brief    	: This enumeration describes the various control modes of the chassis
* @param    	: anglex_ref    		
*	@param			:	anglex_fdb			
* @param			: speedx_ref 		
* @param			: speedx_fdb 		 		
* @Note    		: 					
************************************************************************************************************************
**/
typedef struct
{
		double           vx; // forward/back
		double           vy; // left/right
		double           vw; // 
		
		chassis_mode_e  ctrl_mode;
		chassis_mode_e  last_ctrl_mode;

		float           gyro_angle;
		float           gyro_palstance;

		int16_t         wheel_speed_fdb[4];
		int16_t         wheel_speed_ref[4];
		int16_t         current[4];
		
		float						sin_chassis_angle;
		float						cos_chassis_angle;
		
		int16_t					foward_back_to_foward_back_rotate_speed;
		int16_t					foward_back_to_left_right_rotate_speed;
		int16_t					left_right_to_foward_back_rotate_speed;
		int16_t					left_right_to_left_right_rotate_speed;
		
		int32_t         position_ref;
		uint8_t         follow_gimbal;
		
		cha_pid_t      cha_pid;
} chassis_t;

/**
************************************************************************************************************************
* @StructName : cha_pid_t
* @brief    	: This enumeration describes the various control modes of the chassis
* @param    	: anglex_ref    		
*	@param			:	anglex_fdb			
* @param			: speedx_ref 		
* @param			: speedx_fdb 		 		
* @Note    		: 					
************************************************************************************************************************
**/
typedef struct
{
		float start_angle[4];
		float include_angle[4];
		float Remote_angle;
		float Remote_speed;
		float deviation_angle[4];
		int16_t handle_speed[4];
		float get_speedw;
		float get_yaw_angle;
}Chassis_angle_t;




void get_remote_set(void);










