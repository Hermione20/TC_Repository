#ifndef  __BALANCE_TASK_H
#define  __BALANCE_TASK_H
#include "public.h"

#define WHEEL_R 0.0675
#define BODY_MASS 8.438
#define WHEEL_MASS 2.268
#define T_MAX 10 
#define TIME_STEP 2


/**
************************************************************************************************************************
* @EnumName : chassis_mode_e
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
  CHASSIS_INIT 			 = 2,
  MANUAL_FOLLOW_GIMBAL   = 3,
  CHASSIS_STAND_MODE             = 4,
  AUTO_SEPARATE_GIMBAL   = 5,
  AUTO_FOLLOW_GIMBAL     = 6,
  CHASSIS_ROTATE         = 7,
  CHASSIS_REVERSE        = 8,
  CHASSIS_CHANGE_REVERSE = 9,
  CHASSIS_SEPARATE 		 	 = 10,
  CHASSIS_AUTO_SUP       = 11,
} chassis_mode_e;


typedef struct
{
	float theta;
	float dtheta;
	float ddtheta;
	float phi;
	float dphi;
	float x;
	float dx;
	float ddz;

	double L0;
	double K[12];
	float k[2][6];
	float state_err[6];

	double lqrOutT;
	double lqrOutTp;
}lqr_system;

typedef struct
{
	float vx;
	float y_position;
	float vy;
	float vw;
	float roll;
	float theta;
	float leglength;
}chassis_ref_t;

#ifndef LEG_STATE
#define LEG_STATE

typedef struct
{
	float pos[2];//pos=[l0; phi0];
	float spd[2];//spd[2]=[dl0; dphi0];
	float T[2];//T[2]=[motor4;motor1];

	//֧���������ü������
	float J[4];
	float j[2][2];
	float F_fdb;
	float Tp_fdb;

	float phi4;
	float phi1;
	float dphi4;
	float dphi1;

	float this_dl0;
	float last_dl0;

	float l0;
	float dl0;
	float ddl0;
	float phi0;
	float dphi0;

	float leg_F;
	float ddzw;
	float leg_FN;

	uint8_t wheel_state;

	pid_t leglengthpid;
}leg_state_t;

#endif // !LEG_STATE




typedef struct
{
	chassis_mode_e ctrl_mode;
	chassis_mode_e last_ctrl_mode;

	lqr_system balance_loop;
	chassis_ref_t chassis_ref;
	chassis_ref_t chassis_dynemic_ref;

	leg_state_t left_leg;
	leg_state_t right_leg;

	pid_t leg_harmonize_pid;
	pid_t vw_pid;
	pid_t roll_pid;
	
	pid_t Init_Tp_pid;

	//tqouce
	double joint_T[4];
	double driving_T[2];
}Balance_chassis_t;

void chassis_Init_handle(void);
void chasis_standup_handle(void);
void chassis_seperate_handle(void);
void balance_chassis_task(void);
void balance_task(void);
void balance_param_init(void);
void lqr_k(double L0, double K[12]);
uint8_t wheel_state_estimate(leg_state_t* leg);

extern Balance_chassis_t b_chassis;


#endif // ! __BALANCE_TASK_H