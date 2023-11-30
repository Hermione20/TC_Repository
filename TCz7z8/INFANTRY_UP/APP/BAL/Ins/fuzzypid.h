
#ifndef __FUZZYPID_H__
#define __FUZZYPID_H__
#include "main.h"


#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3

//注意：static PID pid= {0, 0, 0};  需要自己赋值

static const float fuzzyRuleKp[7][7]={
	PL,	PL,	PM,	PM,	PS,	PS,	ZE,
	PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
	PM,	PM,	PM,	PS,	ZE,	NS,	NM,
	PM,	PS,	PS,	ZE,	NS,	NM,	NM,
	PS,	PS,	ZE,	NS,	NS,	NM,	NM,
	ZE,	ZE,	NS,	NM,	NM,	NM,	NL,
	ZE,	NS,	NS,	NM,	NM,	NL,	NL
};

static const float fuzzyRuleKi[7][7]={
	NL,	NL,	NL,	NM,	NM,	ZE,	ZE,
	NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
	NM,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	ZE,	PS,	PS,	PM,
	NS,	NS,	ZE,	PS,	PS,	PM,	PM,
	ZE,	ZE,	PS,	PM,	PM,	PL,	PL,
	ZE,	ZE,	PS,	PM,	PL,	PL,	PL
};

static const float fuzzyRuleKd[7][7]={
	PS,	PS,	ZE,	ZE,	ZE,	PL,	PL,
	NS,	NS,	NS,	NS,	ZE,	NS,	PM,
	NL,	NL,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NM,	NS,	ZE,	PS,	PM,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NS,	NS,	NS,	ZE,	PS,	PS,
	PS,	ZE,	ZE,	ZE,	ZE,	PL,	PL
};

typedef struct fuzzy_pid_t
{
  float Kp_Init;
	float Kd_Init;
	float Ki_Init;
	
	float fuzzy_kp;
  float fuzzy_ki;
  float fuzzy_kd;

  float set;
  float get;
  float erro[3];
	float erro_of_change;
	int   sum_of_erro;                   //累加偏差

	float erro_max; //用于进行e/ec的映射，调整其有效范围,默认[-3,3]
	float erro_min;
	float erro_of_change_max;
	float erro_of_change_min;
	
	float kp_max;
	float kp_min;
	float ki_max;
	float ki_min;
	float kd_max;
	float kd_min;
		
  float fuzzy_pout;
  float fuzzy_iout;
  float fuzzy_dout;
  float fuzzy_out;

  float input_max_err;    //input max err;
  float output_deadband;  //output deadband; 
  
	
	float erro_middle,erro_of_change_middle; 																		 //	对应区间中值
	float erro_mapped,erro_of_change_mapped;																		 //区间映射		
	float erro_left_grade_of_membership,erro_of_change_left_grade_of_membership;   //映射后误差的左右隶属度
	float erro_right_grade_of_membership,erro_of_change_right_grade_of_membership;

	int erro_left_index,erro_of_change_left_index;                                 //映射后误差的左右索引值
	int erro_right_index,erro_of_change_right_index;                         
	
  uint32_t pid_mode;
  uint32_t max_out;
  uint32_t integral_limit;

  void (*fuzzy_param_init)(struct fuzzy_pid_t *pid, 
                       uint32_t      pid_mode,
                       uint32_t      max_output,
                       uint32_t      inte_limit,
                       float         fuzzy_kp,
                       float         fuzzy_ki,
                       float         fuzzy_kd,
											 float 				 kp_max,
											 float         kp_min,
											 float 				 ki_max,
											 float 				 ki_min,
											 float 				 kd_max,
											 float 				 kd_min,
											 float 				 erro_max,
											 float 			   erro_min,
											 float 				 erro_of_change_max,
											 float 				 erro_of_change_min 
											 );
  void (*fuzzy_pid_reset)(struct fuzzy_pid_t *pid, float fuzzy_kp, float fuzzy_ki, float fuzzy_kd,float kp_max,float kp_min,float ki_max,float ki_min,float kd_max,float kd_min,float erro_max,float erro_min,float erro_of_change_max,float erro_of_change_min);
 
} fuzzy_pid_t;


#define FuzzyPidMotor_OpenPwm 3500 // 电机死区
#define FuzzyPidTarge_Error 0 // 目标值允许误差


void fuzzy_process(fuzzy_pid_t *fuzzy_pid,float erro,float erro_of_change);
float Mapping(float maximum,float minimum,float x);
float Inverse_mapping(float maximum, float minimum, float map);
float fuzzy_pid_calc(fuzzy_pid_t *fuzzy_pid,float get,float set);
static void pid_reset(fuzzy_pid_t *pid, float kp, float ki, float kd);
static void fuzzy_pid_param_init(
    fuzzy_pid_t*             pid,
    uint32_t                mode,
    uint32_t              maxout,
    uint32_t     intergral_limit,
    float    kp,
    float    ki,
    float    kd,
		float kp_max,
		float kp_min,
		float ki_max,
		float ki_min,
		float kd_max,
		float kd_min,
		float erro_max,
		float erro_min,
		float erro_of_change_max,
		float erro_of_change_min    		
		);

void fuzzy_pid_struct_init(
    fuzzy_pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd,
			
		float kp_max,
		float kp_min,
		float ki_max,
		float ki_min,
		float kd_max,
		float kd_min,
		float erro_max,
		float erro_min,
		float erro_of_change_max,
		float erro_of_change_min
		);

#endif

